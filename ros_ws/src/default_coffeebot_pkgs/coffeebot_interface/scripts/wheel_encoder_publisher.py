#!/usr/bin/env python

from utils import general_init, get_left_encoder_vals, get_right_encoder_vals, general_cleanup
from nuturtlebot.msg import SensorData
import rospy
import RPi.GPIO as GPIO
import numpy as np


Left_Phase_A = 7      #(left motor phase A)
Right_Phase_A = 6     #(right motor phase A)
Left_Phase_B = 12    #(left motor phase B)
Right_Phase_B = 17 #(right phaseB)
RIGHT_ENCODER_RESOLUTION = 362 #361.8  #(encoder ticks, according to the datasheet, it's 374)
LEFT_ENCODER_RESOLUTION = 367 #367.39  #(encoder ticks, according to the datasheet, it's 374)

class WheelEncodersPublisher(object):
    def __init__(self):
        general_init()
        self.encoder_measurement_init()
        self.robot_param_init()
        self.publisher_init()
        rospy.loginfo("Wheel Encoder Publisher initialized successfully ")
    def __del__(self):
        general_cleanup()

    def encoder_measurement_init(self):
        GPIO.setup(Left_Phase_A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(Right_Phase_A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(Left_Phase_B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(Right_Phase_B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        """Initialize interrupt-driven encoder callbacks"""
        GPIO.add_event_detect(Right_Phase_A, GPIO.RISING, self.encoder_counter)
        GPIO.add_event_detect(Left_Phase_B, GPIO.RISING, self.encoder_counter)  #Note we cannot use Left_Phase_A because of other functions on this pin. This is why we need two seaparate encoder counters.
        self.right_count = 0
        self.last_right_count = 0
        self.left_count = 0
        self.last_left_count = 0

    def robot_param_init(self):
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.044)

    def publisher_init(self):
        self.pub = rospy.Publisher("wheel_linear_vel", SensorData)
        self.last_pub_time = rospy.Time.now()

    def get_encoder_linear_variation(self):
        """ Return the angular varation of the motors between two calls of this function.
        Note that we assume during two reads, the encoder won't rotate by more than half a revolution."""
        diff = self.right_count - self.last_right_count
        right_angular_var = 0
        self.right_count = self.last_right_count
        #Normalize diff, here we asssume during two reads, the encoder won't rotate by more than half a revolution.
        if np.abs(diff) > 0.5 * RIGHT_ENCODER_RESOLUTION:
            right_angular_var = np.sign(diff) * (RIGHT_ENCODER_RESOLUTION - np.abs(diff)) * 2.0 * np.pi/RIGHT_ENCODER_RESOLUTION
        else:
            right_angular_var = diff * 2.0 * np.pi/RIGHT_ENCODER_RESOLUTION

        diff = self.left_count - self.last_left_count
        left_angular_var = 0
        self.left_count = self.last_left_count
        #Normalize diff, here we asssume during two reads, the encoder won't rotate by more than half a revolution.
        if np.abs(diff) > 0.5 * LEFT_ENCODER_RESOLUTION:
            left_angular_var = np.sign(diff) * (LEFT_ENCODER_RESOLUTION - np.abs(diff)) * 2.0 * np.pi/LEFT_ENCODER_RESOLUTION
        else:
            left_angular_var = diff * 2.0 * np.pi/LEFT_ENCODER_RESOLUTION

        left_linear_var = left_angular_var * self.wheel_radius
        right_linear_var = right_angular_var * self.wheel_radius
        rospy.loginfo("left vel: %f | right vel: %f ", left_linear_var, right_linear_var)
        return (left_linear_var, right_linear_var)

    def encoder_counter(self, pin):
        if (pin == Right_Phase_A):
            vals = get_right_encoder_vals()
            self.right_count += (self.encounter_variation_right(A_val=vals[0], B_val=vals[1]))
            # Angle wrapping: half angle
            if np.abs(self.right_count) > 0.5 * RIGHT_ENCODER_RESOLUTION:
                self.right_count = np.abs(self.right_count) * (np.abs(self.right_count) - RIGHT_ENCODER_RESOLUTION)
        else:
            vals = get_left_encoder_vals()
            self.left_count += (self.encounter_variation_left(A_val=vals[0], B_val=vals[1]))
            # Angle wrapping: half angle
            if np.abs(self.left_count) > 0.5 * LEFT_ENCODER_RESOLUTION:
                self.left_count = np.abs(self.left_count) * (np.abs(self.left_count) - LEFT_ENCODER_RESOLUTION)


    def encounter_variation_right(self, A_val, B_val):
        """
        Low level helper funtion to a single increment or decrement of the right encoder on rising edges. (we read from Phase A)
        Here Phase A should be 90 electrical degree ahead of Phase B
        Inspired by: https://www.rs-online.com/designspark/detecting-quadrature-encoders-with-arduino-uno
        :param A_val: Phase A value of the encoder
        :param B_val: Phase B value of the encoder
        :return: a single encoder . Positive means CW, negative means CCW
        """
        return 1 if B_val == A_val else -1

    def encounter_variation_left(self, A_val, B_val):
        """
        Low level helper funtion to a single increment or decrement of the left encoder on rising edges. (we read from phase B)
        Here Phase A should be 90 electrical degree ahead of Phase B
        Inspired by: https://www.rs-online.com/designspark/detecting-quadrature-encoders-with-arduino-uno
        :param A_val: Phase A value of the encoder
        :param B_val: Phase B value of the encoder
        :return: a single encoder . Positive means CW, negative means CCW
        """
        return -1 if B_val == A_val else 1

    def publish_wheel_linear_vel(self):
        """ Publish each wheel velocity in m/s """
        msg = SensorData()
        msg.left_encoder, msg.right_encoder = self.get_encoder_linear_variation()
        msg.left_encoder /= (rospy.Time.now() - self.last_pub_time)
        msg.right_encoder /= (rospy.Time.now() - self.last_pub_time)
        msg.stamp = rospy.Time.now()
        self.last_pub_time = rospy.Time.now()
        self.pub.publish(msg)