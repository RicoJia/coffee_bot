;; Auto-generated. Do not edit!


(when (boundp 'nuturtlebot::SensorData)
  (if (not (find-package "NUTURTLEBOT"))
    (make-package "NUTURTLEBOT"))
  (shadow 'SensorData (find-package "NUTURTLEBOT")))
(unless (find-package "NUTURTLEBOT::SENSORDATA")
  (make-package "NUTURTLEBOT::SENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude SensorData.msg.html


(defclass nuturtlebot::SensorData
  :super ros::object
  :slots (_stamp _left_encoder _right_encoder _accelX _accelY _accelZ _gyroX _gyroY _gyroZ _magX _magY _magZ _battery_voltage ))

(defmethod nuturtlebot::SensorData
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:left_encoder __left_encoder) 0)
    ((:right_encoder __right_encoder) 0)
    ((:accelX __accelX) 0)
    ((:accelY __accelY) 0)
    ((:accelZ __accelZ) 0)
    ((:gyroX __gyroX) 0)
    ((:gyroY __gyroY) 0)
    ((:gyroZ __gyroZ) 0)
    ((:magX __magX) 0)
    ((:magY __magY) 0)
    ((:magZ __magZ) 0)
    ((:battery_voltage __battery_voltage) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _left_encoder (round __left_encoder))
   (setq _right_encoder (round __right_encoder))
   (setq _accelX (round __accelX))
   (setq _accelY (round __accelY))
   (setq _accelZ (round __accelZ))
   (setq _gyroX (round __gyroX))
   (setq _gyroY (round __gyroY))
   (setq _gyroZ (round __gyroZ))
   (setq _magX (round __magX))
   (setq _magY (round __magY))
   (setq _magZ (round __magZ))
   (setq _battery_voltage (float __battery_voltage))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:left_encoder
   (&optional __left_encoder)
   (if __left_encoder (setq _left_encoder __left_encoder)) _left_encoder)
  (:right_encoder
   (&optional __right_encoder)
   (if __right_encoder (setq _right_encoder __right_encoder)) _right_encoder)
  (:accelX
   (&optional __accelX)
   (if __accelX (setq _accelX __accelX)) _accelX)
  (:accelY
   (&optional __accelY)
   (if __accelY (setq _accelY __accelY)) _accelY)
  (:accelZ
   (&optional __accelZ)
   (if __accelZ (setq _accelZ __accelZ)) _accelZ)
  (:gyroX
   (&optional __gyroX)
   (if __gyroX (setq _gyroX __gyroX)) _gyroX)
  (:gyroY
   (&optional __gyroY)
   (if __gyroY (setq _gyroY __gyroY)) _gyroY)
  (:gyroZ
   (&optional __gyroZ)
   (if __gyroZ (setq _gyroZ __gyroZ)) _gyroZ)
  (:magX
   (&optional __magX)
   (if __magX (setq _magX __magX)) _magX)
  (:magY
   (&optional __magY)
   (if __magY (setq _magY __magY)) _magY)
  (:magZ
   (&optional __magZ)
   (if __magZ (setq _magZ __magZ)) _magZ)
  (:battery_voltage
   (&optional __battery_voltage)
   (if __battery_voltage (setq _battery_voltage __battery_voltage)) _battery_voltage)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; int32 _left_encoder
    4
    ;; int32 _right_encoder
    4
    ;; int16 _accelX
    2
    ;; int16 _accelY
    2
    ;; int16 _accelZ
    2
    ;; int16 _gyroX
    2
    ;; int16 _gyroY
    2
    ;; int16 _gyroZ
    2
    ;; int16 _magX
    2
    ;; int16 _magY
    2
    ;; int16 _magZ
    2
    ;; float32 _battery_voltage
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _left_encoder
       (write-long _left_encoder s)
     ;; int32 _right_encoder
       (write-long _right_encoder s)
     ;; int16 _accelX
       (write-word _accelX s)
     ;; int16 _accelY
       (write-word _accelY s)
     ;; int16 _accelZ
       (write-word _accelZ s)
     ;; int16 _gyroX
       (write-word _gyroX s)
     ;; int16 _gyroY
       (write-word _gyroY s)
     ;; int16 _gyroZ
       (write-word _gyroZ s)
     ;; int16 _magX
       (write-word _magX s)
     ;; int16 _magY
       (write-word _magY s)
     ;; int16 _magZ
       (write-word _magZ s)
     ;; float32 _battery_voltage
       (sys::poke _battery_voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _left_encoder
     (setq _left_encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_encoder
     (setq _right_encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _accelX
     (setq _accelX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _accelY
     (setq _accelY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _accelZ
     (setq _accelZ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _gyroX
     (setq _gyroX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _gyroY
     (setq _gyroY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _gyroZ
     (setq _gyroZ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _magX
     (setq _magX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _magY
     (setq _magY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _magZ
     (setq _magZ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _battery_voltage
     (setq _battery_voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nuturtlebot::SensorData :md5sum-) "e5bb303db7aaeaf900294f1b85ecc818")
(setf (get nuturtlebot::SensorData :datatype-) "nuturtlebot/SensorData")
(setf (get nuturtlebot::SensorData :definition-)
      "time stamp
int32 left_encoder
int32 right_encoder
int16 accelX
int16 accelY
int16 accelZ
int16 gyroX
int16 gyroY
int16 gyroZ
int16 magX
int16 magY
int16 magZ
float32 battery_voltage

")



(provide :nuturtlebot/SensorData "e5bb303db7aaeaf900294f1b85ecc818")


