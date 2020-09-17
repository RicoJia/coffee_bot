;; Auto-generated. Do not edit!


(when (boundp 'nuturtlebot::WheelCommands)
  (if (not (find-package "NUTURTLEBOT"))
    (make-package "NUTURTLEBOT"))
  (shadow 'WheelCommands (find-package "NUTURTLEBOT")))
(unless (find-package "NUTURTLEBOT::WHEELCOMMANDS")
  (make-package "NUTURTLEBOT::WHEELCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude WheelCommands.msg.html


(defclass nuturtlebot::WheelCommands
  :super ros::object
  :slots (_left_velocity _right_velocity ))

(defmethod nuturtlebot::WheelCommands
  (:init
   (&key
    ((:left_velocity __left_velocity) 0)
    ((:right_velocity __right_velocity) 0)
    )
   (send-super :init)
   (setq _left_velocity (round __left_velocity))
   (setq _right_velocity (round __right_velocity))
   self)
  (:left_velocity
   (&optional __left_velocity)
   (if __left_velocity (setq _left_velocity __left_velocity)) _left_velocity)
  (:right_velocity
   (&optional __right_velocity)
   (if __right_velocity (setq _right_velocity __right_velocity)) _right_velocity)
  (:serialization-length
   ()
   (+
    ;; int32 _left_velocity
    4
    ;; int32 _right_velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _left_velocity
       (write-long _left_velocity s)
     ;; int32 _right_velocity
       (write-long _right_velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _left_velocity
     (setq _left_velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_velocity
     (setq _right_velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nuturtlebot::WheelCommands :md5sum-) "55f26e03298a44797b568254274af487")
(setf (get nuturtlebot::WheelCommands :datatype-) "nuturtlebot/WheelCommands")
(setf (get nuturtlebot::WheelCommands :definition-)
      "int32 left_velocity
int32 right_velocity

")



(provide :nuturtlebot/WheelCommands "55f26e03298a44797b568254274af487")


