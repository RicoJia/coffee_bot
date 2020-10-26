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
  :slots (_stamp _left_encoder _right_encoder ))

(defmethod nuturtlebot::SensorData
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:left_encoder __left_encoder) 0.0)
    ((:right_encoder __right_encoder) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _left_encoder (float __left_encoder))
   (setq _right_encoder (float __right_encoder))
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
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32 _left_encoder
    4
    ;; float32 _right_encoder
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _left_encoder
       (sys::poke _left_encoder (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_encoder
       (sys::poke _right_encoder (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _left_encoder
     (setq _left_encoder (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_encoder
     (setq _right_encoder (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nuturtlebot::SensorData :md5sum-) "5e11846fa8bcf550efab217e45ea9e19")
(setf (get nuturtlebot::SensorData :datatype-) "nuturtlebot/SensorData")
(setf (get nuturtlebot::SensorData :definition-)
      "time stamp
float32 left_encoder
float32 right_encoder

")



(provide :nuturtlebot/SensorData "5e11846fa8bcf550efab217e45ea9e19")


