;; Auto-generated. Do not edit!


(when (boundp 'nuturtle_robot::Start)
  (if (not (find-package "NUTURTLE_ROBOT"))
    (make-package "NUTURTLE_ROBOT"))
  (shadow 'Start (find-package "NUTURTLE_ROBOT")))
(unless (find-package "NUTURTLE_ROBOT::START")
  (make-package "NUTURTLE_ROBOT::START"))
(unless (find-package "NUTURTLE_ROBOT::STARTREQUEST")
  (make-package "NUTURTLE_ROBOT::STARTREQUEST"))
(unless (find-package "NUTURTLE_ROBOT::STARTRESPONSE")
  (make-package "NUTURTLE_ROBOT::STARTRESPONSE"))

(in-package "ROS")





(defclass nuturtle_robot::StartRequest
  :super ros::object
  :slots (_ccw_or_forward ))

(defmethod nuturtle_robot::StartRequest
  (:init
   (&key
    ((:ccw_or_forward __ccw_or_forward) nil)
    )
   (send-super :init)
   (setq _ccw_or_forward __ccw_or_forward)
   self)
  (:ccw_or_forward
   (&optional __ccw_or_forward)
   (if __ccw_or_forward (setq _ccw_or_forward __ccw_or_forward)) _ccw_or_forward)
  (:serialization-length
   ()
   (+
    ;; bool _ccw_or_forward
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ccw_or_forward
       (if _ccw_or_forward (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ccw_or_forward
     (setq _ccw_or_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass nuturtle_robot::StartResponse
  :super ros::object
  :slots ())

(defmethod nuturtle_robot::StartResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass nuturtle_robot::Start
  :super ros::object
  :slots ())

(setf (get nuturtle_robot::Start :md5sum-) "e3cef58e896b7b1fe8be0821b7a77707")
(setf (get nuturtle_robot::Start :datatype-) "nuturtle_robot/Start")
(setf (get nuturtle_robot::Start :request) nuturtle_robot::StartRequest)
(setf (get nuturtle_robot::Start :response) nuturtle_robot::StartResponse)

(defmethod nuturtle_robot::StartRequest
  (:response () (instance nuturtle_robot::StartResponse :init)))

(setf (get nuturtle_robot::StartRequest :md5sum-) "e3cef58e896b7b1fe8be0821b7a77707")
(setf (get nuturtle_robot::StartRequest :datatype-) "nuturtle_robot/StartRequest")
(setf (get nuturtle_robot::StartRequest :definition-)
      "bool ccw_or_forward
---
")

(setf (get nuturtle_robot::StartResponse :md5sum-) "e3cef58e896b7b1fe8be0821b7a77707")
(setf (get nuturtle_robot::StartResponse :datatype-) "nuturtle_robot/StartResponse")
(setf (get nuturtle_robot::StartResponse :definition-)
      "bool ccw_or_forward
---
")



(provide :nuturtle_robot/Start "e3cef58e896b7b1fe8be0821b7a77707")


