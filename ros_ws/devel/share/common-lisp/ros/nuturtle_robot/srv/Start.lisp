; Auto-generated. Do not edit!


(cl:in-package nuturtle_robot-srv)


;//! \htmlinclude Start-request.msg.html

(cl:defclass <Start-request> (roslisp-msg-protocol:ros-message)
  ((ccw_or_forward
    :reader ccw_or_forward
    :initarg :ccw_or_forward
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Start-request (<Start-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Start-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Start-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nuturtle_robot-srv:<Start-request> is deprecated: use nuturtle_robot-srv:Start-request instead.")))

(cl:ensure-generic-function 'ccw_or_forward-val :lambda-list '(m))
(cl:defmethod ccw_or_forward-val ((m <Start-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nuturtle_robot-srv:ccw_or_forward-val is deprecated.  Use nuturtle_robot-srv:ccw_or_forward instead.")
  (ccw_or_forward m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Start-request>) ostream)
  "Serializes a message object of type '<Start-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ccw_or_forward) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Start-request>) istream)
  "Deserializes a message object of type '<Start-request>"
    (cl:setf (cl:slot-value msg 'ccw_or_forward) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Start-request>)))
  "Returns string type for a service object of type '<Start-request>"
  "nuturtle_robot/StartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start-request)))
  "Returns string type for a service object of type 'Start-request"
  "nuturtle_robot/StartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Start-request>)))
  "Returns md5sum for a message object of type '<Start-request>"
  "e3cef58e896b7b1fe8be0821b7a77707")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Start-request)))
  "Returns md5sum for a message object of type 'Start-request"
  "e3cef58e896b7b1fe8be0821b7a77707")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Start-request>)))
  "Returns full string definition for message of type '<Start-request>"
  (cl:format cl:nil "bool ccw_or_forward~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Start-request)))
  "Returns full string definition for message of type 'Start-request"
  (cl:format cl:nil "bool ccw_or_forward~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Start-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Start-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Start-request
    (cl:cons ':ccw_or_forward (ccw_or_forward msg))
))
;//! \htmlinclude Start-response.msg.html

(cl:defclass <Start-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Start-response (<Start-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Start-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Start-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nuturtle_robot-srv:<Start-response> is deprecated: use nuturtle_robot-srv:Start-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Start-response>) ostream)
  "Serializes a message object of type '<Start-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Start-response>) istream)
  "Deserializes a message object of type '<Start-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Start-response>)))
  "Returns string type for a service object of type '<Start-response>"
  "nuturtle_robot/StartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start-response)))
  "Returns string type for a service object of type 'Start-response"
  "nuturtle_robot/StartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Start-response>)))
  "Returns md5sum for a message object of type '<Start-response>"
  "e3cef58e896b7b1fe8be0821b7a77707")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Start-response)))
  "Returns md5sum for a message object of type 'Start-response"
  "e3cef58e896b7b1fe8be0821b7a77707")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Start-response>)))
  "Returns full string definition for message of type '<Start-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Start-response)))
  "Returns full string definition for message of type 'Start-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Start-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Start-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Start-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Start)))
  'Start-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Start)))
  'Start-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start)))
  "Returns string type for a service object of type '<Start>"
  "nuturtle_robot/Start")