; Auto-generated. Do not edit!


(cl:in-package nuturtlebot-msg)


;//! \htmlinclude WheelCommands.msg.html

(cl:defclass <WheelCommands> (roslisp-msg-protocol:ros-message)
  ((left_velocity
    :reader left_velocity
    :initarg :left_velocity
    :type cl:integer
    :initform 0)
   (right_velocity
    :reader right_velocity
    :initarg :right_velocity
    :type cl:integer
    :initform 0))
)

(cl:defclass WheelCommands (<WheelCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nuturtlebot-msg:<WheelCommands> is deprecated: use nuturtlebot-msg:WheelCommands instead.")))

(cl:ensure-generic-function 'left_velocity-val :lambda-list '(m))
(cl:defmethod left_velocity-val ((m <WheelCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nuturtlebot-msg:left_velocity-val is deprecated.  Use nuturtlebot-msg:left_velocity instead.")
  (left_velocity m))

(cl:ensure-generic-function 'right_velocity-val :lambda-list '(m))
(cl:defmethod right_velocity-val ((m <WheelCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nuturtlebot-msg:right_velocity-val is deprecated.  Use nuturtlebot-msg:right_velocity instead.")
  (right_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelCommands>) ostream)
  "Serializes a message object of type '<WheelCommands>"
  (cl:let* ((signed (cl:slot-value msg 'left_velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelCommands>) istream)
  "Deserializes a message object of type '<WheelCommands>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelCommands>)))
  "Returns string type for a message object of type '<WheelCommands>"
  "nuturtlebot/WheelCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelCommands)))
  "Returns string type for a message object of type 'WheelCommands"
  "nuturtlebot/WheelCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelCommands>)))
  "Returns md5sum for a message object of type '<WheelCommands>"
  "55f26e03298a44797b568254274af487")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelCommands)))
  "Returns md5sum for a message object of type 'WheelCommands"
  "55f26e03298a44797b568254274af487")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelCommands>)))
  "Returns full string definition for message of type '<WheelCommands>"
  (cl:format cl:nil "int32 left_velocity~%int32 right_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelCommands)))
  "Returns full string definition for message of type 'WheelCommands"
  (cl:format cl:nil "int32 left_velocity~%int32 right_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelCommands>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelCommands
    (cl:cons ':left_velocity (left_velocity msg))
    (cl:cons ':right_velocity (right_velocity msg))
))
