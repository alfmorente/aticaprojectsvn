; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude msg_emergency_stop.msg.html

(cl:defclass <msg_emergency_stop> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_emergency_stop (<msg_emergency_stop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_emergency_stop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_emergency_stop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<msg_emergency_stop> is deprecated: use Modulo_Conduccion-msg:msg_emergency_stop instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_emergency_stop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:value-val is deprecated.  Use Modulo_Conduccion-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_emergency_stop>) ostream)
  "Serializes a message object of type '<msg_emergency_stop>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_emergency_stop>) istream)
  "Deserializes a message object of type '<msg_emergency_stop>"
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_emergency_stop>)))
  "Returns string type for a message object of type '<msg_emergency_stop>"
  "Modulo_Conduccion/msg_emergency_stop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_emergency_stop)))
  "Returns string type for a message object of type 'msg_emergency_stop"
  "Modulo_Conduccion/msg_emergency_stop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_emergency_stop>)))
  "Returns md5sum for a message object of type '<msg_emergency_stop>"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_emergency_stop)))
  "Returns md5sum for a message object of type 'msg_emergency_stop"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_emergency_stop>)))
  "Returns full string definition for message of type '<msg_emergency_stop>"
  (cl:format cl:nil "bool value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_emergency_stop)))
  "Returns full string definition for message of type 'msg_emergency_stop"
  (cl:format cl:nil "bool value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_emergency_stop>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_emergency_stop>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_emergency_stop
    (cl:cons ':value (value msg))
))
