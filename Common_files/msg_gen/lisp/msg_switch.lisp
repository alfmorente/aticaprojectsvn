; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_switch.msg.html

(cl:defclass <msg_switch> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_switch (<msg_switch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_switch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_switch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_switch> is deprecated: use Common_files-msg:msg_switch instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_switch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:value-val is deprecated.  Use Common_files-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_switch>) ostream)
  "Serializes a message object of type '<msg_switch>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_switch>) istream)
  "Deserializes a message object of type '<msg_switch>"
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_switch>)))
  "Returns string type for a message object of type '<msg_switch>"
  "Common_files/msg_switch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_switch)))
  "Returns string type for a message object of type 'msg_switch"
  "Common_files/msg_switch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_switch>)))
  "Returns md5sum for a message object of type '<msg_switch>"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_switch)))
  "Returns md5sum for a message object of type 'msg_switch"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_switch>)))
  "Returns full string definition for message of type '<msg_switch>"
  (cl:format cl:nil "bool value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_switch)))
  "Returns full string definition for message of type 'msg_switch"
  (cl:format cl:nil "bool value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_switch>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_switch>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_switch
    (cl:cons ':value (value msg))
))
