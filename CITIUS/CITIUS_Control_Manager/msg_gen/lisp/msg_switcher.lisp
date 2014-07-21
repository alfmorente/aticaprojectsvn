; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Manager-msg)


;//! \htmlinclude msg_switcher.msg.html

(cl:defclass <msg_switcher> (roslisp-msg-protocol:ros-message)
  ((switcher
    :reader switcher
    :initarg :switcher
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_switcher (<msg_switcher>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_switcher>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_switcher)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-msg:<msg_switcher> is deprecated: use CITIUS_Control_Manager-msg:msg_switcher instead.")))

(cl:ensure-generic-function 'switcher-val :lambda-list '(m))
(cl:defmethod switcher-val ((m <msg_switcher>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-msg:switcher-val is deprecated.  Use CITIUS_Control_Manager-msg:switcher instead.")
  (switcher m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_switcher>) ostream)
  "Serializes a message object of type '<msg_switcher>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'switcher)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_switcher>) istream)
  "Deserializes a message object of type '<msg_switcher>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'switcher)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_switcher>)))
  "Returns string type for a message object of type '<msg_switcher>"
  "CITIUS_Control_Manager/msg_switcher")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_switcher)))
  "Returns string type for a message object of type 'msg_switcher"
  "CITIUS_Control_Manager/msg_switcher")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_switcher>)))
  "Returns md5sum for a message object of type '<msg_switcher>"
  "6f9f7ff47283f474d33dbf2d2963b97e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_switcher)))
  "Returns md5sum for a message object of type 'msg_switcher"
  "6f9f7ff47283f474d33dbf2d2963b97e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_switcher>)))
  "Returns full string definition for message of type '<msg_switcher>"
  (cl:format cl:nil "uint8 switcher~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_switcher)))
  "Returns full string definition for message of type 'msg_switcher"
  (cl:format cl:nil "uint8 switcher~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_switcher>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_switcher>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_switcher
    (cl:cons ':switcher (switcher msg))
))
