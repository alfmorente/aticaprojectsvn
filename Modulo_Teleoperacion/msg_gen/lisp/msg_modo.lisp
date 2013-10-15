; Auto-generated. Do not edit!


(cl:in-package Modulo_Teleoperacion-msg)


;//! \htmlinclude msg_modo.msg.html

(cl:defclass <msg_modo> (roslisp-msg-protocol:ros-message)
  ((modo
    :reader modo
    :initarg :modo
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_modo (<msg_modo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_modo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_modo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Teleoperacion-msg:<msg_modo> is deprecated: use Modulo_Teleoperacion-msg:msg_modo instead.")))

(cl:ensure-generic-function 'modo-val :lambda-list '(m))
(cl:defmethod modo-val ((m <msg_modo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Teleoperacion-msg:modo-val is deprecated.  Use Modulo_Teleoperacion-msg:modo instead.")
  (modo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_modo>) ostream)
  "Serializes a message object of type '<msg_modo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'modo)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_modo>) istream)
  "Deserializes a message object of type '<msg_modo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'modo)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_modo>)))
  "Returns string type for a message object of type '<msg_modo>"
  "Modulo_Teleoperacion/msg_modo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_modo)))
  "Returns string type for a message object of type 'msg_modo"
  "Modulo_Teleoperacion/msg_modo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_modo>)))
  "Returns md5sum for a message object of type '<msg_modo>"
  "bc4ecbd89e2dc3ed7375583640445dbd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_modo)))
  "Returns md5sum for a message object of type 'msg_modo"
  "bc4ecbd89e2dc3ed7375583640445dbd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_modo>)))
  "Returns full string definition for message of type '<msg_modo>"
  (cl:format cl:nil "uint8 modo~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_modo)))
  "Returns full string definition for message of type 'msg_modo"
  (cl:format cl:nil "uint8 modo~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_modo>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_modo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_modo
    (cl:cons ':modo (modo msg))
))
