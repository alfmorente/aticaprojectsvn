; Auto-generated. Do not edit!


(cl:in-package Modulo_Gest_Sistema-msg)


;//! \htmlinclude msg_prueba.msg.html

(cl:defclass <msg_prueba> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass msg_prueba (<msg_prueba>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_prueba>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_prueba)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Gest_Sistema-msg:<msg_prueba> is deprecated: use Modulo_Gest_Sistema-msg:msg_prueba instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_prueba>) ostream)
  "Serializes a message object of type '<msg_prueba>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_prueba>) istream)
  "Deserializes a message object of type '<msg_prueba>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_prueba>)))
  "Returns string type for a message object of type '<msg_prueba>"
  "Modulo_Gest_Sistema/msg_prueba")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_prueba)))
  "Returns string type for a message object of type 'msg_prueba"
  "Modulo_Gest_Sistema/msg_prueba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_prueba>)))
  "Returns md5sum for a message object of type '<msg_prueba>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_prueba)))
  "Returns md5sum for a message object of type 'msg_prueba"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_prueba>)))
  "Returns full string definition for message of type '<msg_prueba>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_prueba)))
  "Returns full string definition for message of type 'msg_prueba"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_prueba>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_prueba>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_prueba
))
