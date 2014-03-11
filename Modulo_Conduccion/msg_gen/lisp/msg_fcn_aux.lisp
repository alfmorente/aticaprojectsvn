; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude msg_fcn_aux.msg.html

(cl:defclass <msg_fcn_aux> (roslisp-msg-protocol:ros-message)
  ((type_msg
    :reader type_msg
    :initarg :type_msg
    :type cl:boolean
    :initform cl:nil)
   (function
    :reader function
    :initarg :function
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_fcn_aux (<msg_fcn_aux>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_fcn_aux>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_fcn_aux)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<msg_fcn_aux> is deprecated: use Modulo_Conduccion-msg:msg_fcn_aux instead.")))

(cl:ensure-generic-function 'type_msg-val :lambda-list '(m))
(cl:defmethod type_msg-val ((m <msg_fcn_aux>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:type_msg-val is deprecated.  Use Modulo_Conduccion-msg:type_msg instead.")
  (type_msg m))

(cl:ensure-generic-function 'function-val :lambda-list '(m))
(cl:defmethod function-val ((m <msg_fcn_aux>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:function-val is deprecated.  Use Modulo_Conduccion-msg:function instead.")
  (function m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_fcn_aux>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:value-val is deprecated.  Use Modulo_Conduccion-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_fcn_aux>) ostream)
  "Serializes a message object of type '<msg_fcn_aux>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'type_msg) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'function)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_fcn_aux>) istream)
  "Deserializes a message object of type '<msg_fcn_aux>"
    (cl:setf (cl:slot-value msg 'type_msg) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'function)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_fcn_aux>)))
  "Returns string type for a message object of type '<msg_fcn_aux>"
  "Modulo_Conduccion/msg_fcn_aux")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_fcn_aux)))
  "Returns string type for a message object of type 'msg_fcn_aux"
  "Modulo_Conduccion/msg_fcn_aux")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_fcn_aux>)))
  "Returns md5sum for a message object of type '<msg_fcn_aux>"
  "5ac596cebd65fb2d60c3c460c58ab4a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_fcn_aux)))
  "Returns md5sum for a message object of type 'msg_fcn_aux"
  "5ac596cebd65fb2d60c3c460c58ab4a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_fcn_aux>)))
  "Returns full string definition for message of type '<msg_fcn_aux>"
  (cl:format cl:nil "bool type_msg~%uint8 function~%bool value      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_fcn_aux)))
  "Returns full string definition for message of type 'msg_fcn_aux"
  (cl:format cl:nil "bool type_msg~%uint8 function~%bool value      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_fcn_aux>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_fcn_aux>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_fcn_aux
    (cl:cons ':type_msg (type_msg msg))
    (cl:cons ':function (function msg))
    (cl:cons ':value (value msg))
))
