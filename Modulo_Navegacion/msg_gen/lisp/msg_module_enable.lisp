; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_module_enable.msg.html

(cl:defclass <msg_module_enable> (roslisp-msg-protocol:ros-message)
  ((id_modulo
    :reader id_modulo
    :initarg :id_modulo
    :type cl:fixnum
    :initform 0)
   (submodo
    :reader submodo
    :initarg :submodo
    :type cl:fixnum
    :initform 0)
   (activo
    :reader activo
    :initarg :activo
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_module_enable (<msg_module_enable>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_module_enable>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_module_enable)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_module_enable> is deprecated: use Modulo_Navegacion-msg:msg_module_enable instead.")))

(cl:ensure-generic-function 'id_modulo-val :lambda-list '(m))
(cl:defmethod id_modulo-val ((m <msg_module_enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:id_modulo-val is deprecated.  Use Modulo_Navegacion-msg:id_modulo instead.")
  (id_modulo m))

(cl:ensure-generic-function 'submodo-val :lambda-list '(m))
(cl:defmethod submodo-val ((m <msg_module_enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:submodo-val is deprecated.  Use Modulo_Navegacion-msg:submodo instead.")
  (submodo m))

(cl:ensure-generic-function 'activo-val :lambda-list '(m))
(cl:defmethod activo-val ((m <msg_module_enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:activo-val is deprecated.  Use Modulo_Navegacion-msg:activo instead.")
  (activo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_module_enable>) ostream)
  "Serializes a message object of type '<msg_module_enable>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_modulo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'submodo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'activo)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_module_enable>) istream)
  "Deserializes a message object of type '<msg_module_enable>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_modulo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'submodo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'activo)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_module_enable>)))
  "Returns string type for a message object of type '<msg_module_enable>"
  "Modulo_Navegacion/msg_module_enable")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_module_enable)))
  "Returns string type for a message object of type 'msg_module_enable"
  "Modulo_Navegacion/msg_module_enable")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_module_enable>)))
  "Returns md5sum for a message object of type '<msg_module_enable>"
  "a982f6a119e51b5cdca5954e75b06d8d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_module_enable)))
  "Returns md5sum for a message object of type 'msg_module_enable"
  "a982f6a119e51b5cdca5954e75b06d8d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_module_enable>)))
  "Returns full string definition for message of type '<msg_module_enable>"
  (cl:format cl:nil "uint8 id_modulo~%uint8 submodo~%uint8 activo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_module_enable)))
  "Returns full string definition for message of type 'msg_module_enable"
  (cl:format cl:nil "uint8 id_modulo~%uint8 submodo~%uint8 activo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_module_enable>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_module_enable>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_module_enable
    (cl:cons ':id_modulo (id_modulo msg))
    (cl:cons ':submodo (submodo msg))
    (cl:cons ':activo (activo msg))
))
