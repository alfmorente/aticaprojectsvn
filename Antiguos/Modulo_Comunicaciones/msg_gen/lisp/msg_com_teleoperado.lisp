; Auto-generated. Do not edit!


(cl:in-package Modulo_Comunicaciones-msg)


;//! \htmlinclude msg_com_teleoperado.msg.html

(cl:defclass <msg_com_teleoperado> (roslisp-msg-protocol:ros-message)
  ((id_elemento
    :reader id_elemento
    :initarg :id_elemento
    :type cl:fixnum
    :initform 0)
   (valor
    :reader valor
    :initarg :valor
    :type cl:integer
    :initform 0)
   (depurado
    :reader depurado
    :initarg :depurado
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_com_teleoperado (<msg_com_teleoperado>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_com_teleoperado>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_com_teleoperado)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Comunicaciones-msg:<msg_com_teleoperado> is deprecated: use Modulo_Comunicaciones-msg:msg_com_teleoperado instead.")))

(cl:ensure-generic-function 'id_elemento-val :lambda-list '(m))
(cl:defmethod id_elemento-val ((m <msg_com_teleoperado>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:id_elemento-val is deprecated.  Use Modulo_Comunicaciones-msg:id_elemento instead.")
  (id_elemento m))

(cl:ensure-generic-function 'valor-val :lambda-list '(m))
(cl:defmethod valor-val ((m <msg_com_teleoperado>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:valor-val is deprecated.  Use Modulo_Comunicaciones-msg:valor instead.")
  (valor m))

(cl:ensure-generic-function 'depurado-val :lambda-list '(m))
(cl:defmethod depurado-val ((m <msg_com_teleoperado>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:depurado-val is deprecated.  Use Modulo_Comunicaciones-msg:depurado instead.")
  (depurado m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_com_teleoperado>) ostream)
  "Serializes a message object of type '<msg_com_teleoperado>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_elemento)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'valor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'depurado) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_com_teleoperado>) istream)
  "Deserializes a message object of type '<msg_com_teleoperado>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_elemento)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'valor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'depurado) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_com_teleoperado>)))
  "Returns string type for a message object of type '<msg_com_teleoperado>"
  "Modulo_Comunicaciones/msg_com_teleoperado")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_com_teleoperado)))
  "Returns string type for a message object of type 'msg_com_teleoperado"
  "Modulo_Comunicaciones/msg_com_teleoperado")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_com_teleoperado>)))
  "Returns md5sum for a message object of type '<msg_com_teleoperado>"
  "c54d49e66d8c41bb0d15084869606880")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_com_teleoperado)))
  "Returns md5sum for a message object of type 'msg_com_teleoperado"
  "c54d49e66d8c41bb0d15084869606880")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_com_teleoperado>)))
  "Returns full string definition for message of type '<msg_com_teleoperado>"
  (cl:format cl:nil "uint8 id_elemento~%int32 valor~%bool depurado~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_com_teleoperado)))
  "Returns full string definition for message of type 'msg_com_teleoperado"
  (cl:format cl:nil "uint8 id_elemento~%int32 valor~%bool depurado~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_com_teleoperado>))
  (cl:+ 0
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_com_teleoperado>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_com_teleoperado
    (cl:cons ':id_elemento (id_elemento msg))
    (cl:cons ':valor (valor msg))
    (cl:cons ':depurado (depurado msg))
))
