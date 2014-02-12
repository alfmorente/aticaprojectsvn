; Auto-generated. Do not edit!


(cl:in-package Modulo_HumanLocalization-msg)


;//! \htmlinclude msg_errores.msg.html

(cl:defclass <msg_errores> (roslisp-msg-protocol:ros-message)
  ((id_subsistema
    :reader id_subsistema
    :initarg :id_subsistema
    :type cl:fixnum
    :initform 0)
   (id_error
    :reader id_error
    :initarg :id_error
    :type cl:fixnum
    :initform 0)
   (tipo_error
    :reader tipo_error
    :initarg :tipo_error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_errores (<msg_errores>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_errores>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_errores)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_HumanLocalization-msg:<msg_errores> is deprecated: use Modulo_HumanLocalization-msg:msg_errores instead.")))

(cl:ensure-generic-function 'id_subsistema-val :lambda-list '(m))
(cl:defmethod id_subsistema-val ((m <msg_errores>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_HumanLocalization-msg:id_subsistema-val is deprecated.  Use Modulo_HumanLocalization-msg:id_subsistema instead.")
  (id_subsistema m))

(cl:ensure-generic-function 'id_error-val :lambda-list '(m))
(cl:defmethod id_error-val ((m <msg_errores>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_HumanLocalization-msg:id_error-val is deprecated.  Use Modulo_HumanLocalization-msg:id_error instead.")
  (id_error m))

(cl:ensure-generic-function 'tipo_error-val :lambda-list '(m))
(cl:defmethod tipo_error-val ((m <msg_errores>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_HumanLocalization-msg:tipo_error-val is deprecated.  Use Modulo_HumanLocalization-msg:tipo_error instead.")
  (tipo_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_errores>) ostream)
  "Serializes a message object of type '<msg_errores>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_subsistema)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tipo_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tipo_error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_errores>) istream)
  "Deserializes a message object of type '<msg_errores>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_subsistema)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tipo_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tipo_error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_errores>)))
  "Returns string type for a message object of type '<msg_errores>"
  "Modulo_HumanLocalization/msg_errores")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_errores)))
  "Returns string type for a message object of type 'msg_errores"
  "Modulo_HumanLocalization/msg_errores")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_errores>)))
  "Returns md5sum for a message object of type '<msg_errores>"
  "452f360bd2b2519d5c8cd86e73b758d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_errores)))
  "Returns md5sum for a message object of type 'msg_errores"
  "452f360bd2b2519d5c8cd86e73b758d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_errores>)))
  "Returns full string definition for message of type '<msg_errores>"
  (cl:format cl:nil "uint8 id_subsistema~%uint16 id_error~%uint16 tipo_error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_errores)))
  "Returns full string definition for message of type 'msg_errores"
  (cl:format cl:nil "uint8 id_subsistema~%uint16 id_error~%uint16 tipo_error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_errores>))
  (cl:+ 0
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_errores>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_errores
    (cl:cons ':id_subsistema (id_subsistema msg))
    (cl:cons ':id_error (id_error msg))
    (cl:cons ':tipo_error (tipo_error msg))
))
