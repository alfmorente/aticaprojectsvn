; Auto-generated. Do not edit!


(cl:in-package Modulo_Teleoperacion-msg)


;//! \htmlinclude msg_habilitacion_modulo.msg.html

(cl:defclass <msg_habilitacion_modulo> (roslisp-msg-protocol:ros-message)
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
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_habilitacion_modulo (<msg_habilitacion_modulo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_habilitacion_modulo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_habilitacion_modulo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Teleoperacion-msg:<msg_habilitacion_modulo> is deprecated: use Modulo_Teleoperacion-msg:msg_habilitacion_modulo instead.")))

(cl:ensure-generic-function 'id_modulo-val :lambda-list '(m))
(cl:defmethod id_modulo-val ((m <msg_habilitacion_modulo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Teleoperacion-msg:id_modulo-val is deprecated.  Use Modulo_Teleoperacion-msg:id_modulo instead.")
  (id_modulo m))

(cl:ensure-generic-function 'submodo-val :lambda-list '(m))
(cl:defmethod submodo-val ((m <msg_habilitacion_modulo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Teleoperacion-msg:submodo-val is deprecated.  Use Modulo_Teleoperacion-msg:submodo instead.")
  (submodo m))

(cl:ensure-generic-function 'activo-val :lambda-list '(m))
(cl:defmethod activo-val ((m <msg_habilitacion_modulo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Teleoperacion-msg:activo-val is deprecated.  Use Modulo_Teleoperacion-msg:activo instead.")
  (activo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_habilitacion_modulo>) ostream)
  "Serializes a message object of type '<msg_habilitacion_modulo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_modulo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'submodo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'activo) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_habilitacion_modulo>) istream)
  "Deserializes a message object of type '<msg_habilitacion_modulo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_modulo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'submodo)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'activo) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_habilitacion_modulo>)))
  "Returns string type for a message object of type '<msg_habilitacion_modulo>"
  "Modulo_Teleoperacion/msg_habilitacion_modulo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_habilitacion_modulo)))
  "Returns string type for a message object of type 'msg_habilitacion_modulo"
  "Modulo_Teleoperacion/msg_habilitacion_modulo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_habilitacion_modulo>)))
  "Returns md5sum for a message object of type '<msg_habilitacion_modulo>"
  "f3922bf2f9e23d1b3b255e4deb4c7240")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_habilitacion_modulo)))
  "Returns md5sum for a message object of type 'msg_habilitacion_modulo"
  "f3922bf2f9e23d1b3b255e4deb4c7240")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_habilitacion_modulo>)))
  "Returns full string definition for message of type '<msg_habilitacion_modulo>"
  (cl:format cl:nil "uint8 id_modulo~%uint8 submodo~%bool activo~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_habilitacion_modulo)))
  "Returns full string definition for message of type 'msg_habilitacion_modulo"
  (cl:format cl:nil "uint8 id_modulo~%uint8 submodo~%bool activo~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_habilitacion_modulo>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_habilitacion_modulo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_habilitacion_modulo
    (cl:cons ':id_modulo (id_modulo msg))
    (cl:cons ':submodo (submodo msg))
    (cl:cons ':activo (activo msg))
))
