; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude mastil.msg.html

(cl:defclass <mastil> (roslisp-msg-protocol:ros-message)
  ((opcionMastil
    :reader opcionMastil
    :initarg :opcionMastil
    :type cl:fixnum
    :initform 0))
)

(cl:defclass mastil (<mastil>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mastil>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mastil)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<mastil> is deprecated: use Modulo_Conduccion-msg:mastil instead.")))

(cl:ensure-generic-function 'opcionMastil-val :lambda-list '(m))
(cl:defmethod opcionMastil-val ((m <mastil>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcionMastil-val is deprecated.  Use Modulo_Conduccion-msg:opcionMastil instead.")
  (opcionMastil m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mastil>) ostream)
  "Serializes a message object of type '<mastil>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'opcionMastil)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mastil>) istream)
  "Deserializes a message object of type '<mastil>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'opcionMastil)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mastil>)))
  "Returns string type for a message object of type '<mastil>"
  "Modulo_Conduccion/mastil")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mastil)))
  "Returns string type for a message object of type 'mastil"
  "Modulo_Conduccion/mastil")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mastil>)))
  "Returns md5sum for a message object of type '<mastil>"
  "6511b7e1560cb5cbadf55dacd8c4fcf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mastil)))
  "Returns md5sum for a message object of type 'mastil"
  "6511b7e1560cb5cbadf55dacd8c4fcf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mastil>)))
  "Returns full string definition for message of type '<mastil>"
  (cl:format cl:nil "# DATOS DEL MÁSTIL~%~%uint8  opcionMastil	# Acciones del mastil:~%			#   0 -> Foco Derecho OFF~%			#   1 -> Foco Derecho ON~%			#   2 -> Foco Izquierdo OFF~%			#   3 -> Foco Izquierdo ON~%			#   4 -> Cabeceo Abajo ~%			#   5 -> Cabeceo Arriba ~%			#   6 -> Cabeceo Stop~%			#   7 -> Giro Derecha~%			#   8 -> Giro Izquierda~%			#   9 -> Giro Stop~%			#   10 -> Bajar~%			#   11 -> Subir~%			#   12 -> STOP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mastil)))
  "Returns full string definition for message of type 'mastil"
  (cl:format cl:nil "# DATOS DEL MÁSTIL~%~%uint8  opcionMastil	# Acciones del mastil:~%			#   0 -> Foco Derecho OFF~%			#   1 -> Foco Derecho ON~%			#   2 -> Foco Izquierdo OFF~%			#   3 -> Foco Izquierdo ON~%			#   4 -> Cabeceo Abajo ~%			#   5 -> Cabeceo Arriba ~%			#   6 -> Cabeceo Stop~%			#   7 -> Giro Derecha~%			#   8 -> Giro Izquierda~%			#   9 -> Giro Stop~%			#   10 -> Bajar~%			#   11 -> Subir~%			#   12 -> STOP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mastil>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mastil>))
  "Converts a ROS message object to a list"
  (cl:list 'mastil
    (cl:cons ':opcionMastil (opcionMastil msg))
))
