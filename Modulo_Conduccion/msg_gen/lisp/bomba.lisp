; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude bomba.msg.html

(cl:defclass <bomba> (roslisp-msg-protocol:ros-message)
  ((comando
    :reader comando
    :initarg :comando
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass bomba (<bomba>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bomba>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bomba)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<bomba> is deprecated: use Modulo_Conduccion-msg:bomba instead.")))

(cl:ensure-generic-function 'comando-val :lambda-list '(m))
(cl:defmethod comando-val ((m <bomba>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:comando-val is deprecated.  Use Modulo_Conduccion-msg:comando instead.")
  (comando m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <bomba>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:value-val is deprecated.  Use Modulo_Conduccion-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bomba>) ostream)
  "Serializes a message object of type '<bomba>"
  (cl:let* ((signed (cl:slot-value msg 'comando)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bomba>) istream)
  "Deserializes a message object of type '<bomba>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'comando) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bomba>)))
  "Returns string type for a message object of type '<bomba>"
  "Modulo_Conduccion/bomba")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bomba)))
  "Returns string type for a message object of type 'bomba"
  "Modulo_Conduccion/bomba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bomba>)))
  "Returns md5sum for a message object of type '<bomba>"
  "bdb7d191e409324f9e33d915b1c8707b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bomba)))
  "Returns md5sum for a message object of type 'bomba"
  "bdb7d191e409324f9e33d915b1c8707b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bomba>)))
  "Returns full string definition for message of type '<bomba>"
  (cl:format cl:nil "# Desacelerar presion (comando=0, value=0)~%# Acelerar presion (comando=0, value=1)~%# Valvula  cisterna OFF (comando=1, value=0)~%# Valvula  cisterna ON  (comando=1, value=1)~%# Valvula  autollenado OFF (comando=2, value=0)~%# Valvula  autollenado ON (comando=2, value=1)~%# Valvula  monitor OFF (comando=3, value=0)~%# Valvula  monitor ON (comando=3, value=1)~%# Movimiento PAN izquierda (comando=4, value=0)~%# Movimiento PAN derecha   (comando=4, value=1)~%# Movimiento PAN Stop (comando=4, value=2)~%# Movimiento TILT arriba (comando=5, value=1)~%# Movimiento tilt abajo (comando=5, value=0)~%# Movimiento Tilt Stop (comando=5,value=0)~%# Tipo de chorro CHORRO (comando 6,value=0)~%# Tipo de chorro NIEBLA (comando 6,value=1)~%~%int8 comando~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bomba)))
  "Returns full string definition for message of type 'bomba"
  (cl:format cl:nil "# Desacelerar presion (comando=0, value=0)~%# Acelerar presion (comando=0, value=1)~%# Valvula  cisterna OFF (comando=1, value=0)~%# Valvula  cisterna ON  (comando=1, value=1)~%# Valvula  autollenado OFF (comando=2, value=0)~%# Valvula  autollenado ON (comando=2, value=1)~%# Valvula  monitor OFF (comando=3, value=0)~%# Valvula  monitor ON (comando=3, value=1)~%# Movimiento PAN izquierda (comando=4, value=0)~%# Movimiento PAN derecha   (comando=4, value=1)~%# Movimiento PAN Stop (comando=4, value=2)~%# Movimiento TILT arriba (comando=5, value=1)~%# Movimiento tilt abajo (comando=5, value=0)~%# Movimiento Tilt Stop (comando=5,value=0)~%# Tipo de chorro CHORRO (comando 6,value=0)~%# Tipo de chorro NIEBLA (comando 6,value=1)~%~%int8 comando~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bomba>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bomba>))
  "Converts a ROS message object to a list"
  (cl:list 'bomba
    (cl:cons ':comando (comando msg))
    (cl:cons ':value (value msg))
))
