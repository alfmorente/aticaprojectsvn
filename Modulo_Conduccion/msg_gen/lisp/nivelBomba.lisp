; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude nivelBomba.msg.html

(cl:defclass <nivelBomba> (roslisp-msg-protocol:ros-message)
  ((nivel
    :reader nivel
    :initarg :nivel
    :type cl:fixnum
    :initform 0))
)

(cl:defclass nivelBomba (<nivelBomba>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nivelBomba>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nivelBomba)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<nivelBomba> is deprecated: use Modulo_Conduccion-msg:nivelBomba instead.")))

(cl:ensure-generic-function 'nivel-val :lambda-list '(m))
(cl:defmethod nivel-val ((m <nivelBomba>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:nivel-val is deprecated.  Use Modulo_Conduccion-msg:nivel instead.")
  (nivel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nivelBomba>) ostream)
  "Serializes a message object of type '<nivelBomba>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nivel)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nivelBomba>) istream)
  "Deserializes a message object of type '<nivelBomba>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nivel)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nivelBomba>)))
  "Returns string type for a message object of type '<nivelBomba>"
  "Modulo_Conduccion/nivelBomba")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nivelBomba)))
  "Returns string type for a message object of type 'nivelBomba"
  "Modulo_Conduccion/nivelBomba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nivelBomba>)))
  "Returns md5sum for a message object of type '<nivelBomba>"
  "c1fa59fa4955af9ca1e740cc83229485")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nivelBomba)))
  "Returns md5sum for a message object of type 'nivelBomba"
  "c1fa59fa4955af9ca1e740cc83229485")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nivelBomba>)))
  "Returns full string definition for message of type '<nivelBomba>"
  (cl:format cl:nil "uint8 nivel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nivelBomba)))
  "Returns full string definition for message of type 'nivelBomba"
  (cl:format cl:nil "uint8 nivel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nivelBomba>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nivelBomba>))
  "Converts a ROS message object to a list"
  (cl:list 'nivelBomba
    (cl:cons ':nivel (nivel msg))
))
