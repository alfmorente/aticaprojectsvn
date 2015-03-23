; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_error.msg.html

(cl:defclass <msg_error> (roslisp-msg-protocol:ros-message)
  ((ID_subsystem
    :reader ID_subsystem
    :initarg :ID_subsystem
    :type cl:fixnum
    :initform 0)
   (ID_error
    :reader ID_error
    :initarg :ID_error
    :type cl:fixnum
    :initform 0)
   (type_error
    :reader type_error
    :initarg :type_error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_error (<msg_error>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_error>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_error)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_error> is deprecated: use Modulo_Navegacion-msg:msg_error instead.")))

(cl:ensure-generic-function 'ID_subsystem-val :lambda-list '(m))
(cl:defmethod ID_subsystem-val ((m <msg_error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:ID_subsystem-val is deprecated.  Use Modulo_Navegacion-msg:ID_subsystem instead.")
  (ID_subsystem m))

(cl:ensure-generic-function 'ID_error-val :lambda-list '(m))
(cl:defmethod ID_error-val ((m <msg_error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:ID_error-val is deprecated.  Use Modulo_Navegacion-msg:ID_error instead.")
  (ID_error m))

(cl:ensure-generic-function 'type_error-val :lambda-list '(m))
(cl:defmethod type_error-val ((m <msg_error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:type_error-val is deprecated.  Use Modulo_Navegacion-msg:type_error instead.")
  (type_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_error>) ostream)
  "Serializes a message object of type '<msg_error>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID_subsystem)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ID_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type_error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_error>) istream)
  "Deserializes a message object of type '<msg_error>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID_subsystem)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ID_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type_error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_error>)))
  "Returns string type for a message object of type '<msg_error>"
  "Modulo_Navegacion/msg_error")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_error)))
  "Returns string type for a message object of type 'msg_error"
  "Modulo_Navegacion/msg_error")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_error>)))
  "Returns md5sum for a message object of type '<msg_error>"
  "e95455438d88b26afce71bfbd21af081")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_error)))
  "Returns md5sum for a message object of type 'msg_error"
  "e95455438d88b26afce71bfbd21af081")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_error>)))
  "Returns full string definition for message of type '<msg_error>"
  (cl:format cl:nil "uint8 ID_subsystem~%uint16 ID_error~%uint16 type_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_error)))
  "Returns full string definition for message of type 'msg_error"
  (cl:format cl:nil "uint8 ID_subsystem~%uint16 ID_error~%uint16 type_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_error>))
  (cl:+ 0
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_error>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_error
    (cl:cons ':ID_subsystem (ID_subsystem msg))
    (cl:cons ':ID_error (ID_error msg))
    (cl:cons ':type_error (type_error msg))
))
