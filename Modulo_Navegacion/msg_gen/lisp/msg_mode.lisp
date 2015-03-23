; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_mode.msg.html

(cl:defclass <msg_mode> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_mode (<msg_mode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_mode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_mode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_mode> is deprecated: use Modulo_Navegacion-msg:msg_mode instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <msg_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:mode-val is deprecated.  Use Modulo_Navegacion-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <msg_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:status-val is deprecated.  Use Modulo_Navegacion-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_mode>) ostream)
  "Serializes a message object of type '<msg_mode>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_mode>) istream)
  "Deserializes a message object of type '<msg_mode>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_mode>)))
  "Returns string type for a message object of type '<msg_mode>"
  "Modulo_Navegacion/msg_mode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_mode)))
  "Returns string type for a message object of type 'msg_mode"
  "Modulo_Navegacion/msg_mode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_mode>)))
  "Returns md5sum for a message object of type '<msg_mode>"
  "31202a93cf12686e4f3edec877644512")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_mode)))
  "Returns md5sum for a message object of type 'msg_mode"
  "31202a93cf12686e4f3edec877644512")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_mode>)))
  "Returns full string definition for message of type '<msg_mode>"
  (cl:format cl:nil "uint8 mode~%uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_mode)))
  "Returns full string definition for message of type 'msg_mode"
  (cl:format cl:nil "uint8 mode~%uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_mode>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_mode>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_mode
    (cl:cons ':mode (mode msg))
    (cl:cons ':status (status msg))
))
