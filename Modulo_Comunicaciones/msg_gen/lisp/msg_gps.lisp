; Auto-generated. Do not edit!


(cl:in-package Modulo_Comunicaciones-msg)


;//! \htmlinclude msg_gps.msg.html

(cl:defclass <msg_gps> (roslisp-msg-protocol:ros-message)
  ((latitud
    :reader latitud
    :initarg :latitud
    :type cl:float
    :initform 0.0)
   (longitud
    :reader longitud
    :initarg :longitud
    :type cl:float
    :initform 0.0)
   (altitud
    :reader altitud
    :initarg :altitud
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0))
)

(cl:defclass msg_gps (<msg_gps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_gps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_gps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Comunicaciones-msg:<msg_gps> is deprecated: use Modulo_Comunicaciones-msg:msg_gps instead.")))

(cl:ensure-generic-function 'latitud-val :lambda-list '(m))
(cl:defmethod latitud-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:latitud-val is deprecated.  Use Modulo_Comunicaciones-msg:latitud instead.")
  (latitud m))

(cl:ensure-generic-function 'longitud-val :lambda-list '(m))
(cl:defmethod longitud-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:longitud-val is deprecated.  Use Modulo_Comunicaciones-msg:longitud instead.")
  (longitud m))

(cl:ensure-generic-function 'altitud-val :lambda-list '(m))
(cl:defmethod altitud-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:altitud-val is deprecated.  Use Modulo_Comunicaciones-msg:altitud instead.")
  (altitud m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:pitch-val is deprecated.  Use Modulo_Comunicaciones-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:yaw-val is deprecated.  Use Modulo_Comunicaciones-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Comunicaciones-msg:roll-val is deprecated.  Use Modulo_Comunicaciones-msg:roll instead.")
  (roll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_gps>) ostream)
  "Serializes a message object of type '<msg_gps>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitud))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitud))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitud))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_gps>) istream)
  "Deserializes a message object of type '<msg_gps>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitud) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitud) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitud) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_gps>)))
  "Returns string type for a message object of type '<msg_gps>"
  "Modulo_Comunicaciones/msg_gps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_gps)))
  "Returns string type for a message object of type 'msg_gps"
  "Modulo_Comunicaciones/msg_gps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_gps>)))
  "Returns md5sum for a message object of type '<msg_gps>"
  "d580e52d6a982a2a25a6c4682d734553")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_gps)))
  "Returns md5sum for a message object of type 'msg_gps"
  "d580e52d6a982a2a25a6c4682d734553")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_gps>)))
  "Returns full string definition for message of type '<msg_gps>"
  (cl:format cl:nil "float32 latitud~%float32 longitud~%float32 altitud~%float32 pitch~%float32 yaw~%float32 roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_gps)))
  "Returns full string definition for message of type 'msg_gps"
  (cl:format cl:nil "float32 latitud~%float32 longitud~%float32 altitud~%float32 pitch~%float32 yaw~%float32 roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_gps>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_gps>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_gps
    (cl:cons ':latitud (latitud msg))
    (cl:cons ':longitud (longitud msg))
    (cl:cons ':altitud (altitud msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':roll (roll msg))
))
