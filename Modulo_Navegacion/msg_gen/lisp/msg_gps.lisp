; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_gps.msg.html

(cl:defclass <msg_gps> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
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
    :initform 0.0)
   (velocity_x
    :reader velocity_x
    :initarg :velocity_x
    :type cl:float
    :initform 0.0)
   (velocity_y
    :reader velocity_y
    :initarg :velocity_y
    :type cl:float
    :initform 0.0)
   (velocity_z
    :reader velocity_z
    :initarg :velocity_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass msg_gps (<msg_gps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_gps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_gps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_gps> is deprecated: use Modulo_Navegacion-msg:msg_gps instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:latitude-val is deprecated.  Use Modulo_Navegacion-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:longitude-val is deprecated.  Use Modulo_Navegacion-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:height-val is deprecated.  Use Modulo_Navegacion-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:pitch-val is deprecated.  Use Modulo_Navegacion-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:yaw-val is deprecated.  Use Modulo_Navegacion-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:roll-val is deprecated.  Use Modulo_Navegacion-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'velocity_x-val :lambda-list '(m))
(cl:defmethod velocity_x-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:velocity_x-val is deprecated.  Use Modulo_Navegacion-msg:velocity_x instead.")
  (velocity_x m))

(cl:ensure-generic-function 'velocity_y-val :lambda-list '(m))
(cl:defmethod velocity_y-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:velocity_y-val is deprecated.  Use Modulo_Navegacion-msg:velocity_y instead.")
  (velocity_y m))

(cl:ensure-generic-function 'velocity_z-val :lambda-list '(m))
(cl:defmethod velocity_z-val ((m <msg_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:velocity_z-val is deprecated.  Use Modulo_Navegacion-msg:velocity_z instead.")
  (velocity_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_gps>) ostream)
  "Serializes a message object of type '<msg_gps>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_z))))
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
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_gps>)))
  "Returns string type for a message object of type '<msg_gps>"
  "Modulo_Navegacion/msg_gps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_gps)))
  "Returns string type for a message object of type 'msg_gps"
  "Modulo_Navegacion/msg_gps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_gps>)))
  "Returns md5sum for a message object of type '<msg_gps>"
  "60954aed4adc07e00019828b664121f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_gps)))
  "Returns md5sum for a message object of type 'msg_gps"
  "60954aed4adc07e00019828b664121f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_gps>)))
  "Returns full string definition for message of type '<msg_gps>"
  (cl:format cl:nil "float32 latitude~%float32 longitude~%float32 height~%float32 pitch~%float32 yaw~%float32 roll~%float32 velocity_x~%float32 velocity_y~%float32 velocity_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_gps)))
  "Returns full string definition for message of type 'msg_gps"
  (cl:format cl:nil "float32 latitude~%float32 longitude~%float32 height~%float32 pitch~%float32 yaw~%float32 roll~%float32 velocity_x~%float32 velocity_y~%float32 velocity_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_gps>))
  (cl:+ 0
     4
     4
     4
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
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':height (height msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':velocity_x (velocity_x msg))
    (cl:cons ':velocity_y (velocity_y msg))
    (cl:cons ':velocity_z (velocity_z msg))
))
