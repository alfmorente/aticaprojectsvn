; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude msg_backup.msg.html

(cl:defclass <msg_backup> (roslisp-msg-protocol:ros-message)
  ((throttle
    :reader throttle
    :initarg :throttle
    :type cl:fixnum
    :initform 0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:fixnum
    :initform 0)
   (steer
    :reader steer
    :initarg :steer
    :type cl:fixnum
    :initform 0)
   (handbrake
    :reader handbrake
    :initarg :handbrake
    :type cl:boolean
    :initform cl:nil)
   (gear
    :reader gear
    :initarg :gear
    :type cl:fixnum
    :initform 0)
   (engine
    :reader engine
    :initarg :engine
    :type cl:boolean
    :initform cl:nil)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_backup (<msg_backup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_backup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_backup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<msg_backup> is deprecated: use Modulo_Conduccion-msg:msg_backup instead.")))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:throttle-val is deprecated.  Use Modulo_Conduccion-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:brake-val is deprecated.  Use Modulo_Conduccion-msg:brake instead.")
  (brake m))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:steer-val is deprecated.  Use Modulo_Conduccion-msg:steer instead.")
  (steer m))

(cl:ensure-generic-function 'handbrake-val :lambda-list '(m))
(cl:defmethod handbrake-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:handbrake-val is deprecated.  Use Modulo_Conduccion-msg:handbrake instead.")
  (handbrake m))

(cl:ensure-generic-function 'gear-val :lambda-list '(m))
(cl:defmethod gear-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:gear-val is deprecated.  Use Modulo_Conduccion-msg:gear instead.")
  (gear m))

(cl:ensure-generic-function 'engine-val :lambda-list '(m))
(cl:defmethod engine-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:engine-val is deprecated.  Use Modulo_Conduccion-msg:engine instead.")
  (engine m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <msg_backup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:speed-val is deprecated.  Use Modulo_Conduccion-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_backup>) ostream)
  "Serializes a message object of type '<msg_backup>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'throttle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'steer)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'handbrake) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'engine) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_backup>) istream)
  "Deserializes a message object of type '<msg_backup>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'throttle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steer) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'handbrake) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'engine) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_backup>)))
  "Returns string type for a message object of type '<msg_backup>"
  "Modulo_Conduccion/msg_backup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_backup)))
  "Returns string type for a message object of type 'msg_backup"
  "Modulo_Conduccion/msg_backup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_backup>)))
  "Returns md5sum for a message object of type '<msg_backup>"
  "ff8688787aa97766b5311a5e39e25a80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_backup)))
  "Returns md5sum for a message object of type 'msg_backup"
  "ff8688787aa97766b5311a5e39e25a80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_backup>)))
  "Returns full string definition for message of type '<msg_backup>"
  (cl:format cl:nil "uint8 throttle~%uint8 brake~%int8 steer~%bool handbrake~%uint8 gear~%bool engine~%uint8 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_backup)))
  "Returns full string definition for message of type 'msg_backup"
  (cl:format cl:nil "uint8 throttle~%uint8 brake~%int8 steer~%bool handbrake~%uint8 gear~%bool engine~%uint8 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_backup>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_backup>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_backup
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':brake (brake msg))
    (cl:cons ':steer (steer msg))
    (cl:cons ':handbrake (handbrake msg))
    (cl:cons ':gear (gear msg))
    (cl:cons ':engine (engine msg))
    (cl:cons ':speed (speed msg))
))
