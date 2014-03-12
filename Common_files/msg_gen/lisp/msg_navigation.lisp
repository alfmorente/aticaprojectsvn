; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_navigation.msg.html

(cl:defclass <msg_navigation> (roslisp-msg-protocol:ros-message)
  ((lineal_velocity_x
    :reader lineal_velocity_x
    :initarg :lineal_velocity_x
    :type cl:float
    :initform 0.0)
   (lineal_velocity_y
    :reader lineal_velocity_y
    :initarg :lineal_velocity_y
    :type cl:float
    :initform 0.0)
   (lineal_velocity_z
    :reader lineal_velocity_z
    :initarg :lineal_velocity_z
    :type cl:float
    :initform 0.0)
   (angular_velocity_x
    :reader angular_velocity_x
    :initarg :angular_velocity_x
    :type cl:float
    :initform 0.0)
   (angular_velocity_y
    :reader angular_velocity_y
    :initarg :angular_velocity_y
    :type cl:float
    :initform 0.0)
   (angular_velocity_z
    :reader angular_velocity_z
    :initarg :angular_velocity_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass msg_navigation (<msg_navigation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_navigation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_navigation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_navigation> is deprecated: use Common_files-msg:msg_navigation instead.")))

(cl:ensure-generic-function 'lineal_velocity_x-val :lambda-list '(m))
(cl:defmethod lineal_velocity_x-val ((m <msg_navigation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:lineal_velocity_x-val is deprecated.  Use Common_files-msg:lineal_velocity_x instead.")
  (lineal_velocity_x m))

(cl:ensure-generic-function 'lineal_velocity_y-val :lambda-list '(m))
(cl:defmethod lineal_velocity_y-val ((m <msg_navigation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:lineal_velocity_y-val is deprecated.  Use Common_files-msg:lineal_velocity_y instead.")
  (lineal_velocity_y m))

(cl:ensure-generic-function 'lineal_velocity_z-val :lambda-list '(m))
(cl:defmethod lineal_velocity_z-val ((m <msg_navigation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:lineal_velocity_z-val is deprecated.  Use Common_files-msg:lineal_velocity_z instead.")
  (lineal_velocity_z m))

(cl:ensure-generic-function 'angular_velocity_x-val :lambda-list '(m))
(cl:defmethod angular_velocity_x-val ((m <msg_navigation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:angular_velocity_x-val is deprecated.  Use Common_files-msg:angular_velocity_x instead.")
  (angular_velocity_x m))

(cl:ensure-generic-function 'angular_velocity_y-val :lambda-list '(m))
(cl:defmethod angular_velocity_y-val ((m <msg_navigation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:angular_velocity_y-val is deprecated.  Use Common_files-msg:angular_velocity_y instead.")
  (angular_velocity_y m))

(cl:ensure-generic-function 'angular_velocity_z-val :lambda-list '(m))
(cl:defmethod angular_velocity_z-val ((m <msg_navigation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:angular_velocity_z-val is deprecated.  Use Common_files-msg:angular_velocity_z instead.")
  (angular_velocity_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_navigation>) ostream)
  "Serializes a message object of type '<msg_navigation>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lineal_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lineal_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lineal_velocity_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_velocity_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_navigation>) istream)
  "Deserializes a message object of type '<msg_navigation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lineal_velocity_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lineal_velocity_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lineal_velocity_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_navigation>)))
  "Returns string type for a message object of type '<msg_navigation>"
  "Common_files/msg_navigation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_navigation)))
  "Returns string type for a message object of type 'msg_navigation"
  "Common_files/msg_navigation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_navigation>)))
  "Returns md5sum for a message object of type '<msg_navigation>"
  "c59f4019725261608410c3332831e194")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_navigation)))
  "Returns md5sum for a message object of type 'msg_navigation"
  "c59f4019725261608410c3332831e194")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_navigation>)))
  "Returns full string definition for message of type '<msg_navigation>"
  (cl:format cl:nil "float64 lineal_velocity_x~%float64 lineal_velocity_y~%float64 lineal_velocity_z~%float64 angular_velocity_x~%float64 angular_velocity_y~%float64 angular_velocity_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_navigation)))
  "Returns full string definition for message of type 'msg_navigation"
  (cl:format cl:nil "float64 lineal_velocity_x~%float64 lineal_velocity_y~%float64 lineal_velocity_z~%float64 angular_velocity_x~%float64 angular_velocity_y~%float64 angular_velocity_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_navigation>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_navigation>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_navigation
    (cl:cons ':lineal_velocity_x (lineal_velocity_x msg))
    (cl:cons ':lineal_velocity_y (lineal_velocity_y msg))
    (cl:cons ':lineal_velocity_z (lineal_velocity_z msg))
    (cl:cons ':angular_velocity_x (angular_velocity_x msg))
    (cl:cons ':angular_velocity_y (angular_velocity_y msg))
    (cl:cons ':angular_velocity_z (angular_velocity_z msg))
))
