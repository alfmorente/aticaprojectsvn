; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_waypoint.msg.html

(cl:defclass <msg_waypoint> (roslisp-msg-protocol:ros-message)
  ((wp_latitude
    :reader wp_latitude
    :initarg :wp_latitude
    :type cl:float
    :initform 0.0)
   (wp_longitude
    :reader wp_longitude
    :initarg :wp_longitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass msg_waypoint (<msg_waypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_waypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_waypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_waypoint> is deprecated: use Common_files-msg:msg_waypoint instead.")))

(cl:ensure-generic-function 'wp_latitude-val :lambda-list '(m))
(cl:defmethod wp_latitude-val ((m <msg_waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:wp_latitude-val is deprecated.  Use Common_files-msg:wp_latitude instead.")
  (wp_latitude m))

(cl:ensure-generic-function 'wp_longitude-val :lambda-list '(m))
(cl:defmethod wp_longitude-val ((m <msg_waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:wp_longitude-val is deprecated.  Use Common_files-msg:wp_longitude instead.")
  (wp_longitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_waypoint>) ostream)
  "Serializes a message object of type '<msg_waypoint>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wp_latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wp_longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_waypoint>) istream)
  "Deserializes a message object of type '<msg_waypoint>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wp_latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wp_longitude) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_waypoint>)))
  "Returns string type for a message object of type '<msg_waypoint>"
  "Common_files/msg_waypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_waypoint)))
  "Returns string type for a message object of type 'msg_waypoint"
  "Common_files/msg_waypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_waypoint>)))
  "Returns md5sum for a message object of type '<msg_waypoint>"
  "dc73a2bad69375b319a1fde550ba2cbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_waypoint)))
  "Returns md5sum for a message object of type 'msg_waypoint"
  "dc73a2bad69375b319a1fde550ba2cbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_waypoint>)))
  "Returns full string definition for message of type '<msg_waypoint>"
  (cl:format cl:nil "float64 wp_latitude~%float64 wp_longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_waypoint)))
  "Returns full string definition for message of type 'msg_waypoint"
  (cl:format cl:nil "float64 wp_latitude~%float64 wp_longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_waypoint>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_waypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_waypoint
    (cl:cons ':wp_latitude (wp_latitude msg))
    (cl:cons ':wp_longitude (wp_longitude msg))
))
