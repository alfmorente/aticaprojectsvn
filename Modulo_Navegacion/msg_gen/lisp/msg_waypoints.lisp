; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_waypoints.msg.html

(cl:defclass <msg_waypoints> (roslisp-msg-protocol:ros-message)
  ((waypoint_lon
    :reader waypoint_lon
    :initarg :waypoint_lon
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (waypoint_lat
    :reader waypoint_lat
    :initarg :waypoint_lat
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (num_waypoints
    :reader num_waypoints
    :initarg :num_waypoints
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_waypoints (<msg_waypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_waypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_waypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_waypoints> is deprecated: use Modulo_Navegacion-msg:msg_waypoints instead.")))

(cl:ensure-generic-function 'waypoint_lon-val :lambda-list '(m))
(cl:defmethod waypoint_lon-val ((m <msg_waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:waypoint_lon-val is deprecated.  Use Modulo_Navegacion-msg:waypoint_lon instead.")
  (waypoint_lon m))

(cl:ensure-generic-function 'waypoint_lat-val :lambda-list '(m))
(cl:defmethod waypoint_lat-val ((m <msg_waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:waypoint_lat-val is deprecated.  Use Modulo_Navegacion-msg:waypoint_lat instead.")
  (waypoint_lat m))

(cl:ensure-generic-function 'num_waypoints-val :lambda-list '(m))
(cl:defmethod num_waypoints-val ((m <msg_waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:num_waypoints-val is deprecated.  Use Modulo_Navegacion-msg:num_waypoints instead.")
  (num_waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_waypoints>) ostream)
  "Serializes a message object of type '<msg_waypoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoint_lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waypoint_lon))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoint_lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waypoint_lat))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_waypoints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_waypoints)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_waypoints>) istream)
  "Deserializes a message object of type '<msg_waypoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoint_lon) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoint_lon)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoint_lat) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoint_lat)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_waypoints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_waypoints)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_waypoints>)))
  "Returns string type for a message object of type '<msg_waypoints>"
  "Modulo_Navegacion/msg_waypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_waypoints)))
  "Returns string type for a message object of type 'msg_waypoints"
  "Modulo_Navegacion/msg_waypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_waypoints>)))
  "Returns md5sum for a message object of type '<msg_waypoints>"
  "ae90d0e56988d7d8973a5bda929d228c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_waypoints)))
  "Returns md5sum for a message object of type 'msg_waypoints"
  "ae90d0e56988d7d8973a5bda929d228c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_waypoints>)))
  "Returns full string definition for message of type '<msg_waypoints>"
  (cl:format cl:nil "float32[] waypoint_lon~%float32[] waypoint_lat~%uint16 num_waypoints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_waypoints)))
  "Returns full string definition for message of type 'msg_waypoints"
  (cl:format cl:nil "float32[] waypoint_lon~%float32[] waypoint_lat~%uint16 num_waypoints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_waypoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoint_lon) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoint_lat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_waypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_waypoints
    (cl:cons ':waypoint_lon (waypoint_lon msg))
    (cl:cons ':waypoint_lat (waypoint_lat msg))
    (cl:cons ':num_waypoints (num_waypoints msg))
))
