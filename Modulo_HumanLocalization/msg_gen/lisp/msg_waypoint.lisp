; Auto-generated. Do not edit!


(cl:in-package Modulo_HumanLocalization-msg)


;//! \htmlinclude msg_waypoint.msg.html

(cl:defclass <msg_waypoint> (roslisp-msg-protocol:ros-message)
  ((waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass msg_waypoint (<msg_waypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_waypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_waypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_HumanLocalization-msg:<msg_waypoint> is deprecated: use Modulo_HumanLocalization-msg:msg_waypoint instead.")))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <msg_waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_HumanLocalization-msg:waypoints-val is deprecated.  Use Modulo_HumanLocalization-msg:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_waypoint>) ostream)
  "Serializes a message object of type '<msg_waypoint>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_waypoint>) istream)
  "Deserializes a message object of type '<msg_waypoint>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_waypoint>)))
  "Returns string type for a message object of type '<msg_waypoint>"
  "Modulo_HumanLocalization/msg_waypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_waypoint)))
  "Returns string type for a message object of type 'msg_waypoint"
  "Modulo_HumanLocalization/msg_waypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_waypoint>)))
  "Returns md5sum for a message object of type '<msg_waypoint>"
  "cb3f2bf066882d854358a8deef3110e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_waypoint)))
  "Returns md5sum for a message object of type 'msg_waypoint"
  "cb3f2bf066882d854358a8deef3110e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_waypoint>)))
  "Returns full string definition for message of type '<msg_waypoint>"
  (cl:format cl:nil "float32[] waypoints~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_waypoint)))
  "Returns full string definition for message of type 'msg_waypoint"
  (cl:format cl:nil "float32[] waypoints~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_waypoint>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_waypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_waypoint
    (cl:cons ':waypoints (waypoints msg))
))
