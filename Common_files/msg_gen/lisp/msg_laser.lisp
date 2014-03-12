; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_laser.msg.html

(cl:defclass <msg_laser> (roslisp-msg-protocol:ros-message)
  ((id_laser
    :reader id_laser
    :initarg :id_laser
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (distance
    :reader distance
    :initarg :distance
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass msg_laser (<msg_laser>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_laser>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_laser)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_laser> is deprecated: use Common_files-msg:msg_laser instead.")))

(cl:ensure-generic-function 'id_laser-val :lambda-list '(m))
(cl:defmethod id_laser-val ((m <msg_laser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:id_laser-val is deprecated.  Use Common_files-msg:id_laser instead.")
  (id_laser m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <msg_laser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:angle-val is deprecated.  Use Common_files-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <msg_laser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:distance-val is deprecated.  Use Common_files-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_laser>) ostream)
  "Serializes a message object of type '<msg_laser>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_laser)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angle))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'distance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_laser>) istream)
  "Deserializes a message object of type '<msg_laser>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_laser)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angle)))
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
  (cl:setf (cl:slot-value msg 'distance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_laser>)))
  "Returns string type for a message object of type '<msg_laser>"
  "Common_files/msg_laser")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_laser)))
  "Returns string type for a message object of type 'msg_laser"
  "Common_files/msg_laser")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_laser>)))
  "Returns md5sum for a message object of type '<msg_laser>"
  "266af0804b5efb7b7f04ed8c5eec96b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_laser)))
  "Returns md5sum for a message object of type 'msg_laser"
  "266af0804b5efb7b7f04ed8c5eec96b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_laser>)))
  "Returns full string definition for message of type '<msg_laser>"
  (cl:format cl:nil "uint8 id_laser~%float32[] angle~%float32[] distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_laser)))
  "Returns full string definition for message of type 'msg_laser"
  (cl:format cl:nil "uint8 id_laser~%float32[] angle~%float32[] distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_laser>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_laser>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_laser
    (cl:cons ':id_laser (id_laser msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':distance (distance msg))
))
