; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_camera.msg.html

(cl:defclass <msg_camera> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass msg_camera (<msg_camera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_camera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_camera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_camera> is deprecated: use Common_files-msg:msg_camera instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <msg_camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:image-val is deprecated.  Use Common_files-msg:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_camera>) ostream)
  "Serializes a message object of type '<msg_camera>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'image))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'image))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_camera>) istream)
  "Deserializes a message object of type '<msg_camera>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'image) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'image)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_camera>)))
  "Returns string type for a message object of type '<msg_camera>"
  "Common_files/msg_camera")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_camera)))
  "Returns string type for a message object of type 'msg_camera"
  "Common_files/msg_camera")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_camera>)))
  "Returns md5sum for a message object of type '<msg_camera>"
  "a7e4fad7b43f0e22a2b2fb0c9470f7d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_camera)))
  "Returns md5sum for a message object of type 'msg_camera"
  "a7e4fad7b43f0e22a2b2fb0c9470f7d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_camera>)))
  "Returns full string definition for message of type '<msg_camera>"
  (cl:format cl:nil "int8[] image~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_camera)))
  "Returns full string definition for message of type 'msg_camera"
  (cl:format cl:nil "int8[] image~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_camera>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'image) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_camera>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_camera
    (cl:cons ':image (image msg))
))
