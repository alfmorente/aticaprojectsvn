; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_laser.msg.html

(cl:defclass <msg_laser> (roslisp-msg-protocol:ros-message)
  ((angulos
    :reader angulos
    :initarg :angulos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (distancias
    :reader distancias
    :initarg :distancias
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass msg_laser (<msg_laser>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_laser>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_laser)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_laser> is deprecated: use Modulo_Navegacion-msg:msg_laser instead.")))

(cl:ensure-generic-function 'angulos-val :lambda-list '(m))
(cl:defmethod angulos-val ((m <msg_laser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:angulos-val is deprecated.  Use Modulo_Navegacion-msg:angulos instead.")
  (angulos m))

(cl:ensure-generic-function 'distancias-val :lambda-list '(m))
(cl:defmethod distancias-val ((m <msg_laser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:distancias-val is deprecated.  Use Modulo_Navegacion-msg:distancias instead.")
  (distancias m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_laser>) ostream)
  "Serializes a message object of type '<msg_laser>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angulos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angulos))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distancias))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'distancias))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_laser>) istream)
  "Deserializes a message object of type '<msg_laser>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angulos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angulos)))
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
  (cl:setf (cl:slot-value msg 'distancias) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distancias)))
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
  "Modulo_Navegacion/msg_laser")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_laser)))
  "Returns string type for a message object of type 'msg_laser"
  "Modulo_Navegacion/msg_laser")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_laser>)))
  "Returns md5sum for a message object of type '<msg_laser>"
  "2583b36e4ed5b0750e4c27e6e92888c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_laser)))
  "Returns md5sum for a message object of type 'msg_laser"
  "2583b36e4ed5b0750e4c27e6e92888c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_laser>)))
  "Returns full string definition for message of type '<msg_laser>"
  (cl:format cl:nil "float32[] angulos~%float32[] distancias~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_laser)))
  "Returns full string definition for message of type 'msg_laser"
  (cl:format cl:nil "float32[] angulos~%float32[] distancias~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_laser>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angulos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distancias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_laser>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_laser
    (cl:cons ':angulos (angulos msg))
    (cl:cons ':distancias (distancias msg))
))
