; Auto-generated. Do not edit!


(cl:in-package Modulo_Camaras-msg)


;//! \htmlinclude msg_camaras.msg.html

(cl:defclass <msg_camaras> (roslisp-msg-protocol:ros-message)
  ((imagen
    :reader imagen
    :initarg :imagen
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass msg_camaras (<msg_camaras>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_camaras>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_camaras)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Camaras-msg:<msg_camaras> is deprecated: use Modulo_Camaras-msg:msg_camaras instead.")))

(cl:ensure-generic-function 'imagen-val :lambda-list '(m))
(cl:defmethod imagen-val ((m <msg_camaras>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Camaras-msg:imagen-val is deprecated.  Use Modulo_Camaras-msg:imagen instead.")
  (imagen m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_camaras>) ostream)
  "Serializes a message object of type '<msg_camaras>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'imagen))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'imagen))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_camaras>) istream)
  "Deserializes a message object of type '<msg_camaras>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'imagen) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'imagen)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_camaras>)))
  "Returns string type for a message object of type '<msg_camaras>"
  "Modulo_Camaras/msg_camaras")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_camaras)))
  "Returns string type for a message object of type 'msg_camaras"
  "Modulo_Camaras/msg_camaras")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_camaras>)))
  "Returns md5sum for a message object of type '<msg_camaras>"
  "14d873e24c34adbd80ad4662f18456c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_camaras)))
  "Returns md5sum for a message object of type 'msg_camaras"
  "14d873e24c34adbd80ad4662f18456c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_camaras>)))
  "Returns full string definition for message of type '<msg_camaras>"
  (cl:format cl:nil "uint8[] imagen~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_camaras)))
  "Returns full string definition for message of type 'msg_camaras"
  (cl:format cl:nil "uint8[] imagen~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_camaras>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'imagen) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_camaras>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_camaras
    (cl:cons ':imagen (imagen msg))
))
