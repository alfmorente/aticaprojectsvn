; Auto-generated. Do not edit!


(cl:in-package Modulo_Gest_Sistema-msg)


;//! \htmlinclude msg_available_mode.msg.html

(cl:defclass <msg_available_mode> (roslisp-msg-protocol:ros-message)
  ((available_mode
    :reader available_mode
    :initarg :available_mode
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 12 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass msg_available_mode (<msg_available_mode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_available_mode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_available_mode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Gest_Sistema-msg:<msg_available_mode> is deprecated: use Modulo_Gest_Sistema-msg:msg_available_mode instead.")))

(cl:ensure-generic-function 'available_mode-val :lambda-list '(m))
(cl:defmethod available_mode-val ((m <msg_available_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Gest_Sistema-msg:available_mode-val is deprecated.  Use Modulo_Gest_Sistema-msg:available_mode instead.")
  (available_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_available_mode>) ostream)
  "Serializes a message object of type '<msg_available_mode>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'available_mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_available_mode>) istream)
  "Deserializes a message object of type '<msg_available_mode>"
  (cl:setf (cl:slot-value msg 'available_mode) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'available_mode)))
    (cl:dotimes (i 12)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_available_mode>)))
  "Returns string type for a message object of type '<msg_available_mode>"
  "Modulo_Gest_Sistema/msg_available_mode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_available_mode)))
  "Returns string type for a message object of type 'msg_available_mode"
  "Modulo_Gest_Sistema/msg_available_mode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_available_mode>)))
  "Returns md5sum for a message object of type '<msg_available_mode>"
  "3d0fd04f4ced9660c43caab85d5c5253")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_available_mode)))
  "Returns md5sum for a message object of type 'msg_available_mode"
  "3d0fd04f4ced9660c43caab85d5c5253")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_available_mode>)))
  "Returns full string definition for message of type '<msg_available_mode>"
  (cl:format cl:nil "bool[12] available_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_available_mode)))
  "Returns full string definition for message of type 'msg_available_mode"
  (cl:format cl:nil "bool[12] available_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_available_mode>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'available_mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_available_mode>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_available_mode
    (cl:cons ':available_mode (available_mode msg))
))
