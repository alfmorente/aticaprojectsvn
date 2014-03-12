; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_available.msg.html

(cl:defclass <msg_available> (roslisp-msg-protocol:ros-message)
  ((available
    :reader available
    :initarg :available
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 12 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass msg_available (<msg_available>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_available>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_available)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_available> is deprecated: use Common_files-msg:msg_available instead.")))

(cl:ensure-generic-function 'available-val :lambda-list '(m))
(cl:defmethod available-val ((m <msg_available>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:available-val is deprecated.  Use Common_files-msg:available instead.")
  (available m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_available>) ostream)
  "Serializes a message object of type '<msg_available>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'available))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_available>) istream)
  "Deserializes a message object of type '<msg_available>"
  (cl:setf (cl:slot-value msg 'available) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'available)))
    (cl:dotimes (i 12)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_available>)))
  "Returns string type for a message object of type '<msg_available>"
  "Common_files/msg_available")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_available)))
  "Returns string type for a message object of type 'msg_available"
  "Common_files/msg_available")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_available>)))
  "Returns md5sum for a message object of type '<msg_available>"
  "537c2fa61f5a5b28fe6de222ff385320")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_available)))
  "Returns md5sum for a message object of type 'msg_available"
  "537c2fa61f5a5b28fe6de222ff385320")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_available>)))
  "Returns full string definition for message of type '<msg_available>"
  (cl:format cl:nil "bool[12] available~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_available)))
  "Returns full string definition for message of type 'msg_available"
  (cl:format cl:nil "bool[12] available~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_available>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'available) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_available>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_available
    (cl:cons ':available (available msg))
))
