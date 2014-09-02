; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_echoesFound.msg.html

(cl:defclass <msg_echoesFound> (roslisp-msg-protocol:ros-message)
  ((echoesFound
    :reader echoesFound
    :initarg :echoesFound
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 5 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass msg_echoesFound (<msg_echoesFound>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_echoesFound>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_echoesFound)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_echoesFound> is deprecated: use CITIUS_Control_Communication-msg:msg_echoesFound instead.")))

(cl:ensure-generic-function 'echoesFound-val :lambda-list '(m))
(cl:defmethod echoesFound-val ((m <msg_echoesFound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:echoesFound-val is deprecated.  Use CITIUS_Control_Communication-msg:echoesFound instead.")
  (echoesFound m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_echoesFound>) ostream)
  "Serializes a message object of type '<msg_echoesFound>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'echoesFound))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_echoesFound>) istream)
  "Deserializes a message object of type '<msg_echoesFound>"
  (cl:setf (cl:slot-value msg 'echoesFound) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'echoesFound)))
    (cl:dotimes (i 5)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_echoesFound>)))
  "Returns string type for a message object of type '<msg_echoesFound>"
  "CITIUS_Control_Communication/msg_echoesFound")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_echoesFound)))
  "Returns string type for a message object of type 'msg_echoesFound"
  "CITIUS_Control_Communication/msg_echoesFound")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_echoesFound>)))
  "Returns md5sum for a message object of type '<msg_echoesFound>"
  "98ba2f76b1beb9b1da1c2ce72cf28ce7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_echoesFound)))
  "Returns md5sum for a message object of type 'msg_echoesFound"
  "98ba2f76b1beb9b1da1c2ce72cf28ce7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_echoesFound>)))
  "Returns full string definition for message of type '<msg_echoesFound>"
  (cl:format cl:nil "int16[5] echoesFound~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_echoesFound)))
  "Returns full string definition for message of type 'msg_echoesFound"
  (cl:format cl:nil "int16[5] echoesFound~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_echoesFound>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'echoesFound) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_echoesFound>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_echoesFound
    (cl:cons ':echoesFound (echoesFound msg))
))
