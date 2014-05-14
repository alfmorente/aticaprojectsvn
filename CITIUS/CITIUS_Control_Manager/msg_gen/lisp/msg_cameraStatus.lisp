; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Manager-msg)


;//! \htmlinclude msg_cameraStatus.msg.html

(cl:defclass <msg_cameraStatus> (roslisp-msg-protocol:ros-message)
  ((test
    :reader test
    :initarg :test
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_cameraStatus (<msg_cameraStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_cameraStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_cameraStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-msg:<msg_cameraStatus> is deprecated: use CITIUS_Control_Manager-msg:msg_cameraStatus instead.")))

(cl:ensure-generic-function 'test-val :lambda-list '(m))
(cl:defmethod test-val ((m <msg_cameraStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-msg:test-val is deprecated.  Use CITIUS_Control_Manager-msg:test instead.")
  (test m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_cameraStatus>) ostream)
  "Serializes a message object of type '<msg_cameraStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'test)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_cameraStatus>) istream)
  "Deserializes a message object of type '<msg_cameraStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'test)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_cameraStatus>)))
  "Returns string type for a message object of type '<msg_cameraStatus>"
  "CITIUS_Control_Manager/msg_cameraStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_cameraStatus)))
  "Returns string type for a message object of type 'msg_cameraStatus"
  "CITIUS_Control_Manager/msg_cameraStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_cameraStatus>)))
  "Returns md5sum for a message object of type '<msg_cameraStatus>"
  "1f6e6cd7d3f3bd76dda35fffbcf752ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_cameraStatus)))
  "Returns md5sum for a message object of type 'msg_cameraStatus"
  "1f6e6cd7d3f3bd76dda35fffbcf752ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_cameraStatus>)))
  "Returns full string definition for message of type '<msg_cameraStatus>"
  (cl:format cl:nil "uint8 test~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_cameraStatus)))
  "Returns full string definition for message of type 'msg_cameraStatus"
  (cl:format cl:nil "uint8 test~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_cameraStatus>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_cameraStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_cameraStatus
    (cl:cons ':test (test msg))
))
