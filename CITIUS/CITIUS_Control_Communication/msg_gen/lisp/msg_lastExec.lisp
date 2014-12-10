; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_lastExec.msg.html

(cl:defclass <msg_lastExec> (roslisp-msg-protocol:ros-message)
  ((badExec
    :reader badExec
    :initarg :badExec
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_lastExec (<msg_lastExec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_lastExec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_lastExec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_lastExec> is deprecated: use CITIUS_Control_Communication-msg:msg_lastExec instead.")))

(cl:ensure-generic-function 'badExec-val :lambda-list '(m))
(cl:defmethod badExec-val ((m <msg_lastExec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:badExec-val is deprecated.  Use CITIUS_Control_Communication-msg:badExec instead.")
  (badExec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_lastExec>) ostream)
  "Serializes a message object of type '<msg_lastExec>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'badExec) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_lastExec>) istream)
  "Deserializes a message object of type '<msg_lastExec>"
    (cl:setf (cl:slot-value msg 'badExec) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_lastExec>)))
  "Returns string type for a message object of type '<msg_lastExec>"
  "CITIUS_Control_Communication/msg_lastExec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_lastExec)))
  "Returns string type for a message object of type 'msg_lastExec"
  "CITIUS_Control_Communication/msg_lastExec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_lastExec>)))
  "Returns md5sum for a message object of type '<msg_lastExec>"
  "817eae779e751c980d6611de3cffc5b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_lastExec)))
  "Returns md5sum for a message object of type 'msg_lastExec"
  "817eae779e751c980d6611de3cffc5b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_lastExec>)))
  "Returns full string definition for message of type '<msg_lastExec>"
  (cl:format cl:nil "bool badExec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_lastExec)))
  "Returns full string definition for message of type 'msg_lastExec"
  (cl:format cl:nil "bool badExec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_lastExec>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_lastExec>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_lastExec
    (cl:cons ':badExec (badExec msg))
))
