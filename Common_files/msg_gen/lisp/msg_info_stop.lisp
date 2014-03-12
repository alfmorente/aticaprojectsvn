; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_info_stop.msg.html

(cl:defclass <msg_info_stop> (roslisp-msg-protocol:ros-message)
  ((id_event
    :reader id_event
    :initarg :id_event
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_info_stop (<msg_info_stop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_info_stop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_info_stop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_info_stop> is deprecated: use Common_files-msg:msg_info_stop instead.")))

(cl:ensure-generic-function 'id_event-val :lambda-list '(m))
(cl:defmethod id_event-val ((m <msg_info_stop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:id_event-val is deprecated.  Use Common_files-msg:id_event instead.")
  (id_event m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_info_stop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:value-val is deprecated.  Use Common_files-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_info_stop>) ostream)
  "Serializes a message object of type '<msg_info_stop>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_info_stop>) istream)
  "Deserializes a message object of type '<msg_info_stop>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_event)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_info_stop>)))
  "Returns string type for a message object of type '<msg_info_stop>"
  "Common_files/msg_info_stop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_info_stop)))
  "Returns string type for a message object of type 'msg_info_stop"
  "Common_files/msg_info_stop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_info_stop>)))
  "Returns md5sum for a message object of type '<msg_info_stop>"
  "ccec10d702be7a8dea85cb771958353c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_info_stop)))
  "Returns md5sum for a message object of type 'msg_info_stop"
  "ccec10d702be7a8dea85cb771958353c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_info_stop>)))
  "Returns full string definition for message of type '<msg_info_stop>"
  (cl:format cl:nil "uint8 id_event~%bool value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_info_stop)))
  "Returns full string definition for message of type 'msg_info_stop"
  (cl:format cl:nil "uint8 id_event~%bool value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_info_stop>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_info_stop>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_info_stop
    (cl:cons ':id_event (id_event msg))
    (cl:cons ':value (value msg))
))
