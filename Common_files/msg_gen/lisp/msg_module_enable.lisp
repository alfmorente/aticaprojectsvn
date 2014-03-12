; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_module_enable.msg.html

(cl:defclass <msg_module_enable> (roslisp-msg-protocol:ros-message)
  ((id_module
    :reader id_module
    :initarg :id_module
    :type cl:fixnum
    :initform 0)
   (submode
    :reader submode
    :initarg :submode
    :type cl:fixnum
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_module_enable (<msg_module_enable>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_module_enable>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_module_enable)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_module_enable> is deprecated: use Common_files-msg:msg_module_enable instead.")))

(cl:ensure-generic-function 'id_module-val :lambda-list '(m))
(cl:defmethod id_module-val ((m <msg_module_enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:id_module-val is deprecated.  Use Common_files-msg:id_module instead.")
  (id_module m))

(cl:ensure-generic-function 'submode-val :lambda-list '(m))
(cl:defmethod submode-val ((m <msg_module_enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:submode-val is deprecated.  Use Common_files-msg:submode instead.")
  (submode m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <msg_module_enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:status-val is deprecated.  Use Common_files-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_module_enable>) ostream)
  "Serializes a message object of type '<msg_module_enable>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_module)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'submode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_module_enable>) istream)
  "Deserializes a message object of type '<msg_module_enable>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_module)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'submode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_module_enable>)))
  "Returns string type for a message object of type '<msg_module_enable>"
  "Common_files/msg_module_enable")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_module_enable)))
  "Returns string type for a message object of type 'msg_module_enable"
  "Common_files/msg_module_enable")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_module_enable>)))
  "Returns md5sum for a message object of type '<msg_module_enable>"
  "b6c32f1e50d47219fc894a3e6073e52c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_module_enable)))
  "Returns md5sum for a message object of type 'msg_module_enable"
  "b6c32f1e50d47219fc894a3e6073e52c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_module_enable>)))
  "Returns full string definition for message of type '<msg_module_enable>"
  (cl:format cl:nil "uint8 id_module~%uint8 submode~%uint8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_module_enable)))
  "Returns full string definition for message of type 'msg_module_enable"
  (cl:format cl:nil "uint8 id_module~%uint8 submode~%uint8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_module_enable>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_module_enable>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_module_enable
    (cl:cons ':id_module (id_module msg))
    (cl:cons ':submode (submode msg))
    (cl:cons ':status (status msg))
))
