; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_error.msg.html

(cl:defclass <msg_error> (roslisp-msg-protocol:ros-message)
  ((id_subsystem
    :reader id_subsystem
    :initarg :id_subsystem
    :type cl:fixnum
    :initform 0)
   (id_error
    :reader id_error
    :initarg :id_error
    :type cl:fixnum
    :initform 0)
   (type_error
    :reader type_error
    :initarg :type_error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_error (<msg_error>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_error>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_error)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_error> is deprecated: use Common_files-msg:msg_error instead.")))

(cl:ensure-generic-function 'id_subsystem-val :lambda-list '(m))
(cl:defmethod id_subsystem-val ((m <msg_error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:id_subsystem-val is deprecated.  Use Common_files-msg:id_subsystem instead.")
  (id_subsystem m))

(cl:ensure-generic-function 'id_error-val :lambda-list '(m))
(cl:defmethod id_error-val ((m <msg_error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:id_error-val is deprecated.  Use Common_files-msg:id_error instead.")
  (id_error m))

(cl:ensure-generic-function 'type_error-val :lambda-list '(m))
(cl:defmethod type_error-val ((m <msg_error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:type_error-val is deprecated.  Use Common_files-msg:type_error instead.")
  (type_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_error>) ostream)
  "Serializes a message object of type '<msg_error>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_subsystem)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id_error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type_error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_error>) istream)
  "Deserializes a message object of type '<msg_error>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_subsystem)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id_error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type_error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_error>)))
  "Returns string type for a message object of type '<msg_error>"
  "Common_files/msg_error")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_error)))
  "Returns string type for a message object of type 'msg_error"
  "Common_files/msg_error")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_error>)))
  "Returns md5sum for a message object of type '<msg_error>"
  "4c66157bef5e8ecc9fc1d60c5e56acf8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_error)))
  "Returns md5sum for a message object of type 'msg_error"
  "4c66157bef5e8ecc9fc1d60c5e56acf8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_error>)))
  "Returns full string definition for message of type '<msg_error>"
  (cl:format cl:nil "uint8 id_subsystem ~%uint16 id_error~%uint8 type_error  ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_error)))
  "Returns full string definition for message of type 'msg_error"
  (cl:format cl:nil "uint8 id_subsystem ~%uint16 id_error~%uint8 type_error  ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_error>))
  (cl:+ 0
     1
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_error>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_error
    (cl:cons ':id_subsystem (id_subsystem msg))
    (cl:cons ':id_error (id_error msg))
    (cl:cons ':type_error (type_error msg))
))
