; Auto-generated. Do not edit!


(cl:in-package Modulo_GPS-msg)


;//! \htmlinclude msg_stream.msg.html

(cl:defclass <msg_stream> (roslisp-msg-protocol:ros-message)
  ((id_file
    :reader id_file
    :initarg :id_file
    :type cl:fixnum
    :initform 0)
   (stream
    :reader stream
    :initarg :stream
    :type cl:string
    :initform ""))
)

(cl:defclass msg_stream (<msg_stream>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_stream>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_stream)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_GPS-msg:<msg_stream> is deprecated: use Modulo_GPS-msg:msg_stream instead.")))

(cl:ensure-generic-function 'id_file-val :lambda-list '(m))
(cl:defmethod id_file-val ((m <msg_stream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_GPS-msg:id_file-val is deprecated.  Use Modulo_GPS-msg:id_file instead.")
  (id_file m))

(cl:ensure-generic-function 'stream-val :lambda-list '(m))
(cl:defmethod stream-val ((m <msg_stream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_GPS-msg:stream-val is deprecated.  Use Modulo_GPS-msg:stream instead.")
  (stream m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_stream>) ostream)
  "Serializes a message object of type '<msg_stream>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_file)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'stream))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'stream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_stream>) istream)
  "Deserializes a message object of type '<msg_stream>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_file)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stream) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'stream) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_stream>)))
  "Returns string type for a message object of type '<msg_stream>"
  "Modulo_GPS/msg_stream")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_stream)))
  "Returns string type for a message object of type 'msg_stream"
  "Modulo_GPS/msg_stream")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_stream>)))
  "Returns md5sum for a message object of type '<msg_stream>"
  "3a57dd0fae35fb2128ca62b071bdcfb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_stream)))
  "Returns md5sum for a message object of type 'msg_stream"
  "3a57dd0fae35fb2128ca62b071bdcfb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_stream>)))
  "Returns full string definition for message of type '<msg_stream>"
  (cl:format cl:nil "uint8 id_file~%string stream~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_stream)))
  "Returns full string definition for message of type 'msg_stream"
  (cl:format cl:nil "uint8 id_file~%string stream~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_stream>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'stream))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_stream>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_stream
    (cl:cons ':id_file (id_file msg))
    (cl:cons ':stream (stream msg))
))
