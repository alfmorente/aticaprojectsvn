; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_dzoom-request.msg.html

(cl:defclass <srv_dzoom-request> (roslisp-msg-protocol:ros-message)
  ((newZoom
    :reader newZoom
    :initarg :newZoom
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_dzoom-request (<srv_dzoom-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_dzoom-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_dzoom-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_dzoom-request> is deprecated: use CITIUS_Control_Communication-srv:srv_dzoom-request instead.")))

(cl:ensure-generic-function 'newZoom-val :lambda-list '(m))
(cl:defmethod newZoom-val ((m <srv_dzoom-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:newZoom-val is deprecated.  Use CITIUS_Control_Communication-srv:newZoom instead.")
  (newZoom m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_dzoom-request>) ostream)
  "Serializes a message object of type '<srv_dzoom-request>"
  (cl:let* ((signed (cl:slot-value msg 'newZoom)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_dzoom-request>) istream)
  "Deserializes a message object of type '<srv_dzoom-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'newZoom) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_dzoom-request>)))
  "Returns string type for a service object of type '<srv_dzoom-request>"
  "CITIUS_Control_Communication/srv_dzoomRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_dzoom-request)))
  "Returns string type for a service object of type 'srv_dzoom-request"
  "CITIUS_Control_Communication/srv_dzoomRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_dzoom-request>)))
  "Returns md5sum for a message object of type '<srv_dzoom-request>"
  "61ad2f4b1da8b259e5c027b0d68e3b71")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_dzoom-request)))
  "Returns md5sum for a message object of type 'srv_dzoom-request"
  "61ad2f4b1da8b259e5c027b0d68e3b71")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_dzoom-request>)))
  "Returns full string definition for message of type '<srv_dzoom-request>"
  (cl:format cl:nil "int8 newZoom~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_dzoom-request)))
  "Returns full string definition for message of type 'srv_dzoom-request"
  (cl:format cl:nil "int8 newZoom~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_dzoom-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_dzoom-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_dzoom-request
    (cl:cons ':newZoom (newZoom msg))
))
;//! \htmlinclude srv_dzoom-response.msg.html

(cl:defclass <srv_dzoom-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_dzoom-response (<srv_dzoom-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_dzoom-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_dzoom-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_dzoom-response> is deprecated: use CITIUS_Control_Communication-srv:srv_dzoom-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_dzoom-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_dzoom-response>) ostream)
  "Serializes a message object of type '<srv_dzoom-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_dzoom-response>) istream)
  "Deserializes a message object of type '<srv_dzoom-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_dzoom-response>)))
  "Returns string type for a service object of type '<srv_dzoom-response>"
  "CITIUS_Control_Communication/srv_dzoomResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_dzoom-response)))
  "Returns string type for a service object of type 'srv_dzoom-response"
  "CITIUS_Control_Communication/srv_dzoomResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_dzoom-response>)))
  "Returns md5sum for a message object of type '<srv_dzoom-response>"
  "61ad2f4b1da8b259e5c027b0d68e3b71")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_dzoom-response)))
  "Returns md5sum for a message object of type 'srv_dzoom-response"
  "61ad2f4b1da8b259e5c027b0d68e3b71")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_dzoom-response>)))
  "Returns full string definition for message of type '<srv_dzoom-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_dzoom-response)))
  "Returns full string definition for message of type 'srv_dzoom-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_dzoom-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_dzoom-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_dzoom-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_dzoom)))
  'srv_dzoom-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_dzoom)))
  'srv_dzoom-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_dzoom)))
  "Returns string type for a service object of type '<srv_dzoom>"
  "CITIUS_Control_Communication/srv_dzoom")