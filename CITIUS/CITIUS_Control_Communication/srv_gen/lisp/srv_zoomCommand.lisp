; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_zoomCommand-request.msg.html

(cl:defclass <srv_zoomCommand-request> (roslisp-msg-protocol:ros-message)
  ((zoomCommand
    :reader zoomCommand
    :initarg :zoomCommand
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_zoomCommand-request (<srv_zoomCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_zoomCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_zoomCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_zoomCommand-request> is deprecated: use CITIUS_Control_Communication-srv:srv_zoomCommand-request instead.")))

(cl:ensure-generic-function 'zoomCommand-val :lambda-list '(m))
(cl:defmethod zoomCommand-val ((m <srv_zoomCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:zoomCommand-val is deprecated.  Use CITIUS_Control_Communication-srv:zoomCommand instead.")
  (zoomCommand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_zoomCommand-request>) ostream)
  "Serializes a message object of type '<srv_zoomCommand-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'zoomCommand)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_zoomCommand-request>) istream)
  "Deserializes a message object of type '<srv_zoomCommand-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'zoomCommand)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_zoomCommand-request>)))
  "Returns string type for a service object of type '<srv_zoomCommand-request>"
  "CITIUS_Control_Communication/srv_zoomCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_zoomCommand-request)))
  "Returns string type for a service object of type 'srv_zoomCommand-request"
  "CITIUS_Control_Communication/srv_zoomCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_zoomCommand-request>)))
  "Returns md5sum for a message object of type '<srv_zoomCommand-request>"
  "94e28555ca7845c55d04dfa70b4c4cff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_zoomCommand-request)))
  "Returns md5sum for a message object of type 'srv_zoomCommand-request"
  "94e28555ca7845c55d04dfa70b4c4cff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_zoomCommand-request>)))
  "Returns full string definition for message of type '<srv_zoomCommand-request>"
  (cl:format cl:nil "uint8 zoomCommand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_zoomCommand-request)))
  "Returns full string definition for message of type 'srv_zoomCommand-request"
  (cl:format cl:nil "uint8 zoomCommand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_zoomCommand-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_zoomCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_zoomCommand-request
    (cl:cons ':zoomCommand (zoomCommand msg))
))
;//! \htmlinclude srv_zoomCommand-response.msg.html

(cl:defclass <srv_zoomCommand-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_zoomCommand-response (<srv_zoomCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_zoomCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_zoomCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_zoomCommand-response> is deprecated: use CITIUS_Control_Communication-srv:srv_zoomCommand-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_zoomCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_zoomCommand-response>) ostream)
  "Serializes a message object of type '<srv_zoomCommand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_zoomCommand-response>) istream)
  "Deserializes a message object of type '<srv_zoomCommand-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_zoomCommand-response>)))
  "Returns string type for a service object of type '<srv_zoomCommand-response>"
  "CITIUS_Control_Communication/srv_zoomCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_zoomCommand-response)))
  "Returns string type for a service object of type 'srv_zoomCommand-response"
  "CITIUS_Control_Communication/srv_zoomCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_zoomCommand-response>)))
  "Returns md5sum for a message object of type '<srv_zoomCommand-response>"
  "94e28555ca7845c55d04dfa70b4c4cff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_zoomCommand-response)))
  "Returns md5sum for a message object of type 'srv_zoomCommand-response"
  "94e28555ca7845c55d04dfa70b4c4cff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_zoomCommand-response>)))
  "Returns full string definition for message of type '<srv_zoomCommand-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_zoomCommand-response)))
  "Returns full string definition for message of type 'srv_zoomCommand-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_zoomCommand-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_zoomCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_zoomCommand-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_zoomCommand)))
  'srv_zoomCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_zoomCommand)))
  'srv_zoomCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_zoomCommand)))
  "Returns string type for a service object of type '<srv_zoomCommand>"
  "CITIUS_Control_Communication/srv_zoomCommand")