; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_panAbsolutePosition-request.msg.html

(cl:defclass <srv_panAbsolutePosition-request> (roslisp-msg-protocol:ros-message)
  ((panPosition
    :reader panPosition
    :initarg :panPosition
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_panAbsolutePosition-request (<srv_panAbsolutePosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_panAbsolutePosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_panAbsolutePosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_panAbsolutePosition-request> is deprecated: use CITIUS_Control_Communication-srv:srv_panAbsolutePosition-request instead.")))

(cl:ensure-generic-function 'panPosition-val :lambda-list '(m))
(cl:defmethod panPosition-val ((m <srv_panAbsolutePosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:panPosition-val is deprecated.  Use CITIUS_Control_Communication-srv:panPosition instead.")
  (panPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_panAbsolutePosition-request>) ostream)
  "Serializes a message object of type '<srv_panAbsolutePosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'panPosition)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'panPosition)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_panAbsolutePosition-request>) istream)
  "Deserializes a message object of type '<srv_panAbsolutePosition-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'panPosition)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'panPosition)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_panAbsolutePosition-request>)))
  "Returns string type for a service object of type '<srv_panAbsolutePosition-request>"
  "CITIUS_Control_Communication/srv_panAbsolutePositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_panAbsolutePosition-request)))
  "Returns string type for a service object of type 'srv_panAbsolutePosition-request"
  "CITIUS_Control_Communication/srv_panAbsolutePositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_panAbsolutePosition-request>)))
  "Returns md5sum for a message object of type '<srv_panAbsolutePosition-request>"
  "00c8e1dcba95ff90c779d8aea4279dc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_panAbsolutePosition-request)))
  "Returns md5sum for a message object of type 'srv_panAbsolutePosition-request"
  "00c8e1dcba95ff90c779d8aea4279dc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_panAbsolutePosition-request>)))
  "Returns full string definition for message of type '<srv_panAbsolutePosition-request>"
  (cl:format cl:nil "uint16 panPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_panAbsolutePosition-request)))
  "Returns full string definition for message of type 'srv_panAbsolutePosition-request"
  (cl:format cl:nil "uint16 panPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_panAbsolutePosition-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_panAbsolutePosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_panAbsolutePosition-request
    (cl:cons ':panPosition (panPosition msg))
))
;//! \htmlinclude srv_panAbsolutePosition-response.msg.html

(cl:defclass <srv_panAbsolutePosition-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_panAbsolutePosition-response (<srv_panAbsolutePosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_panAbsolutePosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_panAbsolutePosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_panAbsolutePosition-response> is deprecated: use CITIUS_Control_Communication-srv:srv_panAbsolutePosition-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_panAbsolutePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_panAbsolutePosition-response>) ostream)
  "Serializes a message object of type '<srv_panAbsolutePosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_panAbsolutePosition-response>) istream)
  "Deserializes a message object of type '<srv_panAbsolutePosition-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_panAbsolutePosition-response>)))
  "Returns string type for a service object of type '<srv_panAbsolutePosition-response>"
  "CITIUS_Control_Communication/srv_panAbsolutePositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_panAbsolutePosition-response)))
  "Returns string type for a service object of type 'srv_panAbsolutePosition-response"
  "CITIUS_Control_Communication/srv_panAbsolutePositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_panAbsolutePosition-response>)))
  "Returns md5sum for a message object of type '<srv_panAbsolutePosition-response>"
  "00c8e1dcba95ff90c779d8aea4279dc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_panAbsolutePosition-response)))
  "Returns md5sum for a message object of type 'srv_panAbsolutePosition-response"
  "00c8e1dcba95ff90c779d8aea4279dc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_panAbsolutePosition-response>)))
  "Returns full string definition for message of type '<srv_panAbsolutePosition-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_panAbsolutePosition-response)))
  "Returns full string definition for message of type 'srv_panAbsolutePosition-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_panAbsolutePosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_panAbsolutePosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_panAbsolutePosition-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_panAbsolutePosition)))
  'srv_panAbsolutePosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_panAbsolutePosition)))
  'srv_panAbsolutePosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_panAbsolutePosition)))
  "Returns string type for a service object of type '<srv_panAbsolutePosition>"
  "CITIUS_Control_Communication/srv_panAbsolutePosition")