; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_panRate-request.msg.html

(cl:defclass <srv_panRate-request> (roslisp-msg-protocol:ros-message)
  ((panRate
    :reader panRate
    :initarg :panRate
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_panRate-request (<srv_panRate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_panRate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_panRate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_panRate-request> is deprecated: use CITIUS_Control_Communication-srv:srv_panRate-request instead.")))

(cl:ensure-generic-function 'panRate-val :lambda-list '(m))
(cl:defmethod panRate-val ((m <srv_panRate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:panRate-val is deprecated.  Use CITIUS_Control_Communication-srv:panRate instead.")
  (panRate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_panRate-request>) ostream)
  "Serializes a message object of type '<srv_panRate-request>"
  (cl:let* ((signed (cl:slot-value msg 'panRate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_panRate-request>) istream)
  "Deserializes a message object of type '<srv_panRate-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'panRate) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_panRate-request>)))
  "Returns string type for a service object of type '<srv_panRate-request>"
  "CITIUS_Control_Communication/srv_panRateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_panRate-request)))
  "Returns string type for a service object of type 'srv_panRate-request"
  "CITIUS_Control_Communication/srv_panRateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_panRate-request>)))
  "Returns md5sum for a message object of type '<srv_panRate-request>"
  "e327e73d1cceeaf12d338e6b1d6a5c0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_panRate-request)))
  "Returns md5sum for a message object of type 'srv_panRate-request"
  "e327e73d1cceeaf12d338e6b1d6a5c0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_panRate-request>)))
  "Returns full string definition for message of type '<srv_panRate-request>"
  (cl:format cl:nil "int8 panRate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_panRate-request)))
  "Returns full string definition for message of type 'srv_panRate-request"
  (cl:format cl:nil "int8 panRate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_panRate-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_panRate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_panRate-request
    (cl:cons ':panRate (panRate msg))
))
;//! \htmlinclude srv_panRate-response.msg.html

(cl:defclass <srv_panRate-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_panRate-response (<srv_panRate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_panRate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_panRate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_panRate-response> is deprecated: use CITIUS_Control_Communication-srv:srv_panRate-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_panRate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_panRate-response>) ostream)
  "Serializes a message object of type '<srv_panRate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_panRate-response>) istream)
  "Deserializes a message object of type '<srv_panRate-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_panRate-response>)))
  "Returns string type for a service object of type '<srv_panRate-response>"
  "CITIUS_Control_Communication/srv_panRateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_panRate-response)))
  "Returns string type for a service object of type 'srv_panRate-response"
  "CITIUS_Control_Communication/srv_panRateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_panRate-response>)))
  "Returns md5sum for a message object of type '<srv_panRate-response>"
  "e327e73d1cceeaf12d338e6b1d6a5c0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_panRate-response)))
  "Returns md5sum for a message object of type 'srv_panRate-response"
  "e327e73d1cceeaf12d338e6b1d6a5c0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_panRate-response>)))
  "Returns full string definition for message of type '<srv_panRate-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_panRate-response)))
  "Returns full string definition for message of type 'srv_panRate-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_panRate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_panRate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_panRate-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_panRate)))
  'srv_panRate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_panRate)))
  'srv_panRate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_panRate)))
  "Returns string type for a service object of type '<srv_panRate>"
  "CITIUS_Control_Communication/srv_panRate")