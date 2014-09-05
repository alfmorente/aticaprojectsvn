; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_tiltAbsolutePosition-request.msg.html

(cl:defclass <srv_tiltAbsolutePosition-request> (roslisp-msg-protocol:ros-message)
  ((tiltPosition
    :reader tiltPosition
    :initarg :tiltPosition
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_tiltAbsolutePosition-request (<srv_tiltAbsolutePosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_tiltAbsolutePosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_tiltAbsolutePosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_tiltAbsolutePosition-request> is deprecated: use CITIUS_Control_Communication-srv:srv_tiltAbsolutePosition-request instead.")))

(cl:ensure-generic-function 'tiltPosition-val :lambda-list '(m))
(cl:defmethod tiltPosition-val ((m <srv_tiltAbsolutePosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:tiltPosition-val is deprecated.  Use CITIUS_Control_Communication-srv:tiltPosition instead.")
  (tiltPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_tiltAbsolutePosition-request>) ostream)
  "Serializes a message object of type '<srv_tiltAbsolutePosition-request>"
  (cl:let* ((signed (cl:slot-value msg 'tiltPosition)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_tiltAbsolutePosition-request>) istream)
  "Deserializes a message object of type '<srv_tiltAbsolutePosition-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tiltPosition) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_tiltAbsolutePosition-request>)))
  "Returns string type for a service object of type '<srv_tiltAbsolutePosition-request>"
  "CITIUS_Control_Communication/srv_tiltAbsolutePositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_tiltAbsolutePosition-request)))
  "Returns string type for a service object of type 'srv_tiltAbsolutePosition-request"
  "CITIUS_Control_Communication/srv_tiltAbsolutePositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_tiltAbsolutePosition-request>)))
  "Returns md5sum for a message object of type '<srv_tiltAbsolutePosition-request>"
  "2b3eede2c56ea928c920c1e9b71325fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_tiltAbsolutePosition-request)))
  "Returns md5sum for a message object of type 'srv_tiltAbsolutePosition-request"
  "2b3eede2c56ea928c920c1e9b71325fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_tiltAbsolutePosition-request>)))
  "Returns full string definition for message of type '<srv_tiltAbsolutePosition-request>"
  (cl:format cl:nil "int16 tiltPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_tiltAbsolutePosition-request)))
  "Returns full string definition for message of type 'srv_tiltAbsolutePosition-request"
  (cl:format cl:nil "int16 tiltPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_tiltAbsolutePosition-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_tiltAbsolutePosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_tiltAbsolutePosition-request
    (cl:cons ':tiltPosition (tiltPosition msg))
))
;//! \htmlinclude srv_tiltAbsolutePosition-response.msg.html

(cl:defclass <srv_tiltAbsolutePosition-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_tiltAbsolutePosition-response (<srv_tiltAbsolutePosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_tiltAbsolutePosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_tiltAbsolutePosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_tiltAbsolutePosition-response> is deprecated: use CITIUS_Control_Communication-srv:srv_tiltAbsolutePosition-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_tiltAbsolutePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_tiltAbsolutePosition-response>) ostream)
  "Serializes a message object of type '<srv_tiltAbsolutePosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_tiltAbsolutePosition-response>) istream)
  "Deserializes a message object of type '<srv_tiltAbsolutePosition-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_tiltAbsolutePosition-response>)))
  "Returns string type for a service object of type '<srv_tiltAbsolutePosition-response>"
  "CITIUS_Control_Communication/srv_tiltAbsolutePositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_tiltAbsolutePosition-response)))
  "Returns string type for a service object of type 'srv_tiltAbsolutePosition-response"
  "CITIUS_Control_Communication/srv_tiltAbsolutePositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_tiltAbsolutePosition-response>)))
  "Returns md5sum for a message object of type '<srv_tiltAbsolutePosition-response>"
  "2b3eede2c56ea928c920c1e9b71325fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_tiltAbsolutePosition-response)))
  "Returns md5sum for a message object of type 'srv_tiltAbsolutePosition-response"
  "2b3eede2c56ea928c920c1e9b71325fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_tiltAbsolutePosition-response>)))
  "Returns full string definition for message of type '<srv_tiltAbsolutePosition-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_tiltAbsolutePosition-response)))
  "Returns full string definition for message of type 'srv_tiltAbsolutePosition-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_tiltAbsolutePosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_tiltAbsolutePosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_tiltAbsolutePosition-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_tiltAbsolutePosition)))
  'srv_tiltAbsolutePosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_tiltAbsolutePosition)))
  'srv_tiltAbsolutePosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_tiltAbsolutePosition)))
  "Returns string type for a service object of type '<srv_tiltAbsolutePosition>"
  "CITIUS_Control_Communication/srv_tiltAbsolutePosition")