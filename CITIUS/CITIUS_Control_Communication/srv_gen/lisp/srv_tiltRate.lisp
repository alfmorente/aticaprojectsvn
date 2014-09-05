; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_tiltRate-request.msg.html

(cl:defclass <srv_tiltRate-request> (roslisp-msg-protocol:ros-message)
  ((tiltRate
    :reader tiltRate
    :initarg :tiltRate
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_tiltRate-request (<srv_tiltRate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_tiltRate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_tiltRate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_tiltRate-request> is deprecated: use CITIUS_Control_Communication-srv:srv_tiltRate-request instead.")))

(cl:ensure-generic-function 'tiltRate-val :lambda-list '(m))
(cl:defmethod tiltRate-val ((m <srv_tiltRate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:tiltRate-val is deprecated.  Use CITIUS_Control_Communication-srv:tiltRate instead.")
  (tiltRate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_tiltRate-request>) ostream)
  "Serializes a message object of type '<srv_tiltRate-request>"
  (cl:let* ((signed (cl:slot-value msg 'tiltRate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_tiltRate-request>) istream)
  "Deserializes a message object of type '<srv_tiltRate-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tiltRate) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_tiltRate-request>)))
  "Returns string type for a service object of type '<srv_tiltRate-request>"
  "CITIUS_Control_Communication/srv_tiltRateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_tiltRate-request)))
  "Returns string type for a service object of type 'srv_tiltRate-request"
  "CITIUS_Control_Communication/srv_tiltRateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_tiltRate-request>)))
  "Returns md5sum for a message object of type '<srv_tiltRate-request>"
  "c4e94f79a5e00eebf3479a12e9d466dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_tiltRate-request)))
  "Returns md5sum for a message object of type 'srv_tiltRate-request"
  "c4e94f79a5e00eebf3479a12e9d466dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_tiltRate-request>)))
  "Returns full string definition for message of type '<srv_tiltRate-request>"
  (cl:format cl:nil "int8 tiltRate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_tiltRate-request)))
  "Returns full string definition for message of type 'srv_tiltRate-request"
  (cl:format cl:nil "int8 tiltRate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_tiltRate-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_tiltRate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_tiltRate-request
    (cl:cons ':tiltRate (tiltRate msg))
))
;//! \htmlinclude srv_tiltRate-response.msg.html

(cl:defclass <srv_tiltRate-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_tiltRate-response (<srv_tiltRate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_tiltRate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_tiltRate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_tiltRate-response> is deprecated: use CITIUS_Control_Communication-srv:srv_tiltRate-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_tiltRate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_tiltRate-response>) ostream)
  "Serializes a message object of type '<srv_tiltRate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_tiltRate-response>) istream)
  "Deserializes a message object of type '<srv_tiltRate-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_tiltRate-response>)))
  "Returns string type for a service object of type '<srv_tiltRate-response>"
  "CITIUS_Control_Communication/srv_tiltRateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_tiltRate-response)))
  "Returns string type for a service object of type 'srv_tiltRate-response"
  "CITIUS_Control_Communication/srv_tiltRateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_tiltRate-response>)))
  "Returns md5sum for a message object of type '<srv_tiltRate-response>"
  "c4e94f79a5e00eebf3479a12e9d466dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_tiltRate-response)))
  "Returns md5sum for a message object of type 'srv_tiltRate-response"
  "c4e94f79a5e00eebf3479a12e9d466dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_tiltRate-response>)))
  "Returns full string definition for message of type '<srv_tiltRate-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_tiltRate-response)))
  "Returns full string definition for message of type 'srv_tiltRate-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_tiltRate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_tiltRate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_tiltRate-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_tiltRate)))
  'srv_tiltRate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_tiltRate)))
  'srv_tiltRate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_tiltRate)))
  "Returns string type for a service object of type '<srv_tiltRate>"
  "CITIUS_Control_Communication/srv_tiltRate")