; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_polarity-request.msg.html

(cl:defclass <srv_polarity-request> (roslisp-msg-protocol:ros-message)
  ((newPolarity
    :reader newPolarity
    :initarg :newPolarity
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_polarity-request (<srv_polarity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_polarity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_polarity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_polarity-request> is deprecated: use CITIUS_Control_Communication-srv:srv_polarity-request instead.")))

(cl:ensure-generic-function 'newPolarity-val :lambda-list '(m))
(cl:defmethod newPolarity-val ((m <srv_polarity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:newPolarity-val is deprecated.  Use CITIUS_Control_Communication-srv:newPolarity instead.")
  (newPolarity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_polarity-request>) ostream)
  "Serializes a message object of type '<srv_polarity-request>"
  (cl:let* ((signed (cl:slot-value msg 'newPolarity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_polarity-request>) istream)
  "Deserializes a message object of type '<srv_polarity-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'newPolarity) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_polarity-request>)))
  "Returns string type for a service object of type '<srv_polarity-request>"
  "CITIUS_Control_Communication/srv_polarityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_polarity-request)))
  "Returns string type for a service object of type 'srv_polarity-request"
  "CITIUS_Control_Communication/srv_polarityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_polarity-request>)))
  "Returns md5sum for a message object of type '<srv_polarity-request>"
  "127bb913cce0f3eeeb8bb8d8486eee81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_polarity-request)))
  "Returns md5sum for a message object of type 'srv_polarity-request"
  "127bb913cce0f3eeeb8bb8d8486eee81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_polarity-request>)))
  "Returns full string definition for message of type '<srv_polarity-request>"
  (cl:format cl:nil "int8 newPolarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_polarity-request)))
  "Returns full string definition for message of type 'srv_polarity-request"
  (cl:format cl:nil "int8 newPolarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_polarity-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_polarity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_polarity-request
    (cl:cons ':newPolarity (newPolarity msg))
))
;//! \htmlinclude srv_polarity-response.msg.html

(cl:defclass <srv_polarity-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_polarity-response (<srv_polarity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_polarity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_polarity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_polarity-response> is deprecated: use CITIUS_Control_Communication-srv:srv_polarity-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_polarity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_polarity-response>) ostream)
  "Serializes a message object of type '<srv_polarity-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_polarity-response>) istream)
  "Deserializes a message object of type '<srv_polarity-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_polarity-response>)))
  "Returns string type for a service object of type '<srv_polarity-response>"
  "CITIUS_Control_Communication/srv_polarityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_polarity-response)))
  "Returns string type for a service object of type 'srv_polarity-response"
  "CITIUS_Control_Communication/srv_polarityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_polarity-response>)))
  "Returns md5sum for a message object of type '<srv_polarity-response>"
  "127bb913cce0f3eeeb8bb8d8486eee81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_polarity-response)))
  "Returns md5sum for a message object of type 'srv_polarity-response"
  "127bb913cce0f3eeeb8bb8d8486eee81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_polarity-response>)))
  "Returns full string definition for message of type '<srv_polarity-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_polarity-response)))
  "Returns full string definition for message of type 'srv_polarity-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_polarity-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_polarity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_polarity-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_polarity)))
  'srv_polarity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_polarity)))
  'srv_polarity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_polarity)))
  "Returns string type for a service object of type '<srv_polarity>"
  "CITIUS_Control_Communication/srv_polarity")