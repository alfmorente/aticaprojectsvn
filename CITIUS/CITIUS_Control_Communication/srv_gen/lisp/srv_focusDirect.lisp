; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_focusDirect-request.msg.html

(cl:defclass <srv_focusDirect-request> (roslisp-msg-protocol:ros-message)
  ((focusDirect
    :reader focusDirect
    :initarg :focusDirect
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_focusDirect-request (<srv_focusDirect-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_focusDirect-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_focusDirect-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_focusDirect-request> is deprecated: use CITIUS_Control_Communication-srv:srv_focusDirect-request instead.")))

(cl:ensure-generic-function 'focusDirect-val :lambda-list '(m))
(cl:defmethod focusDirect-val ((m <srv_focusDirect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:focusDirect-val is deprecated.  Use CITIUS_Control_Communication-srv:focusDirect instead.")
  (focusDirect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_focusDirect-request>) ostream)
  "Serializes a message object of type '<srv_focusDirect-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'focusDirect)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_focusDirect-request>) istream)
  "Deserializes a message object of type '<srv_focusDirect-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'focusDirect)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_focusDirect-request>)))
  "Returns string type for a service object of type '<srv_focusDirect-request>"
  "CITIUS_Control_Communication/srv_focusDirectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_focusDirect-request)))
  "Returns string type for a service object of type 'srv_focusDirect-request"
  "CITIUS_Control_Communication/srv_focusDirectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_focusDirect-request>)))
  "Returns md5sum for a message object of type '<srv_focusDirect-request>"
  "a4dfa2720320ab393b086bef03523e56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_focusDirect-request)))
  "Returns md5sum for a message object of type 'srv_focusDirect-request"
  "a4dfa2720320ab393b086bef03523e56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_focusDirect-request>)))
  "Returns full string definition for message of type '<srv_focusDirect-request>"
  (cl:format cl:nil "uint8 focusDirect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_focusDirect-request)))
  "Returns full string definition for message of type 'srv_focusDirect-request"
  (cl:format cl:nil "uint8 focusDirect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_focusDirect-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_focusDirect-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_focusDirect-request
    (cl:cons ':focusDirect (focusDirect msg))
))
;//! \htmlinclude srv_focusDirect-response.msg.html

(cl:defclass <srv_focusDirect-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_focusDirect-response (<srv_focusDirect-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_focusDirect-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_focusDirect-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_focusDirect-response> is deprecated: use CITIUS_Control_Communication-srv:srv_focusDirect-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_focusDirect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_focusDirect-response>) ostream)
  "Serializes a message object of type '<srv_focusDirect-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_focusDirect-response>) istream)
  "Deserializes a message object of type '<srv_focusDirect-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_focusDirect-response>)))
  "Returns string type for a service object of type '<srv_focusDirect-response>"
  "CITIUS_Control_Communication/srv_focusDirectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_focusDirect-response)))
  "Returns string type for a service object of type 'srv_focusDirect-response"
  "CITIUS_Control_Communication/srv_focusDirectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_focusDirect-response>)))
  "Returns md5sum for a message object of type '<srv_focusDirect-response>"
  "a4dfa2720320ab393b086bef03523e56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_focusDirect-response)))
  "Returns md5sum for a message object of type 'srv_focusDirect-response"
  "a4dfa2720320ab393b086bef03523e56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_focusDirect-response>)))
  "Returns full string definition for message of type '<srv_focusDirect-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_focusDirect-response)))
  "Returns full string definition for message of type 'srv_focusDirect-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_focusDirect-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_focusDirect-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_focusDirect-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_focusDirect)))
  'srv_focusDirect-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_focusDirect)))
  'srv_focusDirect-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_focusDirect)))
  "Returns string type for a service object of type '<srv_focusDirect>"
  "CITIUS_Control_Communication/srv_focusDirect")