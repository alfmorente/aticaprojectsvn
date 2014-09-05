; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_autofocusMode-request.msg.html

(cl:defclass <srv_autofocusMode-request> (roslisp-msg-protocol:ros-message)
  ((autofocus
    :reader autofocus
    :initarg :autofocus
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_autofocusMode-request (<srv_autofocusMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_autofocusMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_autofocusMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_autofocusMode-request> is deprecated: use CITIUS_Control_Communication-srv:srv_autofocusMode-request instead.")))

(cl:ensure-generic-function 'autofocus-val :lambda-list '(m))
(cl:defmethod autofocus-val ((m <srv_autofocusMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:autofocus-val is deprecated.  Use CITIUS_Control_Communication-srv:autofocus instead.")
  (autofocus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_autofocusMode-request>) ostream)
  "Serializes a message object of type '<srv_autofocusMode-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'autofocus) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_autofocusMode-request>) istream)
  "Deserializes a message object of type '<srv_autofocusMode-request>"
    (cl:setf (cl:slot-value msg 'autofocus) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_autofocusMode-request>)))
  "Returns string type for a service object of type '<srv_autofocusMode-request>"
  "CITIUS_Control_Communication/srv_autofocusModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_autofocusMode-request)))
  "Returns string type for a service object of type 'srv_autofocusMode-request"
  "CITIUS_Control_Communication/srv_autofocusModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_autofocusMode-request>)))
  "Returns md5sum for a message object of type '<srv_autofocusMode-request>"
  "89ae402f3ba9f97f2f271d895d5d4ddd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_autofocusMode-request)))
  "Returns md5sum for a message object of type 'srv_autofocusMode-request"
  "89ae402f3ba9f97f2f271d895d5d4ddd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_autofocusMode-request>)))
  "Returns full string definition for message of type '<srv_autofocusMode-request>"
  (cl:format cl:nil "bool autofocus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_autofocusMode-request)))
  "Returns full string definition for message of type 'srv_autofocusMode-request"
  (cl:format cl:nil "bool autofocus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_autofocusMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_autofocusMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_autofocusMode-request
    (cl:cons ':autofocus (autofocus msg))
))
;//! \htmlinclude srv_autofocusMode-response.msg.html

(cl:defclass <srv_autofocusMode-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_autofocusMode-response (<srv_autofocusMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_autofocusMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_autofocusMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_autofocusMode-response> is deprecated: use CITIUS_Control_Communication-srv:srv_autofocusMode-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_autofocusMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_autofocusMode-response>) ostream)
  "Serializes a message object of type '<srv_autofocusMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_autofocusMode-response>) istream)
  "Deserializes a message object of type '<srv_autofocusMode-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_autofocusMode-response>)))
  "Returns string type for a service object of type '<srv_autofocusMode-response>"
  "CITIUS_Control_Communication/srv_autofocusModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_autofocusMode-response)))
  "Returns string type for a service object of type 'srv_autofocusMode-response"
  "CITIUS_Control_Communication/srv_autofocusModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_autofocusMode-response>)))
  "Returns md5sum for a message object of type '<srv_autofocusMode-response>"
  "89ae402f3ba9f97f2f271d895d5d4ddd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_autofocusMode-response)))
  "Returns md5sum for a message object of type 'srv_autofocusMode-response"
  "89ae402f3ba9f97f2f271d895d5d4ddd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_autofocusMode-response>)))
  "Returns full string definition for message of type '<srv_autofocusMode-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_autofocusMode-response)))
  "Returns full string definition for message of type 'srv_autofocusMode-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_autofocusMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_autofocusMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_autofocusMode-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_autofocusMode)))
  'srv_autofocusMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_autofocusMode)))
  'srv_autofocusMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_autofocusMode)))
  "Returns string type for a service object of type '<srv_autofocusMode>"
  "CITIUS_Control_Communication/srv_autofocusMode")