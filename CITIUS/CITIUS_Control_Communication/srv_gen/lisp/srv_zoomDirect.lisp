; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_zoomDirect-request.msg.html

(cl:defclass <srv_zoomDirect-request> (roslisp-msg-protocol:ros-message)
  ((zoomDirect
    :reader zoomDirect
    :initarg :zoomDirect
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_zoomDirect-request (<srv_zoomDirect-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_zoomDirect-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_zoomDirect-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_zoomDirect-request> is deprecated: use CITIUS_Control_Communication-srv:srv_zoomDirect-request instead.")))

(cl:ensure-generic-function 'zoomDirect-val :lambda-list '(m))
(cl:defmethod zoomDirect-val ((m <srv_zoomDirect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:zoomDirect-val is deprecated.  Use CITIUS_Control_Communication-srv:zoomDirect instead.")
  (zoomDirect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_zoomDirect-request>) ostream)
  "Serializes a message object of type '<srv_zoomDirect-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'zoomDirect)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_zoomDirect-request>) istream)
  "Deserializes a message object of type '<srv_zoomDirect-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'zoomDirect)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_zoomDirect-request>)))
  "Returns string type for a service object of type '<srv_zoomDirect-request>"
  "CITIUS_Control_Communication/srv_zoomDirectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_zoomDirect-request)))
  "Returns string type for a service object of type 'srv_zoomDirect-request"
  "CITIUS_Control_Communication/srv_zoomDirectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_zoomDirect-request>)))
  "Returns md5sum for a message object of type '<srv_zoomDirect-request>"
  "6b8da5ba7c2df6d7c8f7d5823242fbd2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_zoomDirect-request)))
  "Returns md5sum for a message object of type 'srv_zoomDirect-request"
  "6b8da5ba7c2df6d7c8f7d5823242fbd2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_zoomDirect-request>)))
  "Returns full string definition for message of type '<srv_zoomDirect-request>"
  (cl:format cl:nil "uint8 zoomDirect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_zoomDirect-request)))
  "Returns full string definition for message of type 'srv_zoomDirect-request"
  (cl:format cl:nil "uint8 zoomDirect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_zoomDirect-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_zoomDirect-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_zoomDirect-request
    (cl:cons ':zoomDirect (zoomDirect msg))
))
;//! \htmlinclude srv_zoomDirect-response.msg.html

(cl:defclass <srv_zoomDirect-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_zoomDirect-response (<srv_zoomDirect-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_zoomDirect-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_zoomDirect-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_zoomDirect-response> is deprecated: use CITIUS_Control_Communication-srv:srv_zoomDirect-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_zoomDirect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_zoomDirect-response>) ostream)
  "Serializes a message object of type '<srv_zoomDirect-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_zoomDirect-response>) istream)
  "Deserializes a message object of type '<srv_zoomDirect-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_zoomDirect-response>)))
  "Returns string type for a service object of type '<srv_zoomDirect-response>"
  "CITIUS_Control_Communication/srv_zoomDirectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_zoomDirect-response)))
  "Returns string type for a service object of type 'srv_zoomDirect-response"
  "CITIUS_Control_Communication/srv_zoomDirectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_zoomDirect-response>)))
  "Returns md5sum for a message object of type '<srv_zoomDirect-response>"
  "6b8da5ba7c2df6d7c8f7d5823242fbd2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_zoomDirect-response)))
  "Returns md5sum for a message object of type 'srv_zoomDirect-response"
  "6b8da5ba7c2df6d7c8f7d5823242fbd2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_zoomDirect-response>)))
  "Returns full string definition for message of type '<srv_zoomDirect-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_zoomDirect-response)))
  "Returns full string definition for message of type 'srv_zoomDirect-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_zoomDirect-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_zoomDirect-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_zoomDirect-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_zoomDirect)))
  'srv_zoomDirect-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_zoomDirect)))
  'srv_zoomDirect-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_zoomDirect)))
  "Returns string type for a service object of type '<srv_zoomDirect>"
  "CITIUS_Control_Communication/srv_zoomDirect")