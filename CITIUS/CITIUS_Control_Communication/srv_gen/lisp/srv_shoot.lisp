; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_shoot-request.msg.html

(cl:defclass <srv_shoot-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass srv_shoot-request (<srv_shoot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_shoot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_shoot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_shoot-request> is deprecated: use CITIUS_Control_Communication-srv:srv_shoot-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_shoot-request>) ostream)
  "Serializes a message object of type '<srv_shoot-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_shoot-request>) istream)
  "Deserializes a message object of type '<srv_shoot-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_shoot-request>)))
  "Returns string type for a service object of type '<srv_shoot-request>"
  "CITIUS_Control_Communication/srv_shootRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_shoot-request)))
  "Returns string type for a service object of type 'srv_shoot-request"
  "CITIUS_Control_Communication/srv_shootRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_shoot-request>)))
  "Returns md5sum for a message object of type '<srv_shoot-request>"
  "e2cc9e9d8c464550830df49c160979ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_shoot-request)))
  "Returns md5sum for a message object of type 'srv_shoot-request"
  "e2cc9e9d8c464550830df49c160979ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_shoot-request>)))
  "Returns full string definition for message of type '<srv_shoot-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_shoot-request)))
  "Returns full string definition for message of type 'srv_shoot-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_shoot-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_shoot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_shoot-request
))
;//! \htmlinclude srv_shoot-response.msg.html

(cl:defclass <srv_shoot-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_shoot-response (<srv_shoot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_shoot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_shoot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_shoot-response> is deprecated: use CITIUS_Control_Communication-srv:srv_shoot-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <srv_shoot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:ret-val is deprecated.  Use CITIUS_Control_Communication-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_shoot-response>) ostream)
  "Serializes a message object of type '<srv_shoot-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ret) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_shoot-response>) istream)
  "Deserializes a message object of type '<srv_shoot-response>"
    (cl:setf (cl:slot-value msg 'ret) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_shoot-response>)))
  "Returns string type for a service object of type '<srv_shoot-response>"
  "CITIUS_Control_Communication/srv_shootResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_shoot-response)))
  "Returns string type for a service object of type 'srv_shoot-response"
  "CITIUS_Control_Communication/srv_shootResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_shoot-response>)))
  "Returns md5sum for a message object of type '<srv_shoot-response>"
  "e2cc9e9d8c464550830df49c160979ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_shoot-response)))
  "Returns md5sum for a message object of type 'srv_shoot-response"
  "e2cc9e9d8c464550830df49c160979ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_shoot-response>)))
  "Returns full string definition for message of type '<srv_shoot-response>"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_shoot-response)))
  "Returns full string definition for message of type 'srv_shoot-response"
  (cl:format cl:nil "bool ret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_shoot-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_shoot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_shoot-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_shoot)))
  'srv_shoot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_shoot)))
  'srv_shoot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_shoot)))
  "Returns string type for a service object of type '<srv_shoot>"
  "CITIUS_Control_Communication/srv_shoot")