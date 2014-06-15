; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_FrontCamera-srv)


;//! \htmlinclude srv_nodeStatus-request.msg.html

(cl:defclass <srv_nodeStatus-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_nodeStatus-request (<srv_nodeStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_nodeStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_nodeStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_FrontCamera-srv:<srv_nodeStatus-request> is deprecated: use CITIUS_Control_FrontCamera-srv:srv_nodeStatus-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <srv_nodeStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_FrontCamera-srv:status-val is deprecated.  Use CITIUS_Control_FrontCamera-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_nodeStatus-request>) ostream)
  "Serializes a message object of type '<srv_nodeStatus-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_nodeStatus-request>) istream)
  "Deserializes a message object of type '<srv_nodeStatus-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_nodeStatus-request>)))
  "Returns string type for a service object of type '<srv_nodeStatus-request>"
  "CITIUS_Control_FrontCamera/srv_nodeStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_nodeStatus-request)))
  "Returns string type for a service object of type 'srv_nodeStatus-request"
  "CITIUS_Control_FrontCamera/srv_nodeStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_nodeStatus-request>)))
  "Returns md5sum for a message object of type '<srv_nodeStatus-request>"
  "a8f417cfe7a1c68eb9250d0c4d4a4bb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_nodeStatus-request)))
  "Returns md5sum for a message object of type 'srv_nodeStatus-request"
  "a8f417cfe7a1c68eb9250d0c4d4a4bb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_nodeStatus-request>)))
  "Returns full string definition for message of type '<srv_nodeStatus-request>"
  (cl:format cl:nil "uint16 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_nodeStatus-request)))
  "Returns full string definition for message of type 'srv_nodeStatus-request"
  (cl:format cl:nil "uint16 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_nodeStatus-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_nodeStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_nodeStatus-request
    (cl:cons ':status (status msg))
))
;//! \htmlinclude srv_nodeStatus-response.msg.html

(cl:defclass <srv_nodeStatus-response> (roslisp-msg-protocol:ros-message)
  ((confirmation
    :reader confirmation
    :initarg :confirmation
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_nodeStatus-response (<srv_nodeStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_nodeStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_nodeStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_FrontCamera-srv:<srv_nodeStatus-response> is deprecated: use CITIUS_Control_FrontCamera-srv:srv_nodeStatus-response instead.")))

(cl:ensure-generic-function 'confirmation-val :lambda-list '(m))
(cl:defmethod confirmation-val ((m <srv_nodeStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_FrontCamera-srv:confirmation-val is deprecated.  Use CITIUS_Control_FrontCamera-srv:confirmation instead.")
  (confirmation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_nodeStatus-response>) ostream)
  "Serializes a message object of type '<srv_nodeStatus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirmation) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_nodeStatus-response>) istream)
  "Deserializes a message object of type '<srv_nodeStatus-response>"
    (cl:setf (cl:slot-value msg 'confirmation) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_nodeStatus-response>)))
  "Returns string type for a service object of type '<srv_nodeStatus-response>"
  "CITIUS_Control_FrontCamera/srv_nodeStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_nodeStatus-response)))
  "Returns string type for a service object of type 'srv_nodeStatus-response"
  "CITIUS_Control_FrontCamera/srv_nodeStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_nodeStatus-response>)))
  "Returns md5sum for a message object of type '<srv_nodeStatus-response>"
  "a8f417cfe7a1c68eb9250d0c4d4a4bb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_nodeStatus-response)))
  "Returns md5sum for a message object of type 'srv_nodeStatus-response"
  "a8f417cfe7a1c68eb9250d0c4d4a4bb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_nodeStatus-response>)))
  "Returns full string definition for message of type '<srv_nodeStatus-response>"
  (cl:format cl:nil "bool confirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_nodeStatus-response)))
  "Returns full string definition for message of type 'srv_nodeStatus-response"
  (cl:format cl:nil "bool confirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_nodeStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_nodeStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_nodeStatus-response
    (cl:cons ':confirmation (confirmation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_nodeStatus)))
  'srv_nodeStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_nodeStatus)))
  'srv_nodeStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_nodeStatus)))
  "Returns string type for a service object of type '<srv_nodeStatus>"
  "CITIUS_Control_FrontCamera/srv_nodeStatus")