; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Manager-srv)


;//! \htmlinclude srv_status-request.msg.html

(cl:defclass <srv_status-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_status-request (<srv_status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_status-request> is deprecated: use CITIUS_Control_Manager-srv:srv_status-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <srv_status-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:status-val is deprecated.  Use CITIUS_Control_Manager-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_status-request>) ostream)
  "Serializes a message object of type '<srv_status-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_status-request>) istream)
  "Deserializes a message object of type '<srv_status-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_status-request>)))
  "Returns string type for a service object of type '<srv_status-request>"
  "CITIUS_Control_Manager/srv_statusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_status-request)))
  "Returns string type for a service object of type 'srv_status-request"
  "CITIUS_Control_Manager/srv_statusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_status-request>)))
  "Returns md5sum for a message object of type '<srv_status-request>"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_status-request)))
  "Returns md5sum for a message object of type 'srv_status-request"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_status-request>)))
  "Returns full string definition for message of type '<srv_status-request>"
  (cl:format cl:nil "uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_status-request)))
  "Returns full string definition for message of type 'srv_status-request"
  (cl:format cl:nil "uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_status-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_status-request
    (cl:cons ':status (status msg))
))
;//! \htmlinclude srv_status-response.msg.html

(cl:defclass <srv_status-response> (roslisp-msg-protocol:ros-message)
  ((confirmation
    :reader confirmation
    :initarg :confirmation
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_status-response (<srv_status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_status-response> is deprecated: use CITIUS_Control_Manager-srv:srv_status-response instead.")))

(cl:ensure-generic-function 'confirmation-val :lambda-list '(m))
(cl:defmethod confirmation-val ((m <srv_status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:confirmation-val is deprecated.  Use CITIUS_Control_Manager-srv:confirmation instead.")
  (confirmation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_status-response>) ostream)
  "Serializes a message object of type '<srv_status-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirmation) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_status-response>) istream)
  "Deserializes a message object of type '<srv_status-response>"
    (cl:setf (cl:slot-value msg 'confirmation) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_status-response>)))
  "Returns string type for a service object of type '<srv_status-response>"
  "CITIUS_Control_Manager/srv_statusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_status-response)))
  "Returns string type for a service object of type 'srv_status-response"
  "CITIUS_Control_Manager/srv_statusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_status-response>)))
  "Returns md5sum for a message object of type '<srv_status-response>"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_status-response)))
  "Returns md5sum for a message object of type 'srv_status-response"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_status-response>)))
  "Returns full string definition for message of type '<srv_status-response>"
  (cl:format cl:nil "bool confirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_status-response)))
  "Returns full string definition for message of type 'srv_status-response"
  (cl:format cl:nil "bool confirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_status-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_status-response
    (cl:cons ':confirmation (confirmation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_status)))
  'srv_status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_status)))
  'srv_status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_status)))
  "Returns string type for a service object of type '<srv_status>"
  "CITIUS_Control_Manager/srv_status")