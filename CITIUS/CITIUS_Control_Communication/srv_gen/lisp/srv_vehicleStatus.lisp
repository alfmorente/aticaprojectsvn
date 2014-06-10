; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-srv)


;//! \htmlinclude srv_vehicleStatus-request.msg.html

(cl:defclass <srv_vehicleStatus-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_vehicleStatus-request (<srv_vehicleStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_vehicleStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_vehicleStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_vehicleStatus-request> is deprecated: use CITIUS_Control_Communication-srv:srv_vehicleStatus-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <srv_vehicleStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:status-val is deprecated.  Use CITIUS_Control_Communication-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_vehicleStatus-request>) ostream)
  "Serializes a message object of type '<srv_vehicleStatus-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_vehicleStatus-request>) istream)
  "Deserializes a message object of type '<srv_vehicleStatus-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_vehicleStatus-request>)))
  "Returns string type for a service object of type '<srv_vehicleStatus-request>"
  "CITIUS_Control_Communication/srv_vehicleStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicleStatus-request)))
  "Returns string type for a service object of type 'srv_vehicleStatus-request"
  "CITIUS_Control_Communication/srv_vehicleStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_vehicleStatus-request>)))
  "Returns md5sum for a message object of type '<srv_vehicleStatus-request>"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_vehicleStatus-request)))
  "Returns md5sum for a message object of type 'srv_vehicleStatus-request"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_vehicleStatus-request>)))
  "Returns full string definition for message of type '<srv_vehicleStatus-request>"
  (cl:format cl:nil "uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_vehicleStatus-request)))
  "Returns full string definition for message of type 'srv_vehicleStatus-request"
  (cl:format cl:nil "uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_vehicleStatus-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_vehicleStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_vehicleStatus-request
    (cl:cons ':status (status msg))
))
;//! \htmlinclude srv_vehicleStatus-response.msg.html

(cl:defclass <srv_vehicleStatus-response> (roslisp-msg-protocol:ros-message)
  ((confirmation
    :reader confirmation
    :initarg :confirmation
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_vehicleStatus-response (<srv_vehicleStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_vehicleStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_vehicleStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-srv:<srv_vehicleStatus-response> is deprecated: use CITIUS_Control_Communication-srv:srv_vehicleStatus-response instead.")))

(cl:ensure-generic-function 'confirmation-val :lambda-list '(m))
(cl:defmethod confirmation-val ((m <srv_vehicleStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-srv:confirmation-val is deprecated.  Use CITIUS_Control_Communication-srv:confirmation instead.")
  (confirmation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_vehicleStatus-response>) ostream)
  "Serializes a message object of type '<srv_vehicleStatus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirmation) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_vehicleStatus-response>) istream)
  "Deserializes a message object of type '<srv_vehicleStatus-response>"
    (cl:setf (cl:slot-value msg 'confirmation) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_vehicleStatus-response>)))
  "Returns string type for a service object of type '<srv_vehicleStatus-response>"
  "CITIUS_Control_Communication/srv_vehicleStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicleStatus-response)))
  "Returns string type for a service object of type 'srv_vehicleStatus-response"
  "CITIUS_Control_Communication/srv_vehicleStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_vehicleStatus-response>)))
  "Returns md5sum for a message object of type '<srv_vehicleStatus-response>"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_vehicleStatus-response)))
  "Returns md5sum for a message object of type 'srv_vehicleStatus-response"
  "2bec2362253a5964940299039dd0e122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_vehicleStatus-response>)))
  "Returns full string definition for message of type '<srv_vehicleStatus-response>"
  (cl:format cl:nil "bool confirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_vehicleStatus-response)))
  "Returns full string definition for message of type 'srv_vehicleStatus-response"
  (cl:format cl:nil "bool confirmation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_vehicleStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_vehicleStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_vehicleStatus-response
    (cl:cons ':confirmation (confirmation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_vehicleStatus)))
  'srv_vehicleStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_vehicleStatus)))
  'srv_vehicleStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicleStatus)))
  "Returns string type for a service object of type '<srv_vehicleStatus>"
  "CITIUS_Control_Communication/srv_vehicleStatus")