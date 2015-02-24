; Auto-generated. Do not edit!


(cl:in-package Driving_Bobcat-srv)


;//! \htmlinclude srv_vehicleStatus-request.msg.html

(cl:defclass <srv_vehicleStatus-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (posSwitcher
    :reader posSwitcher
    :initarg :posSwitcher
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_vehicleStatus-request (<srv_vehicleStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_vehicleStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_vehicleStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Driving_Bobcat-srv:<srv_vehicleStatus-request> is deprecated: use Driving_Bobcat-srv:srv_vehicleStatus-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <srv_vehicleStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Driving_Bobcat-srv:status-val is deprecated.  Use Driving_Bobcat-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'posSwitcher-val :lambda-list '(m))
(cl:defmethod posSwitcher-val ((m <srv_vehicleStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Driving_Bobcat-srv:posSwitcher-val is deprecated.  Use Driving_Bobcat-srv:posSwitcher instead.")
  (posSwitcher m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_vehicleStatus-request>) ostream)
  "Serializes a message object of type '<srv_vehicleStatus-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'posSwitcher)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_vehicleStatus-request>) istream)
  "Deserializes a message object of type '<srv_vehicleStatus-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'posSwitcher)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_vehicleStatus-request>)))
  "Returns string type for a service object of type '<srv_vehicleStatus-request>"
  "Driving_Bobcat/srv_vehicleStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicleStatus-request)))
  "Returns string type for a service object of type 'srv_vehicleStatus-request"
  "Driving_Bobcat/srv_vehicleStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_vehicleStatus-request>)))
  "Returns md5sum for a message object of type '<srv_vehicleStatus-request>"
  "1f3623f9c7abdd5ed10d69c51d0221ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_vehicleStatus-request)))
  "Returns md5sum for a message object of type 'srv_vehicleStatus-request"
  "1f3623f9c7abdd5ed10d69c51d0221ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_vehicleStatus-request>)))
  "Returns full string definition for message of type '<srv_vehicleStatus-request>"
  (cl:format cl:nil "uint8 status~%uint8 posSwitcher~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_vehicleStatus-request)))
  "Returns full string definition for message of type 'srv_vehicleStatus-request"
  (cl:format cl:nil "uint8 status~%uint8 posSwitcher~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_vehicleStatus-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_vehicleStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_vehicleStatus-request
    (cl:cons ':status (status msg))
    (cl:cons ':posSwitcher (posSwitcher msg))
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
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Driving_Bobcat-srv:<srv_vehicleStatus-response> is deprecated: use Driving_Bobcat-srv:srv_vehicleStatus-response instead.")))

(cl:ensure-generic-function 'confirmation-val :lambda-list '(m))
(cl:defmethod confirmation-val ((m <srv_vehicleStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Driving_Bobcat-srv:confirmation-val is deprecated.  Use Driving_Bobcat-srv:confirmation instead.")
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
  "Driving_Bobcat/srv_vehicleStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicleStatus-response)))
  "Returns string type for a service object of type 'srv_vehicleStatus-response"
  "Driving_Bobcat/srv_vehicleStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_vehicleStatus-response>)))
  "Returns md5sum for a message object of type '<srv_vehicleStatus-response>"
  "1f3623f9c7abdd5ed10d69c51d0221ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_vehicleStatus-response)))
  "Returns md5sum for a message object of type 'srv_vehicleStatus-response"
  "1f3623f9c7abdd5ed10d69c51d0221ac")
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
  "Driving_Bobcat/srv_vehicleStatus")