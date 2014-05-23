; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Manager-srv)


;//! \htmlinclude srv_vehicle-request.msg.html

(cl:defclass <srv_vehicle-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_vehicle-request (<srv_vehicle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_vehicle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_vehicle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_vehicle-request> is deprecated: use CITIUS_Control_Manager-srv:srv_vehicle-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <srv_vehicle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:request-val is deprecated.  Use CITIUS_Control_Manager-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_vehicle-request>) ostream)
  "Serializes a message object of type '<srv_vehicle-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_vehicle-request>) istream)
  "Deserializes a message object of type '<srv_vehicle-request>"
    (cl:setf (cl:slot-value msg 'request) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_vehicle-request>)))
  "Returns string type for a service object of type '<srv_vehicle-request>"
  "CITIUS_Control_Manager/srv_vehicleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicle-request)))
  "Returns string type for a service object of type 'srv_vehicle-request"
  "CITIUS_Control_Manager/srv_vehicleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_vehicle-request>)))
  "Returns md5sum for a message object of type '<srv_vehicle-request>"
  "89265b91e72c65b55fffce25bc80e81f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_vehicle-request)))
  "Returns md5sum for a message object of type 'srv_vehicle-request"
  "89265b91e72c65b55fffce25bc80e81f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_vehicle-request>)))
  "Returns full string definition for message of type '<srv_vehicle-request>"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_vehicle-request)))
  "Returns full string definition for message of type 'srv_vehicle-request"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_vehicle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_vehicle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_vehicle-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude srv_vehicle-response.msg.html

(cl:defclass <srv_vehicle-response> (roslisp-msg-protocol:ros-message)
  ((presentThottle
    :reader presentThottle
    :initarg :presentThottle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_vehicle-response (<srv_vehicle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_vehicle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_vehicle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_vehicle-response> is deprecated: use CITIUS_Control_Manager-srv:srv_vehicle-response instead.")))

(cl:ensure-generic-function 'presentThottle-val :lambda-list '(m))
(cl:defmethod presentThottle-val ((m <srv_vehicle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:presentThottle-val is deprecated.  Use CITIUS_Control_Manager-srv:presentThottle instead.")
  (presentThottle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_vehicle-response>) ostream)
  "Serializes a message object of type '<srv_vehicle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'presentThottle)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_vehicle-response>) istream)
  "Deserializes a message object of type '<srv_vehicle-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'presentThottle)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_vehicle-response>)))
  "Returns string type for a service object of type '<srv_vehicle-response>"
  "CITIUS_Control_Manager/srv_vehicleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicle-response)))
  "Returns string type for a service object of type 'srv_vehicle-response"
  "CITIUS_Control_Manager/srv_vehicleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_vehicle-response>)))
  "Returns md5sum for a message object of type '<srv_vehicle-response>"
  "89265b91e72c65b55fffce25bc80e81f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_vehicle-response)))
  "Returns md5sum for a message object of type 'srv_vehicle-response"
  "89265b91e72c65b55fffce25bc80e81f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_vehicle-response>)))
  "Returns full string definition for message of type '<srv_vehicle-response>"
  (cl:format cl:nil "uint8 presentThottle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_vehicle-response)))
  "Returns full string definition for message of type 'srv_vehicle-response"
  (cl:format cl:nil "uint8 presentThottle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_vehicle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_vehicle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_vehicle-response
    (cl:cons ':presentThottle (presentThottle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_vehicle)))
  'srv_vehicle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_vehicle)))
  'srv_vehicle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_vehicle)))
  "Returns string type for a service object of type '<srv_vehicle>"
  "CITIUS_Control_Manager/srv_vehicle")