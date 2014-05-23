; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Manager-srv)


;//! \htmlinclude srv_electric-request.msg.html

(cl:defclass <srv_electric-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_electric-request (<srv_electric-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_electric-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_electric-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_electric-request> is deprecated: use CITIUS_Control_Manager-srv:srv_electric-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <srv_electric-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:request-val is deprecated.  Use CITIUS_Control_Manager-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_electric-request>) ostream)
  "Serializes a message object of type '<srv_electric-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_electric-request>) istream)
  "Deserializes a message object of type '<srv_electric-request>"
    (cl:setf (cl:slot-value msg 'request) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_electric-request>)))
  "Returns string type for a service object of type '<srv_electric-request>"
  "CITIUS_Control_Manager/srv_electricRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_electric-request)))
  "Returns string type for a service object of type 'srv_electric-request"
  "CITIUS_Control_Manager/srv_electricRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_electric-request>)))
  "Returns md5sum for a message object of type '<srv_electric-request>"
  "f931d616e7b5ac515d6f94740d46929b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_electric-request)))
  "Returns md5sum for a message object of type 'srv_electric-request"
  "f931d616e7b5ac515d6f94740d46929b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_electric-request>)))
  "Returns full string definition for message of type '<srv_electric-request>"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_electric-request)))
  "Returns full string definition for message of type 'srv_electric-request"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_electric-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_electric-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_electric-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude srv_electric-response.msg.html

(cl:defclass <srv_electric-response> (roslisp-msg-protocol:ros-message)
  ((batteryLevel
    :reader batteryLevel
    :initarg :batteryLevel
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_electric-response (<srv_electric-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_electric-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_electric-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_electric-response> is deprecated: use CITIUS_Control_Manager-srv:srv_electric-response instead.")))

(cl:ensure-generic-function 'batteryLevel-val :lambda-list '(m))
(cl:defmethod batteryLevel-val ((m <srv_electric-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:batteryLevel-val is deprecated.  Use CITIUS_Control_Manager-srv:batteryLevel instead.")
  (batteryLevel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_electric-response>) ostream)
  "Serializes a message object of type '<srv_electric-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'batteryLevel)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_electric-response>) istream)
  "Deserializes a message object of type '<srv_electric-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'batteryLevel)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_electric-response>)))
  "Returns string type for a service object of type '<srv_electric-response>"
  "CITIUS_Control_Manager/srv_electricResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_electric-response)))
  "Returns string type for a service object of type 'srv_electric-response"
  "CITIUS_Control_Manager/srv_electricResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_electric-response>)))
  "Returns md5sum for a message object of type '<srv_electric-response>"
  "f931d616e7b5ac515d6f94740d46929b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_electric-response)))
  "Returns md5sum for a message object of type 'srv_electric-response"
  "f931d616e7b5ac515d6f94740d46929b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_electric-response>)))
  "Returns full string definition for message of type '<srv_electric-response>"
  (cl:format cl:nil "uint8 batteryLevel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_electric-response)))
  "Returns full string definition for message of type 'srv_electric-response"
  (cl:format cl:nil "uint8 batteryLevel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_electric-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_electric-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_electric-response
    (cl:cons ':batteryLevel (batteryLevel msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_electric)))
  'srv_electric-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_electric)))
  'srv_electric-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_electric)))
  "Returns string type for a service object of type '<srv_electric>"
  "CITIUS_Control_Manager/srv_electric")