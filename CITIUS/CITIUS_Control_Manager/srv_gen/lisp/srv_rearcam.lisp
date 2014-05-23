; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Manager-srv)


;//! \htmlinclude srv_rearcam-request.msg.html

(cl:defclass <srv_rearcam-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass srv_rearcam-request (<srv_rearcam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_rearcam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_rearcam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_rearcam-request> is deprecated: use CITIUS_Control_Manager-srv:srv_rearcam-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <srv_rearcam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:request-val is deprecated.  Use CITIUS_Control_Manager-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_rearcam-request>) ostream)
  "Serializes a message object of type '<srv_rearcam-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_rearcam-request>) istream)
  "Deserializes a message object of type '<srv_rearcam-request>"
    (cl:setf (cl:slot-value msg 'request) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_rearcam-request>)))
  "Returns string type for a service object of type '<srv_rearcam-request>"
  "CITIUS_Control_Manager/srv_rearcamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_rearcam-request)))
  "Returns string type for a service object of type 'srv_rearcam-request"
  "CITIUS_Control_Manager/srv_rearcamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_rearcam-request>)))
  "Returns md5sum for a message object of type '<srv_rearcam-request>"
  "231efe7fe75158176932d3c27d259dcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_rearcam-request)))
  "Returns md5sum for a message object of type 'srv_rearcam-request"
  "231efe7fe75158176932d3c27d259dcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_rearcam-request>)))
  "Returns full string definition for message of type '<srv_rearcam-request>"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_rearcam-request)))
  "Returns full string definition for message of type 'srv_rearcam-request"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_rearcam-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_rearcam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_rearcam-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude srv_rearcam-response.msg.html

(cl:defclass <srv_rearcam-response> (roslisp-msg-protocol:ros-message)
  ((pan
    :reader pan
    :initarg :pan
    :type cl:fixnum
    :initform 0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srv_rearcam-response (<srv_rearcam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srv_rearcam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srv_rearcam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Manager-srv:<srv_rearcam-response> is deprecated: use CITIUS_Control_Manager-srv:srv_rearcam-response instead.")))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <srv_rearcam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:pan-val is deprecated.  Use CITIUS_Control_Manager-srv:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <srv_rearcam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Manager-srv:tilt-val is deprecated.  Use CITIUS_Control_Manager-srv:tilt instead.")
  (tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srv_rearcam-response>) ostream)
  "Serializes a message object of type '<srv_rearcam-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pan)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tilt)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srv_rearcam-response>) istream)
  "Deserializes a message object of type '<srv_rearcam-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pan)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tilt)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srv_rearcam-response>)))
  "Returns string type for a service object of type '<srv_rearcam-response>"
  "CITIUS_Control_Manager/srv_rearcamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_rearcam-response)))
  "Returns string type for a service object of type 'srv_rearcam-response"
  "CITIUS_Control_Manager/srv_rearcamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srv_rearcam-response>)))
  "Returns md5sum for a message object of type '<srv_rearcam-response>"
  "231efe7fe75158176932d3c27d259dcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srv_rearcam-response)))
  "Returns md5sum for a message object of type 'srv_rearcam-response"
  "231efe7fe75158176932d3c27d259dcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srv_rearcam-response>)))
  "Returns full string definition for message of type '<srv_rearcam-response>"
  (cl:format cl:nil "uint8 pan~%uint8 tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srv_rearcam-response)))
  "Returns full string definition for message of type 'srv_rearcam-response"
  (cl:format cl:nil "uint8 pan~%uint8 tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srv_rearcam-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srv_rearcam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srv_rearcam-response
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srv_rearcam)))
  'srv_rearcam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srv_rearcam)))
  'srv_rearcam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srv_rearcam)))
  "Returns string type for a service object of type '<srv_rearcam>"
  "CITIUS_Control_Manager/srv_rearcam")