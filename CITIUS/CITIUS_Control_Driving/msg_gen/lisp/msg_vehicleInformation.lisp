; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Driving-msg)


;//! \htmlinclude msg_vehicleInformation.msg.html

(cl:defclass <msg_vehicleInformation> (roslisp-msg-protocol:ros-message)
  ((id_device
    :reader id_device
    :initarg :id_device
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_vehicleInformation (<msg_vehicleInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_vehicleInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_vehicleInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Driving-msg:<msg_vehicleInformation> is deprecated: use CITIUS_Control_Driving-msg:msg_vehicleInformation instead.")))

(cl:ensure-generic-function 'id_device-val :lambda-list '(m))
(cl:defmethod id_device-val ((m <msg_vehicleInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:id_device-val is deprecated.  Use CITIUS_Control_Driving-msg:id_device instead.")
  (id_device m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_vehicleInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:value-val is deprecated.  Use CITIUS_Control_Driving-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_vehicleInformation>) ostream)
  "Serializes a message object of type '<msg_vehicleInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_device)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id_device)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_vehicleInformation>) istream)
  "Deserializes a message object of type '<msg_vehicleInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_device)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id_device)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_vehicleInformation>)))
  "Returns string type for a message object of type '<msg_vehicleInformation>"
  "CITIUS_Control_Driving/msg_vehicleInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_vehicleInformation)))
  "Returns string type for a message object of type 'msg_vehicleInformation"
  "CITIUS_Control_Driving/msg_vehicleInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_vehicleInformation>)))
  "Returns md5sum for a message object of type '<msg_vehicleInformation>"
  "99cf754747b52aeb199e99a3aa80459e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_vehicleInformation)))
  "Returns md5sum for a message object of type 'msg_vehicleInformation"
  "99cf754747b52aeb199e99a3aa80459e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_vehicleInformation>)))
  "Returns full string definition for message of type '<msg_vehicleInformation>"
  (cl:format cl:nil "uint16 id_device~%int16 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_vehicleInformation)))
  "Returns full string definition for message of type 'msg_vehicleInformation"
  (cl:format cl:nil "uint16 id_device~%int16 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_vehicleInformation>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_vehicleInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_vehicleInformation
    (cl:cons ':id_device (id_device msg))
    (cl:cons ':value (value msg))
))
