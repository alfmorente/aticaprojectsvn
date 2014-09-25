; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_electricInfo.msg.html

(cl:defclass <msg_electricInfo> (roslisp-msg-protocol:ros-message)
  ((battery_level
    :reader battery_level
    :initarg :battery_level
    :type cl:fixnum
    :initform 0)
   (battery_voltage
    :reader battery_voltage
    :initarg :battery_voltage
    :type cl:fixnum
    :initform 0)
   (battery_current
    :reader battery_current
    :initarg :battery_current
    :type cl:fixnum
    :initform 0)
   (battery_temperature
    :reader battery_temperature
    :initarg :battery_temperature
    :type cl:fixnum
    :initform 0)
   (supply_alarms
    :reader supply_alarms
    :initarg :supply_alarms
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_electricInfo (<msg_electricInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_electricInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_electricInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_electricInfo> is deprecated: use CITIUS_Control_Communication-msg:msg_electricInfo instead.")))

(cl:ensure-generic-function 'battery_level-val :lambda-list '(m))
(cl:defmethod battery_level-val ((m <msg_electricInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:battery_level-val is deprecated.  Use CITIUS_Control_Communication-msg:battery_level instead.")
  (battery_level m))

(cl:ensure-generic-function 'battery_voltage-val :lambda-list '(m))
(cl:defmethod battery_voltage-val ((m <msg_electricInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:battery_voltage-val is deprecated.  Use CITIUS_Control_Communication-msg:battery_voltage instead.")
  (battery_voltage m))

(cl:ensure-generic-function 'battery_current-val :lambda-list '(m))
(cl:defmethod battery_current-val ((m <msg_electricInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:battery_current-val is deprecated.  Use CITIUS_Control_Communication-msg:battery_current instead.")
  (battery_current m))

(cl:ensure-generic-function 'battery_temperature-val :lambda-list '(m))
(cl:defmethod battery_temperature-val ((m <msg_electricInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:battery_temperature-val is deprecated.  Use CITIUS_Control_Communication-msg:battery_temperature instead.")
  (battery_temperature m))

(cl:ensure-generic-function 'supply_alarms-val :lambda-list '(m))
(cl:defmethod supply_alarms-val ((m <msg_electricInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:supply_alarms-val is deprecated.  Use CITIUS_Control_Communication-msg:supply_alarms instead.")
  (supply_alarms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_electricInfo>) ostream)
  "Serializes a message object of type '<msg_electricInfo>"
  (cl:let* ((signed (cl:slot-value msg 'battery_level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'battery_voltage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'battery_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'battery_temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_alarms)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_electricInfo>) istream)
  "Deserializes a message object of type '<msg_electricInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'battery_level) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'battery_voltage) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'battery_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'battery_temperature) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_alarms)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_electricInfo>)))
  "Returns string type for a message object of type '<msg_electricInfo>"
  "CITIUS_Control_Communication/msg_electricInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_electricInfo)))
  "Returns string type for a message object of type 'msg_electricInfo"
  "CITIUS_Control_Communication/msg_electricInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_electricInfo>)))
  "Returns md5sum for a message object of type '<msg_electricInfo>"
  "d42ed0969069aa6805076160fc2ef03d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_electricInfo)))
  "Returns md5sum for a message object of type 'msg_electricInfo"
  "d42ed0969069aa6805076160fc2ef03d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_electricInfo>)))
  "Returns full string definition for message of type '<msg_electricInfo>"
  (cl:format cl:nil "int16 battery_level~%int16 battery_voltage~%int16 battery_current~%int16 battery_temperature~%uint8 supply_alarms~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_electricInfo)))
  "Returns full string definition for message of type 'msg_electricInfo"
  (cl:format cl:nil "int16 battery_level~%int16 battery_voltage~%int16 battery_current~%int16 battery_temperature~%uint8 supply_alarms~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_electricInfo>))
  (cl:+ 0
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_electricInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_electricInfo
    (cl:cons ':battery_level (battery_level msg))
    (cl:cons ':battery_voltage (battery_voltage msg))
    (cl:cons ':battery_current (battery_current msg))
    (cl:cons ':battery_temperature (battery_temperature msg))
    (cl:cons ':supply_alarms (supply_alarms msg))
))
