; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Driving-msg)


;//! \htmlinclude msg_vehicleInfo.msg.html

(cl:defclass <msg_vehicleInfo> (roslisp-msg-protocol:ros-message)
  ((steering
    :reader steering
    :initarg :steering
    :type cl:fixnum
    :initform 0)
   (thottle
    :reader thottle
    :initarg :thottle
    :type cl:fixnum
    :initform 0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:fixnum
    :initform 0)
   (parkingBrake
    :reader parkingBrake
    :initarg :parkingBrake
    :type cl:boolean
    :initform cl:nil)
   (gear
    :reader gear
    :initarg :gear
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0)
   (motorRPM
    :reader motorRPM
    :initarg :motorRPM
    :type cl:fixnum
    :initform 0)
   (motorTemperature
    :reader motorTemperature
    :initarg :motorTemperature
    :type cl:fixnum
    :initform 0)
   (lights
    :reader lights
    :initarg :lights
    :type cl:boolean
    :initform cl:nil)
   (blinkerLeft
    :reader blinkerLeft
    :initarg :blinkerLeft
    :type cl:boolean
    :initform cl:nil)
   (blinkerRight
    :reader blinkerRight
    :initarg :blinkerRight
    :type cl:boolean
    :initform cl:nil)
   (dipss
    :reader dipss
    :initarg :dipss
    :type cl:boolean
    :initform cl:nil)
   (dipsr
    :reader dipsr
    :initarg :dipsr
    :type cl:boolean
    :initform cl:nil)
   (dipsp
    :reader dipsp
    :initarg :dipsp
    :type cl:boolean
    :initform cl:nil)
   (klaxon
    :reader klaxon
    :initarg :klaxon
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_vehicleInfo (<msg_vehicleInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_vehicleInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_vehicleInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Driving-msg:<msg_vehicleInfo> is deprecated: use CITIUS_Control_Driving-msg:msg_vehicleInfo instead.")))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:steering-val is deprecated.  Use CITIUS_Control_Driving-msg:steering instead.")
  (steering m))

(cl:ensure-generic-function 'thottle-val :lambda-list '(m))
(cl:defmethod thottle-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:thottle-val is deprecated.  Use CITIUS_Control_Driving-msg:thottle instead.")
  (thottle m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:brake-val is deprecated.  Use CITIUS_Control_Driving-msg:brake instead.")
  (brake m))

(cl:ensure-generic-function 'parkingBrake-val :lambda-list '(m))
(cl:defmethod parkingBrake-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:parkingBrake-val is deprecated.  Use CITIUS_Control_Driving-msg:parkingBrake instead.")
  (parkingBrake m))

(cl:ensure-generic-function 'gear-val :lambda-list '(m))
(cl:defmethod gear-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:gear-val is deprecated.  Use CITIUS_Control_Driving-msg:gear instead.")
  (gear m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:speed-val is deprecated.  Use CITIUS_Control_Driving-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'motorRPM-val :lambda-list '(m))
(cl:defmethod motorRPM-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:motorRPM-val is deprecated.  Use CITIUS_Control_Driving-msg:motorRPM instead.")
  (motorRPM m))

(cl:ensure-generic-function 'motorTemperature-val :lambda-list '(m))
(cl:defmethod motorTemperature-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:motorTemperature-val is deprecated.  Use CITIUS_Control_Driving-msg:motorTemperature instead.")
  (motorTemperature m))

(cl:ensure-generic-function 'lights-val :lambda-list '(m))
(cl:defmethod lights-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:lights-val is deprecated.  Use CITIUS_Control_Driving-msg:lights instead.")
  (lights m))

(cl:ensure-generic-function 'blinkerLeft-val :lambda-list '(m))
(cl:defmethod blinkerLeft-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:blinkerLeft-val is deprecated.  Use CITIUS_Control_Driving-msg:blinkerLeft instead.")
  (blinkerLeft m))

(cl:ensure-generic-function 'blinkerRight-val :lambda-list '(m))
(cl:defmethod blinkerRight-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:blinkerRight-val is deprecated.  Use CITIUS_Control_Driving-msg:blinkerRight instead.")
  (blinkerRight m))

(cl:ensure-generic-function 'dipss-val :lambda-list '(m))
(cl:defmethod dipss-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:dipss-val is deprecated.  Use CITIUS_Control_Driving-msg:dipss instead.")
  (dipss m))

(cl:ensure-generic-function 'dipsr-val :lambda-list '(m))
(cl:defmethod dipsr-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:dipsr-val is deprecated.  Use CITIUS_Control_Driving-msg:dipsr instead.")
  (dipsr m))

(cl:ensure-generic-function 'dipsp-val :lambda-list '(m))
(cl:defmethod dipsp-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:dipsp-val is deprecated.  Use CITIUS_Control_Driving-msg:dipsp instead.")
  (dipsp m))

(cl:ensure-generic-function 'klaxon-val :lambda-list '(m))
(cl:defmethod klaxon-val ((m <msg_vehicleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Driving-msg:klaxon-val is deprecated.  Use CITIUS_Control_Driving-msg:klaxon instead.")
  (klaxon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_vehicleInfo>) ostream)
  "Serializes a message object of type '<msg_vehicleInfo>"
  (cl:let* ((signed (cl:slot-value msg 'steering)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'thottle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'brake)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'parkingBrake) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'motorRPM)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motorTemperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lights) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'blinkerLeft) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'blinkerRight) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dipss) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dipsr) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dipsp) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'klaxon) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_vehicleInfo>) istream)
  "Deserializes a message object of type '<msg_vehicleInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thottle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'brake) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'parkingBrake) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motorRPM) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motorTemperature) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'lights) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'blinkerLeft) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'blinkerRight) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dipss) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dipsr) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dipsp) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'klaxon) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_vehicleInfo>)))
  "Returns string type for a message object of type '<msg_vehicleInfo>"
  "CITIUS_Control_Driving/msg_vehicleInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_vehicleInfo)))
  "Returns string type for a message object of type 'msg_vehicleInfo"
  "CITIUS_Control_Driving/msg_vehicleInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_vehicleInfo>)))
  "Returns md5sum for a message object of type '<msg_vehicleInfo>"
  "f5ad468e30e0eec9c9f9d0323c8e4eca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_vehicleInfo)))
  "Returns md5sum for a message object of type 'msg_vehicleInfo"
  "f5ad468e30e0eec9c9f9d0323c8e4eca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_vehicleInfo>)))
  "Returns full string definition for message of type '<msg_vehicleInfo>"
  (cl:format cl:nil "int16 steering~%int16 thottle~%int16 brake~%bool parkingBrake~%uint8 gear~%uint16 speed~%int16 motorRPM~%int16 motorTemperature~%bool lights~%bool blinkerLeft~%bool blinkerRight~%bool dipss~%bool dipsr~%bool dipsp~%bool klaxon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_vehicleInfo)))
  "Returns full string definition for message of type 'msg_vehicleInfo"
  (cl:format cl:nil "int16 steering~%int16 thottle~%int16 brake~%bool parkingBrake~%uint8 gear~%uint16 speed~%int16 motorRPM~%int16 motorTemperature~%bool lights~%bool blinkerLeft~%bool blinkerRight~%bool dipss~%bool dipsr~%bool dipsp~%bool klaxon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_vehicleInfo>))
  (cl:+ 0
     2
     2
     2
     1
     1
     2
     2
     2
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_vehicleInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_vehicleInfo
    (cl:cons ':steering (steering msg))
    (cl:cons ':thottle (thottle msg))
    (cl:cons ':brake (brake msg))
    (cl:cons ':parkingBrake (parkingBrake msg))
    (cl:cons ':gear (gear msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':motorRPM (motorRPM msg))
    (cl:cons ':motorTemperature (motorTemperature msg))
    (cl:cons ':lights (lights msg))
    (cl:cons ':blinkerLeft (blinkerLeft msg))
    (cl:cons ':blinkerRight (blinkerRight msg))
    (cl:cons ':dipss (dipss msg))
    (cl:cons ':dipsr (dipsr msg))
    (cl:cons ':dipsp (dipsp msg))
    (cl:cons ':klaxon (klaxon msg))
))
