; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_panTiltPosition.msg.html

(cl:defclass <msg_panTiltPosition> (roslisp-msg-protocol:ros-message)
  ((panPosition
    :reader panPosition
    :initarg :panPosition
    :type cl:fixnum
    :initform 0)
   (tiltPosition
    :reader tiltPosition
    :initarg :tiltPosition
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_panTiltPosition (<msg_panTiltPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_panTiltPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_panTiltPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_panTiltPosition> is deprecated: use CITIUS_Control_Communication-msg:msg_panTiltPosition instead.")))

(cl:ensure-generic-function 'panPosition-val :lambda-list '(m))
(cl:defmethod panPosition-val ((m <msg_panTiltPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:panPosition-val is deprecated.  Use CITIUS_Control_Communication-msg:panPosition instead.")
  (panPosition m))

(cl:ensure-generic-function 'tiltPosition-val :lambda-list '(m))
(cl:defmethod tiltPosition-val ((m <msg_panTiltPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:tiltPosition-val is deprecated.  Use CITIUS_Control_Communication-msg:tiltPosition instead.")
  (tiltPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_panTiltPosition>) ostream)
  "Serializes a message object of type '<msg_panTiltPosition>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'panPosition)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'panPosition)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tiltPosition)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_panTiltPosition>) istream)
  "Deserializes a message object of type '<msg_panTiltPosition>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'panPosition)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'panPosition)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tiltPosition) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_panTiltPosition>)))
  "Returns string type for a message object of type '<msg_panTiltPosition>"
  "CITIUS_Control_Communication/msg_panTiltPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_panTiltPosition)))
  "Returns string type for a message object of type 'msg_panTiltPosition"
  "CITIUS_Control_Communication/msg_panTiltPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_panTiltPosition>)))
  "Returns md5sum for a message object of type '<msg_panTiltPosition>"
  "4d3b018255f829633b9f2dc37f6c1da8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_panTiltPosition)))
  "Returns md5sum for a message object of type 'msg_panTiltPosition"
  "4d3b018255f829633b9f2dc37f6c1da8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_panTiltPosition>)))
  "Returns full string definition for message of type '<msg_panTiltPosition>"
  (cl:format cl:nil "uint16 panPosition~%int16 tiltPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_panTiltPosition)))
  "Returns full string definition for message of type 'msg_panTiltPosition"
  (cl:format cl:nil "uint16 panPosition~%int16 tiltPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_panTiltPosition>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_panTiltPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_panTiltPosition
    (cl:cons ':panPosition (panPosition msg))
    (cl:cons ':tiltPosition (tiltPosition msg))
))
