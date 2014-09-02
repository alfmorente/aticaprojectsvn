; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_ctrlRearCamera.msg.html

(cl:defclass <msg_ctrlRearCamera> (roslisp-msg-protocol:ros-message)
  ((isZoom
    :reader isZoom
    :initarg :isZoom
    :type cl:boolean
    :initform cl:nil)
   (isPan
    :reader isPan
    :initarg :isPan
    :type cl:boolean
    :initform cl:nil)
   (isTilt
    :reader isTilt
    :initarg :isTilt
    :type cl:boolean
    :initform cl:nil)
   (zoom
    :reader zoom
    :initarg :zoom
    :type cl:float
    :initform 0.0)
   (pan
    :reader pan
    :initarg :pan
    :type cl:float
    :initform 0.0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:float
    :initform 0.0))
)

(cl:defclass msg_ctrlRearCamera (<msg_ctrlRearCamera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_ctrlRearCamera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_ctrlRearCamera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_ctrlRearCamera> is deprecated: use CITIUS_Control_Communication-msg:msg_ctrlRearCamera instead.")))

(cl:ensure-generic-function 'isZoom-val :lambda-list '(m))
(cl:defmethod isZoom-val ((m <msg_ctrlRearCamera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:isZoom-val is deprecated.  Use CITIUS_Control_Communication-msg:isZoom instead.")
  (isZoom m))

(cl:ensure-generic-function 'isPan-val :lambda-list '(m))
(cl:defmethod isPan-val ((m <msg_ctrlRearCamera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:isPan-val is deprecated.  Use CITIUS_Control_Communication-msg:isPan instead.")
  (isPan m))

(cl:ensure-generic-function 'isTilt-val :lambda-list '(m))
(cl:defmethod isTilt-val ((m <msg_ctrlRearCamera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:isTilt-val is deprecated.  Use CITIUS_Control_Communication-msg:isTilt instead.")
  (isTilt m))

(cl:ensure-generic-function 'zoom-val :lambda-list '(m))
(cl:defmethod zoom-val ((m <msg_ctrlRearCamera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:zoom-val is deprecated.  Use CITIUS_Control_Communication-msg:zoom instead.")
  (zoom m))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <msg_ctrlRearCamera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:pan-val is deprecated.  Use CITIUS_Control_Communication-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <msg_ctrlRearCamera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:tilt-val is deprecated.  Use CITIUS_Control_Communication-msg:tilt instead.")
  (tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_ctrlRearCamera>) ostream)
  "Serializes a message object of type '<msg_ctrlRearCamera>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isZoom) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isPan) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isTilt) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zoom))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_ctrlRearCamera>) istream)
  "Deserializes a message object of type '<msg_ctrlRearCamera>"
    (cl:setf (cl:slot-value msg 'isZoom) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isPan) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isTilt) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zoom) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pan) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_ctrlRearCamera>)))
  "Returns string type for a message object of type '<msg_ctrlRearCamera>"
  "CITIUS_Control_Communication/msg_ctrlRearCamera")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_ctrlRearCamera)))
  "Returns string type for a message object of type 'msg_ctrlRearCamera"
  "CITIUS_Control_Communication/msg_ctrlRearCamera")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_ctrlRearCamera>)))
  "Returns md5sum for a message object of type '<msg_ctrlRearCamera>"
  "ff23505ed4df1f3d3844a0151a537e6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_ctrlRearCamera)))
  "Returns md5sum for a message object of type 'msg_ctrlRearCamera"
  "ff23505ed4df1f3d3844a0151a537e6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_ctrlRearCamera>)))
  "Returns full string definition for message of type '<msg_ctrlRearCamera>"
  (cl:format cl:nil "bool isZoom~%bool isPan~%bool isTilt~%float32 zoom~%float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_ctrlRearCamera)))
  "Returns full string definition for message of type 'msg_ctrlRearCamera"
  (cl:format cl:nil "bool isZoom~%bool isPan~%bool isTilt~%float32 zoom~%float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_ctrlRearCamera>))
  (cl:+ 0
     1
     1
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_ctrlRearCamera>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_ctrlRearCamera
    (cl:cons ':isZoom (isZoom msg))
    (cl:cons ':isPan (isPan msg))
    (cl:cons ':isTilt (isTilt msg))
    (cl:cons ':zoom (zoom msg))
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
