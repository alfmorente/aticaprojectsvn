; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_rearCameraInfo.msg.html

(cl:defclass <msg_rearCameraInfo> (roslisp-msg-protocol:ros-message)
  ((pan
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

(cl:defclass msg_rearCameraInfo (<msg_rearCameraInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_rearCameraInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_rearCameraInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_rearCameraInfo> is deprecated: use CITIUS_Control_Communication-msg:msg_rearCameraInfo instead.")))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <msg_rearCameraInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:pan-val is deprecated.  Use CITIUS_Control_Communication-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <msg_rearCameraInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:tilt-val is deprecated.  Use CITIUS_Control_Communication-msg:tilt instead.")
  (tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_rearCameraInfo>) ostream)
  "Serializes a message object of type '<msg_rearCameraInfo>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_rearCameraInfo>) istream)
  "Deserializes a message object of type '<msg_rearCameraInfo>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_rearCameraInfo>)))
  "Returns string type for a message object of type '<msg_rearCameraInfo>"
  "CITIUS_Control_Communication/msg_rearCameraInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_rearCameraInfo)))
  "Returns string type for a message object of type 'msg_rearCameraInfo"
  "CITIUS_Control_Communication/msg_rearCameraInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_rearCameraInfo>)))
  "Returns md5sum for a message object of type '<msg_rearCameraInfo>"
  "938e11f380abc0513a5b7367d0f157bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_rearCameraInfo)))
  "Returns md5sum for a message object of type 'msg_rearCameraInfo"
  "938e11f380abc0513a5b7367d0f157bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_rearCameraInfo>)))
  "Returns full string definition for message of type '<msg_rearCameraInfo>"
  (cl:format cl:nil "float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_rearCameraInfo)))
  "Returns full string definition for message of type 'msg_rearCameraInfo"
  (cl:format cl:nil "float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_rearCameraInfo>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_rearCameraInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_rearCameraInfo
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
