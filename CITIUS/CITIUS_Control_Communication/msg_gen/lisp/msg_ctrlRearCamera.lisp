; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_ctrlRearCamera.msg.html

(cl:defclass <msg_ctrlRearCamera> (roslisp-msg-protocol:ros-message)
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

(cl:defclass msg_ctrlRearCamera (<msg_ctrlRearCamera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_ctrlRearCamera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_ctrlRearCamera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_ctrlRearCamera> is deprecated: use CITIUS_Control_Communication-msg:msg_ctrlRearCamera instead.")))

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
  "938e11f380abc0513a5b7367d0f157bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_ctrlRearCamera)))
  "Returns md5sum for a message object of type 'msg_ctrlRearCamera"
  "938e11f380abc0513a5b7367d0f157bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_ctrlRearCamera>)))
  "Returns full string definition for message of type '<msg_ctrlRearCamera>"
  (cl:format cl:nil "float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_ctrlRearCamera)))
  "Returns full string definition for message of type 'msg_ctrlRearCamera"
  (cl:format cl:nil "float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_ctrlRearCamera>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_ctrlRearCamera>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_ctrlRearCamera
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
