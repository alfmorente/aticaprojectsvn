; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_tvinfo.msg.html

(cl:defclass <msg_tvinfo> (roslisp-msg-protocol:ros-message)
  ((currentZoom
    :reader currentZoom
    :initarg :currentZoom
    :type cl:fixnum
    :initform 0)
   (currentFocus
    :reader currentFocus
    :initarg :currentFocus
    :type cl:fixnum
    :initform 0)
   (autofocusMode
    :reader autofocusMode
    :initarg :autofocusMode
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_tvinfo (<msg_tvinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_tvinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_tvinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_tvinfo> is deprecated: use CITIUS_Control_Communication-msg:msg_tvinfo instead.")))

(cl:ensure-generic-function 'currentZoom-val :lambda-list '(m))
(cl:defmethod currentZoom-val ((m <msg_tvinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:currentZoom-val is deprecated.  Use CITIUS_Control_Communication-msg:currentZoom instead.")
  (currentZoom m))

(cl:ensure-generic-function 'currentFocus-val :lambda-list '(m))
(cl:defmethod currentFocus-val ((m <msg_tvinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:currentFocus-val is deprecated.  Use CITIUS_Control_Communication-msg:currentFocus instead.")
  (currentFocus m))

(cl:ensure-generic-function 'autofocusMode-val :lambda-list '(m))
(cl:defmethod autofocusMode-val ((m <msg_tvinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:autofocusMode-val is deprecated.  Use CITIUS_Control_Communication-msg:autofocusMode instead.")
  (autofocusMode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_tvinfo>) ostream)
  "Serializes a message object of type '<msg_tvinfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'currentZoom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'currentFocus)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'autofocusMode) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_tvinfo>) istream)
  "Deserializes a message object of type '<msg_tvinfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'currentZoom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'currentFocus)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'autofocusMode) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_tvinfo>)))
  "Returns string type for a message object of type '<msg_tvinfo>"
  "CITIUS_Control_Communication/msg_tvinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_tvinfo)))
  "Returns string type for a message object of type 'msg_tvinfo"
  "CITIUS_Control_Communication/msg_tvinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_tvinfo>)))
  "Returns md5sum for a message object of type '<msg_tvinfo>"
  "b1a2ff8becb96594ca8e3d19a22225e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_tvinfo)))
  "Returns md5sum for a message object of type 'msg_tvinfo"
  "b1a2ff8becb96594ca8e3d19a22225e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_tvinfo>)))
  "Returns full string definition for message of type '<msg_tvinfo>"
  (cl:format cl:nil "uint8 currentZoom~%uint8 currentFocus~%bool autofocusMode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_tvinfo)))
  "Returns full string definition for message of type 'msg_tvinfo"
  (cl:format cl:nil "uint8 currentZoom~%uint8 currentFocus~%bool autofocusMode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_tvinfo>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_tvinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_tvinfo
    (cl:cons ':currentZoom (currentZoom msg))
    (cl:cons ':currentFocus (currentFocus msg))
    (cl:cons ':autofocusMode (autofocusMode msg))
))
