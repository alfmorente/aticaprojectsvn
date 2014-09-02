; Auto-generated. Do not edit!


(cl:in-package CITIUS_Control_Communication-msg)


;//! \htmlinclude msg_irinfo.msg.html

(cl:defclass <msg_irinfo> (roslisp-msg-protocol:ros-message)
  ((currentDZoom
    :reader currentDZoom
    :initarg :currentDZoom
    :type cl:fixnum
    :initform 0)
   (currentPolarity
    :reader currentPolarity
    :initarg :currentPolarity
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_irinfo (<msg_irinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_irinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_irinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CITIUS_Control_Communication-msg:<msg_irinfo> is deprecated: use CITIUS_Control_Communication-msg:msg_irinfo instead.")))

(cl:ensure-generic-function 'currentDZoom-val :lambda-list '(m))
(cl:defmethod currentDZoom-val ((m <msg_irinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:currentDZoom-val is deprecated.  Use CITIUS_Control_Communication-msg:currentDZoom instead.")
  (currentDZoom m))

(cl:ensure-generic-function 'currentPolarity-val :lambda-list '(m))
(cl:defmethod currentPolarity-val ((m <msg_irinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CITIUS_Control_Communication-msg:currentPolarity-val is deprecated.  Use CITIUS_Control_Communication-msg:currentPolarity instead.")
  (currentPolarity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_irinfo>) ostream)
  "Serializes a message object of type '<msg_irinfo>"
  (cl:let* ((signed (cl:slot-value msg 'currentDZoom)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'currentPolarity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_irinfo>) istream)
  "Deserializes a message object of type '<msg_irinfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'currentDZoom) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'currentPolarity) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_irinfo>)))
  "Returns string type for a message object of type '<msg_irinfo>"
  "CITIUS_Control_Communication/msg_irinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_irinfo)))
  "Returns string type for a message object of type 'msg_irinfo"
  "CITIUS_Control_Communication/msg_irinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_irinfo>)))
  "Returns md5sum for a message object of type '<msg_irinfo>"
  "cd6587ca073694f8b8aa0f66b1d602b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_irinfo)))
  "Returns md5sum for a message object of type 'msg_irinfo"
  "cd6587ca073694f8b8aa0f66b1d602b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_irinfo>)))
  "Returns full string definition for message of type '<msg_irinfo>"
  (cl:format cl:nil "int8 currentDZoom~%int8 currentPolarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_irinfo)))
  "Returns full string definition for message of type 'msg_irinfo"
  (cl:format cl:nil "int8 currentDZoom~%int8 currentPolarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_irinfo>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_irinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_irinfo
    (cl:cons ':currentDZoom (currentDZoom msg))
    (cl:cons ':currentPolarity (currentPolarity msg))
))
