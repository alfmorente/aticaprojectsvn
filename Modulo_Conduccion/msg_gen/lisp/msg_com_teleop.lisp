; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude msg_com_teleop.msg.html

(cl:defclass <msg_com_teleop> (roslisp-msg-protocol:ros-message)
  ((id_element
    :reader id_element
    :initarg :id_element
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_com_teleop (<msg_com_teleop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_com_teleop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_com_teleop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<msg_com_teleop> is deprecated: use Modulo_Conduccion-msg:msg_com_teleop instead.")))

(cl:ensure-generic-function 'id_element-val :lambda-list '(m))
(cl:defmethod id_element-val ((m <msg_com_teleop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:id_element-val is deprecated.  Use Modulo_Conduccion-msg:id_element instead.")
  (id_element m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_com_teleop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:value-val is deprecated.  Use Modulo_Conduccion-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_com_teleop>) ostream)
  "Serializes a message object of type '<msg_com_teleop>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_element)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_com_teleop>) istream)
  "Deserializes a message object of type '<msg_com_teleop>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_element)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_com_teleop>)))
  "Returns string type for a message object of type '<msg_com_teleop>"
  "Modulo_Conduccion/msg_com_teleop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_com_teleop)))
  "Returns string type for a message object of type 'msg_com_teleop"
  "Modulo_Conduccion/msg_com_teleop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_com_teleop>)))
  "Returns md5sum for a message object of type '<msg_com_teleop>"
  "8e68cf4eaf8cc2bf2ebbab2ba4940029")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_com_teleop)))
  "Returns md5sum for a message object of type 'msg_com_teleop"
  "8e68cf4eaf8cc2bf2ebbab2ba4940029")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_com_teleop>)))
  "Returns full string definition for message of type '<msg_com_teleop>"
  (cl:format cl:nil "uint8 id_element~%int16 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_com_teleop)))
  "Returns full string definition for message of type 'msg_com_teleop"
  (cl:format cl:nil "uint8 id_element~%int16 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_com_teleop>))
  (cl:+ 0
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_com_teleop>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_com_teleop
    (cl:cons ':id_element (id_element msg))
    (cl:cons ':value (value msg))
))
