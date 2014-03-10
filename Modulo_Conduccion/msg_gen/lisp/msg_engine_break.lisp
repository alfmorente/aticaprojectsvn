; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude msg_engine_break.msg.html

(cl:defclass <msg_engine_break> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:boolean
    :initform cl:nil)
   (value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass msg_engine_break (<msg_engine_break>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_engine_break>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_engine_break)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<msg_engine_break> is deprecated: use Modulo_Conduccion-msg:msg_engine_break instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <msg_engine_break>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:command-val is deprecated.  Use Modulo_Conduccion-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_engine_break>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:value-val is deprecated.  Use Modulo_Conduccion-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_engine_break>) ostream)
  "Serializes a message object of type '<msg_engine_break>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'command) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_engine_break>) istream)
  "Deserializes a message object of type '<msg_engine_break>"
    (cl:setf (cl:slot-value msg 'command) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_engine_break>)))
  "Returns string type for a message object of type '<msg_engine_break>"
  "Modulo_Conduccion/msg_engine_break")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_engine_break)))
  "Returns string type for a message object of type 'msg_engine_break"
  "Modulo_Conduccion/msg_engine_break")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_engine_break>)))
  "Returns md5sum for a message object of type '<msg_engine_break>"
  "afabe79133bf3b438c176b9a4d0a4902")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_engine_break)))
  "Returns md5sum for a message object of type 'msg_engine_break"
  "afabe79133bf3b438c176b9a4d0a4902")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_engine_break>)))
  "Returns full string definition for message of type '<msg_engine_break>"
  (cl:format cl:nil "bool command~%bool value      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_engine_break)))
  "Returns full string definition for message of type 'msg_engine_break"
  (cl:format cl:nil "bool command~%bool value      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_engine_break>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_engine_break>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_engine_break
    (cl:cons ':command (command msg))
    (cl:cons ':value (value msg))
))
