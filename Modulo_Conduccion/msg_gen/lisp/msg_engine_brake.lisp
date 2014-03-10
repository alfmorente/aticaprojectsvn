; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude msg_engine_brake.msg.html

(cl:defclass <msg_engine_brake> (roslisp-msg-protocol:ros-message)
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

(cl:defclass msg_engine_brake (<msg_engine_brake>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_engine_brake>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_engine_brake)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<msg_engine_brake> is deprecated: use Modulo_Conduccion-msg:msg_engine_brake instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <msg_engine_brake>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:command-val is deprecated.  Use Modulo_Conduccion-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_engine_brake>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:value-val is deprecated.  Use Modulo_Conduccion-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_engine_brake>) ostream)
  "Serializes a message object of type '<msg_engine_brake>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'command) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_engine_brake>) istream)
  "Deserializes a message object of type '<msg_engine_brake>"
    (cl:setf (cl:slot-value msg 'command) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_engine_brake>)))
  "Returns string type for a message object of type '<msg_engine_brake>"
  "Modulo_Conduccion/msg_engine_brake")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_engine_brake)))
  "Returns string type for a message object of type 'msg_engine_brake"
  "Modulo_Conduccion/msg_engine_brake")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_engine_brake>)))
  "Returns md5sum for a message object of type '<msg_engine_brake>"
  "afabe79133bf3b438c176b9a4d0a4902")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_engine_brake)))
  "Returns md5sum for a message object of type 'msg_engine_brake"
  "afabe79133bf3b438c176b9a4d0a4902")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_engine_brake>)))
  "Returns full string definition for message of type '<msg_engine_brake>"
  (cl:format cl:nil "bool command~%bool value      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_engine_brake)))
  "Returns full string definition for message of type 'msg_engine_brake"
  (cl:format cl:nil "bool command~%bool value      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_engine_brake>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_engine_brake>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_engine_brake
    (cl:cons ':command (command msg))
    (cl:cons ':value (value msg))
))
