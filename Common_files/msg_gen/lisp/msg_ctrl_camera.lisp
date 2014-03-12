; Auto-generated. Do not edit!


(cl:in-package Common_files-msg)


;//! \htmlinclude msg_ctrl_camera.msg.html

(cl:defclass <msg_ctrl_camera> (roslisp-msg-protocol:ros-message)
  ((id_control
    :reader id_control
    :initarg :id_control
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msg_ctrl_camera (<msg_ctrl_camera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_ctrl_camera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_ctrl_camera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Common_files-msg:<msg_ctrl_camera> is deprecated: use Common_files-msg:msg_ctrl_camera instead.")))

(cl:ensure-generic-function 'id_control-val :lambda-list '(m))
(cl:defmethod id_control-val ((m <msg_ctrl_camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:id_control-val is deprecated.  Use Common_files-msg:id_control instead.")
  (id_control m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <msg_ctrl_camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Common_files-msg:value-val is deprecated.  Use Common_files-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_ctrl_camera>) ostream)
  "Serializes a message object of type '<msg_ctrl_camera>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_control)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_ctrl_camera>) istream)
  "Deserializes a message object of type '<msg_ctrl_camera>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id_control)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_ctrl_camera>)))
  "Returns string type for a message object of type '<msg_ctrl_camera>"
  "Common_files/msg_ctrl_camera")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_ctrl_camera)))
  "Returns string type for a message object of type 'msg_ctrl_camera"
  "Common_files/msg_ctrl_camera")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_ctrl_camera>)))
  "Returns md5sum for a message object of type '<msg_ctrl_camera>"
  "c8c2f9b76345acfe8099761c31308cd4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_ctrl_camera)))
  "Returns md5sum for a message object of type 'msg_ctrl_camera"
  "c8c2f9b76345acfe8099761c31308cd4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_ctrl_camera>)))
  "Returns full string definition for message of type '<msg_ctrl_camera>"
  (cl:format cl:nil "uint8 id_control~%uint8 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_ctrl_camera)))
  "Returns full string definition for message of type 'msg_ctrl_camera"
  (cl:format cl:nil "uint8 id_control~%uint8 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_ctrl_camera>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_ctrl_camera>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_ctrl_camera
    (cl:cons ':id_control (id_control msg))
    (cl:cons ':value (value msg))
))
