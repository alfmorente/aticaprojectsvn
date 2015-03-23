; Auto-generated. Do not edit!


(cl:in-package Modulo_Navegacion-msg)


;//! \htmlinclude msg_gest_navegacion.msg.html

(cl:defclass <msg_gest_navegacion> (roslisp-msg-protocol:ros-message)
  ((mensajeTF
    :reader mensajeTF
    :initarg :mensajeTF
    :type tf-msg:tfMessage
    :initform (cl:make-instance 'tf-msg:tfMessage))
   (odometria
    :reader odometria
    :initarg :odometria
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (laserInfo
    :reader laserInfo
    :initarg :laserInfo
    :type sensor_msgs-msg:LaserScan
    :initform (cl:make-instance 'sensor_msgs-msg:LaserScan))
   (posicion
    :reader posicion
    :initarg :posicion
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass msg_gest_navegacion (<msg_gest_navegacion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_gest_navegacion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_gest_navegacion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Navegacion-msg:<msg_gest_navegacion> is deprecated: use Modulo_Navegacion-msg:msg_gest_navegacion instead.")))

(cl:ensure-generic-function 'mensajeTF-val :lambda-list '(m))
(cl:defmethod mensajeTF-val ((m <msg_gest_navegacion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:mensajeTF-val is deprecated.  Use Modulo_Navegacion-msg:mensajeTF instead.")
  (mensajeTF m))

(cl:ensure-generic-function 'odometria-val :lambda-list '(m))
(cl:defmethod odometria-val ((m <msg_gest_navegacion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:odometria-val is deprecated.  Use Modulo_Navegacion-msg:odometria instead.")
  (odometria m))

(cl:ensure-generic-function 'laserInfo-val :lambda-list '(m))
(cl:defmethod laserInfo-val ((m <msg_gest_navegacion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:laserInfo-val is deprecated.  Use Modulo_Navegacion-msg:laserInfo instead.")
  (laserInfo m))

(cl:ensure-generic-function 'posicion-val :lambda-list '(m))
(cl:defmethod posicion-val ((m <msg_gest_navegacion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Navegacion-msg:posicion-val is deprecated.  Use Modulo_Navegacion-msg:posicion instead.")
  (posicion m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_gest_navegacion>) ostream)
  "Serializes a message object of type '<msg_gest_navegacion>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mensajeTF) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'odometria) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laserInfo) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posicion) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_gest_navegacion>) istream)
  "Deserializes a message object of type '<msg_gest_navegacion>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mensajeTF) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'odometria) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laserInfo) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posicion) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_gest_navegacion>)))
  "Returns string type for a message object of type '<msg_gest_navegacion>"
  "Modulo_Navegacion/msg_gest_navegacion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_gest_navegacion)))
  "Returns string type for a message object of type 'msg_gest_navegacion"
  "Modulo_Navegacion/msg_gest_navegacion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_gest_navegacion>)))
  "Returns md5sum for a message object of type '<msg_gest_navegacion>"
  "e80f53167c8cde7cbde4a732ad762d62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_gest_navegacion)))
  "Returns md5sum for a message object of type 'msg_gest_navegacion"
  "e80f53167c8cde7cbde4a732ad762d62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_gest_navegacion>)))
  "Returns full string definition for message of type '<msg_gest_navegacion>"
  (cl:format cl:nil "tf/tfMessage mensajeTF~%nav_msgs/Odometry odometria~%sensor_msgs/LaserScan laserInfo~%geometry_msgs/PoseStamped posicion~%================================================================================~%MSG: tf/tfMessage~%geometry_msgs/TransformStamped[] transforms~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/LaserScan~%# Single scan from a planar laser range-finder~%#~%# If you have another ranging device with different behavior (e.g. a sonar~%# array), please find or create a different message, since applications~%# will make fairly laser-specific assumptions about this data~%~%Header header            # timestamp in the header is the acquisition time of ~%                         # the first ray in the scan.~%                         #~%                         # in frame frame_id, angles are measured around ~%                         # the positive Z axis (counterclockwise, if Z is up)~%                         # with zero angle being forward along the x axis~%                         ~%float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%float32 angle_increment  # angular distance between measurements [rad]~%~%float32 time_increment   # time between measurements [seconds] - if your scanner~%                         # is moving, this will be used in interpolating position~%                         # of 3d points~%float32 scan_time        # time between scans [seconds]~%~%float32 range_min        # minimum range value [m]~%float32 range_max        # maximum range value [m]~%~%float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)~%float32[] intensities    # intensity data [device-specific units].  If your~%                         # device does not provide intensities, please leave~%                         # the array empty.~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_gest_navegacion)))
  "Returns full string definition for message of type 'msg_gest_navegacion"
  (cl:format cl:nil "tf/tfMessage mensajeTF~%nav_msgs/Odometry odometria~%sensor_msgs/LaserScan laserInfo~%geometry_msgs/PoseStamped posicion~%================================================================================~%MSG: tf/tfMessage~%geometry_msgs/TransformStamped[] transforms~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/LaserScan~%# Single scan from a planar laser range-finder~%#~%# If you have another ranging device with different behavior (e.g. a sonar~%# array), please find or create a different message, since applications~%# will make fairly laser-specific assumptions about this data~%~%Header header            # timestamp in the header is the acquisition time of ~%                         # the first ray in the scan.~%                         #~%                         # in frame frame_id, angles are measured around ~%                         # the positive Z axis (counterclockwise, if Z is up)~%                         # with zero angle being forward along the x axis~%                         ~%float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%float32 angle_increment  # angular distance between measurements [rad]~%~%float32 time_increment   # time between measurements [seconds] - if your scanner~%                         # is moving, this will be used in interpolating position~%                         # of 3d points~%float32 scan_time        # time between scans [seconds]~%~%float32 range_min        # minimum range value [m]~%float32 range_max        # maximum range value [m]~%~%float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)~%float32[] intensities    # intensity data [device-specific units].  If your~%                         # device does not provide intensities, please leave~%                         # the array empty.~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_gest_navegacion>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mensajeTF))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'odometria))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laserInfo))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posicion))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_gest_navegacion>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_gest_navegacion
    (cl:cons ':mensajeTF (mensajeTF msg))
    (cl:cons ':odometria (odometria msg))
    (cl:cons ':laserInfo (laserInfo msg))
    (cl:cons ':posicion (posicion msg))
))
