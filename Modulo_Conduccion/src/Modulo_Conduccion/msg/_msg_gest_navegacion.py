"""autogenerated by genpy from Modulo_Conduccion/msg_gest_navegacion.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import tf.msg

class msg_gest_navegacion(genpy.Message):
  _md5sum = "e80f53167c8cde7cbde4a732ad762d62"
  _type = "Modulo_Conduccion/msg_gest_navegacion"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """tf/tfMessage mensajeTF
nav_msgs/Odometry odometria
sensor_msgs/LaserScan laserInfo
geometry_msgs/PoseStamped posicion
================================================================================
MSG: tf/tfMessage
geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://www.ros.org/wiki/tf">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: sensor_msgs/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

"""
  __slots__ = ['mensajeTF','odometria','laserInfo','posicion']
  _slot_types = ['tf/tfMessage','nav_msgs/Odometry','sensor_msgs/LaserScan','geometry_msgs/PoseStamped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mensajeTF,odometria,laserInfo,posicion

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_gest_navegacion, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.mensajeTF is None:
        self.mensajeTF = tf.msg.tfMessage()
      if self.odometria is None:
        self.odometria = nav_msgs.msg.Odometry()
      if self.laserInfo is None:
        self.laserInfo = sensor_msgs.msg.LaserScan()
      if self.posicion is None:
        self.posicion = geometry_msgs.msg.PoseStamped()
    else:
      self.mensajeTF = tf.msg.tfMessage()
      self.odometria = nav_msgs.msg.Odometry()
      self.laserInfo = sensor_msgs.msg.LaserScan()
      self.posicion = geometry_msgs.msg.PoseStamped()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.mensajeTF.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.mensajeTF.transforms:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.child_frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.transform
        _v4 = _v3.translation
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = _v3.rotation
        _x = _v5
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_3I.pack(_x.odometria.header.seq, _x.odometria.header.stamp.secs, _x.odometria.header.stamp.nsecs))
      _x = self.odometria.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.odometria.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.odometria.pose.pose.position.x, _x.odometria.pose.pose.position.y, _x.odometria.pose.pose.position.z, _x.odometria.pose.pose.orientation.x, _x.odometria.pose.pose.orientation.y, _x.odometria.pose.pose.orientation.z, _x.odometria.pose.pose.orientation.w))
      buff.write(_struct_36d.pack(*self.odometria.pose.covariance))
      _x = self
      buff.write(_struct_6d.pack(_x.odometria.twist.twist.linear.x, _x.odometria.twist.twist.linear.y, _x.odometria.twist.twist.linear.z, _x.odometria.twist.twist.angular.x, _x.odometria.twist.twist.angular.y, _x.odometria.twist.twist.angular.z))
      buff.write(_struct_36d.pack(*self.odometria.twist.covariance))
      _x = self
      buff.write(_struct_3I.pack(_x.laserInfo.header.seq, _x.laserInfo.header.stamp.secs, _x.laserInfo.header.stamp.nsecs))
      _x = self.laserInfo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7f.pack(_x.laserInfo.angle_min, _x.laserInfo.angle_max, _x.laserInfo.angle_increment, _x.laserInfo.time_increment, _x.laserInfo.scan_time, _x.laserInfo.range_min, _x.laserInfo.range_max))
      length = len(self.laserInfo.ranges)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.laserInfo.ranges))
      length = len(self.laserInfo.intensities)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.laserInfo.intensities))
      _x = self
      buff.write(_struct_3I.pack(_x.posicion.header.seq, _x.posicion.header.stamp.secs, _x.posicion.header.stamp.nsecs))
      _x = self.posicion.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.posicion.pose.position.x, _x.posicion.pose.position.y, _x.posicion.pose.position.z, _x.posicion.pose.orientation.x, _x.posicion.pose.orientation.y, _x.posicion.pose.orientation.z, _x.posicion.pose.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.mensajeTF is None:
        self.mensajeTF = tf.msg.tfMessage()
      if self.odometria is None:
        self.odometria = nav_msgs.msg.Odometry()
      if self.laserInfo is None:
        self.laserInfo = sensor_msgs.msg.LaserScan()
      if self.posicion is None:
        self.posicion = geometry_msgs.msg.PoseStamped()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.mensajeTF.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TransformStamped()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _struct_I.unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v6.frame_id = str[start:end].decode('utf-8')
        else:
          _v6.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.child_frame_id = str[start:end].decode('utf-8')
        else:
          val1.child_frame_id = str[start:end]
        _v8 = val1.transform
        _v9 = _v8.translation
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v10 = _v8.rotation
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.mensajeTF.transforms.append(val1)
      _x = self
      start = end
      end += 12
      (_x.odometria.header.seq, _x.odometria.header.stamp.secs, _x.odometria.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.odometria.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.odometria.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.odometria.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.odometria.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.odometria.pose.pose.position.x, _x.odometria.pose.pose.position.y, _x.odometria.pose.pose.position.z, _x.odometria.pose.pose.orientation.x, _x.odometria.pose.pose.orientation.y, _x.odometria.pose.pose.orientation.z, _x.odometria.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.odometria.pose.covariance = _struct_36d.unpack(str[start:end])
      _x = self
      start = end
      end += 48
      (_x.odometria.twist.twist.linear.x, _x.odometria.twist.twist.linear.y, _x.odometria.twist.twist.linear.z, _x.odometria.twist.twist.angular.x, _x.odometria.twist.twist.angular.y, _x.odometria.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.odometria.twist.covariance = _struct_36d.unpack(str[start:end])
      _x = self
      start = end
      end += 12
      (_x.laserInfo.header.seq, _x.laserInfo.header.stamp.secs, _x.laserInfo.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.laserInfo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.laserInfo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.laserInfo.angle_min, _x.laserInfo.angle_max, _x.laserInfo.angle_increment, _x.laserInfo.time_increment, _x.laserInfo.scan_time, _x.laserInfo.range_min, _x.laserInfo.range_max,) = _struct_7f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.laserInfo.ranges = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.laserInfo.intensities = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 12
      (_x.posicion.header.seq, _x.posicion.header.stamp.secs, _x.posicion.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.posicion.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.posicion.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.posicion.pose.position.x, _x.posicion.pose.position.y, _x.posicion.pose.position.z, _x.posicion.pose.orientation.x, _x.posicion.pose.orientation.y, _x.posicion.pose.orientation.z, _x.posicion.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.mensajeTF.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.mensajeTF.transforms:
        _v11 = val1.header
        buff.write(_struct_I.pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.child_frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v13 = val1.transform
        _v14 = _v13.translation
        _x = _v14
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v15 = _v13.rotation
        _x = _v15
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_3I.pack(_x.odometria.header.seq, _x.odometria.header.stamp.secs, _x.odometria.header.stamp.nsecs))
      _x = self.odometria.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.odometria.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.odometria.pose.pose.position.x, _x.odometria.pose.pose.position.y, _x.odometria.pose.pose.position.z, _x.odometria.pose.pose.orientation.x, _x.odometria.pose.pose.orientation.y, _x.odometria.pose.pose.orientation.z, _x.odometria.pose.pose.orientation.w))
      buff.write(self.odometria.pose.covariance.tostring())
      _x = self
      buff.write(_struct_6d.pack(_x.odometria.twist.twist.linear.x, _x.odometria.twist.twist.linear.y, _x.odometria.twist.twist.linear.z, _x.odometria.twist.twist.angular.x, _x.odometria.twist.twist.angular.y, _x.odometria.twist.twist.angular.z))
      buff.write(self.odometria.twist.covariance.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.laserInfo.header.seq, _x.laserInfo.header.stamp.secs, _x.laserInfo.header.stamp.nsecs))
      _x = self.laserInfo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7f.pack(_x.laserInfo.angle_min, _x.laserInfo.angle_max, _x.laserInfo.angle_increment, _x.laserInfo.time_increment, _x.laserInfo.scan_time, _x.laserInfo.range_min, _x.laserInfo.range_max))
      length = len(self.laserInfo.ranges)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.laserInfo.ranges.tostring())
      length = len(self.laserInfo.intensities)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.laserInfo.intensities.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.posicion.header.seq, _x.posicion.header.stamp.secs, _x.posicion.header.stamp.nsecs))
      _x = self.posicion.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.posicion.pose.position.x, _x.posicion.pose.position.y, _x.posicion.pose.position.z, _x.posicion.pose.orientation.x, _x.posicion.pose.orientation.y, _x.posicion.pose.orientation.z, _x.posicion.pose.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.mensajeTF is None:
        self.mensajeTF = tf.msg.tfMessage()
      if self.odometria is None:
        self.odometria = nav_msgs.msg.Odometry()
      if self.laserInfo is None:
        self.laserInfo = sensor_msgs.msg.LaserScan()
      if self.posicion is None:
        self.posicion = geometry_msgs.msg.PoseStamped()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.mensajeTF.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TransformStamped()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _struct_I.unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v16.frame_id = str[start:end].decode('utf-8')
        else:
          _v16.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.child_frame_id = str[start:end].decode('utf-8')
        else:
          val1.child_frame_id = str[start:end]
        _v18 = val1.transform
        _v19 = _v18.translation
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v20 = _v18.rotation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.mensajeTF.transforms.append(val1)
      _x = self
      start = end
      end += 12
      (_x.odometria.header.seq, _x.odometria.header.stamp.secs, _x.odometria.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.odometria.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.odometria.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.odometria.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.odometria.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.odometria.pose.pose.position.x, _x.odometria.pose.pose.position.y, _x.odometria.pose.pose.position.z, _x.odometria.pose.pose.orientation.x, _x.odometria.pose.pose.orientation.y, _x.odometria.pose.pose.orientation.z, _x.odometria.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.odometria.pose.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 48
      (_x.odometria.twist.twist.linear.x, _x.odometria.twist.twist.linear.y, _x.odometria.twist.twist.linear.z, _x.odometria.twist.twist.angular.x, _x.odometria.twist.twist.angular.y, _x.odometria.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.odometria.twist.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 12
      (_x.laserInfo.header.seq, _x.laserInfo.header.stamp.secs, _x.laserInfo.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.laserInfo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.laserInfo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.laserInfo.angle_min, _x.laserInfo.angle_max, _x.laserInfo.angle_increment, _x.laserInfo.time_increment, _x.laserInfo.scan_time, _x.laserInfo.range_min, _x.laserInfo.range_max,) = _struct_7f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.laserInfo.ranges = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.laserInfo.intensities = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 12
      (_x.posicion.header.seq, _x.posicion.header.stamp.secs, _x.posicion.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.posicion.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.posicion.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.posicion.pose.position.x, _x.posicion.pose.position.y, _x.posicion.pose.position.z, _x.posicion.pose.orientation.x, _x.posicion.pose.orientation.y, _x.posicion.pose.orientation.z, _x.posicion.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7f = struct.Struct("<7f")
_struct_7d = struct.Struct("<7d")
_struct_6d = struct.Struct("<6d")
_struct_36d = struct.Struct("<36d")
_struct_3I = struct.Struct("<3I")
_struct_4d = struct.Struct("<4d")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")