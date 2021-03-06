"""autogenerated by genpy from CITIUS_Control_PositionOrientation/msg_posOriInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class msg_posOriInfo(genpy.Message):
  _md5sum = "18dcfbcc11c0eee5c95670f31271590f"
  _type = "CITIUS_Control_PositionOrientation/msg_posOriInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint16 positionStatus
uint16 orientationStatus
float64 latitude
float64 longitude
float32 altitude
float32 roll
float32 pitch
float32 yaw
float32 velX
float32 velY
float32 velZ
float32 accX
float32 accY
float32 accZ
float32 rateX
float32 rateY
float32 rateZ
"""
  __slots__ = ['positionStatus','orientationStatus','latitude','longitude','altitude','roll','pitch','yaw','velX','velY','velZ','accX','accY','accZ','rateX','rateY','rateZ']
  _slot_types = ['uint16','uint16','float64','float64','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       positionStatus,orientationStatus,latitude,longitude,altitude,roll,pitch,yaw,velX,velY,velZ,accX,accY,accZ,rateX,rateY,rateZ

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_posOriInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.positionStatus is None:
        self.positionStatus = 0
      if self.orientationStatus is None:
        self.orientationStatus = 0
      if self.latitude is None:
        self.latitude = 0.
      if self.longitude is None:
        self.longitude = 0.
      if self.altitude is None:
        self.altitude = 0.
      if self.roll is None:
        self.roll = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.velX is None:
        self.velX = 0.
      if self.velY is None:
        self.velY = 0.
      if self.velZ is None:
        self.velZ = 0.
      if self.accX is None:
        self.accX = 0.
      if self.accY is None:
        self.accY = 0.
      if self.accZ is None:
        self.accZ = 0.
      if self.rateX is None:
        self.rateX = 0.
      if self.rateY is None:
        self.rateY = 0.
      if self.rateZ is None:
        self.rateZ = 0.
    else:
      self.positionStatus = 0
      self.orientationStatus = 0
      self.latitude = 0.
      self.longitude = 0.
      self.altitude = 0.
      self.roll = 0.
      self.pitch = 0.
      self.yaw = 0.
      self.velX = 0.
      self.velY = 0.
      self.velZ = 0.
      self.accX = 0.
      self.accY = 0.
      self.accZ = 0.
      self.rateX = 0.
      self.rateY = 0.
      self.rateZ = 0.

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
      _x = self
      buff.write(_struct_2H2d13f.pack(_x.positionStatus, _x.orientationStatus, _x.latitude, _x.longitude, _x.altitude, _x.roll, _x.pitch, _x.yaw, _x.velX, _x.velY, _x.velZ, _x.accX, _x.accY, _x.accZ, _x.rateX, _x.rateY, _x.rateZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 72
      (_x.positionStatus, _x.orientationStatus, _x.latitude, _x.longitude, _x.altitude, _x.roll, _x.pitch, _x.yaw, _x.velX, _x.velY, _x.velZ, _x.accX, _x.accY, _x.accZ, _x.rateX, _x.rateY, _x.rateZ,) = _struct_2H2d13f.unpack(str[start:end])
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
      _x = self
      buff.write(_struct_2H2d13f.pack(_x.positionStatus, _x.orientationStatus, _x.latitude, _x.longitude, _x.altitude, _x.roll, _x.pitch, _x.yaw, _x.velX, _x.velY, _x.velZ, _x.accX, _x.accY, _x.accZ, _x.rateX, _x.rateY, _x.rateZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 72
      (_x.positionStatus, _x.orientationStatus, _x.latitude, _x.longitude, _x.altitude, _x.roll, _x.pitch, _x.yaw, _x.velX, _x.velY, _x.velZ, _x.accX, _x.accY, _x.accZ, _x.rateX, _x.rateY, _x.rateZ,) = _struct_2H2d13f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2H2d13f = struct.Struct("<2H2d13f")
