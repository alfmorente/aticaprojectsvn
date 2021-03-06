"""autogenerated by genpy from Modulo_Navegacion/msg_gps.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class msg_gps(genpy.Message):
  _md5sum = "60954aed4adc07e00019828b664121f7"
  _type = "Modulo_Navegacion/msg_gps"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 latitude
float32 longitude
float32 height
float32 pitch
float32 yaw
float32 roll
float32 velocity_x
float32 velocity_y
float32 velocity_z

"""
  __slots__ = ['latitude','longitude','height','pitch','yaw','roll','velocity_x','velocity_y','velocity_z']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       latitude,longitude,height,pitch,yaw,roll,velocity_x,velocity_y,velocity_z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_gps, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.latitude is None:
        self.latitude = 0.
      if self.longitude is None:
        self.longitude = 0.
      if self.height is None:
        self.height = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.roll is None:
        self.roll = 0.
      if self.velocity_x is None:
        self.velocity_x = 0.
      if self.velocity_y is None:
        self.velocity_y = 0.
      if self.velocity_z is None:
        self.velocity_z = 0.
    else:
      self.latitude = 0.
      self.longitude = 0.
      self.height = 0.
      self.pitch = 0.
      self.yaw = 0.
      self.roll = 0.
      self.velocity_x = 0.
      self.velocity_y = 0.
      self.velocity_z = 0.

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
      buff.write(_struct_9f.pack(_x.latitude, _x.longitude, _x.height, _x.pitch, _x.yaw, _x.roll, _x.velocity_x, _x.velocity_y, _x.velocity_z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.latitude, _x.longitude, _x.height, _x.pitch, _x.yaw, _x.roll, _x.velocity_x, _x.velocity_y, _x.velocity_z,) = _struct_9f.unpack(str[start:end])
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
      buff.write(_struct_9f.pack(_x.latitude, _x.longitude, _x.height, _x.pitch, _x.yaw, _x.roll, _x.velocity_x, _x.velocity_y, _x.velocity_z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

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
      end += 36
      (_x.latitude, _x.longitude, _x.height, _x.pitch, _x.yaw, _x.roll, _x.velocity_x, _x.velocity_y, _x.velocity_z,) = _struct_9f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_9f = struct.Struct("<9f")
