"""autogenerated by genpy from Modulo_GPS/msg_gps.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class msg_gps(genpy.Message):
  _md5sum = "d580e52d6a982a2a25a6c4682d734553"
  _type = "Modulo_GPS/msg_gps"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 latitud
float32 longitud
float32 altitud
float32 pitch
float32 yaw
float32 roll

"""
  __slots__ = ['latitud','longitud','altitud','pitch','yaw','roll']
  _slot_types = ['float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       latitud,longitud,altitud,pitch,yaw,roll

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_gps, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.latitud is None:
        self.latitud = 0.
      if self.longitud is None:
        self.longitud = 0.
      if self.altitud is None:
        self.altitud = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.roll is None:
        self.roll = 0.
    else:
      self.latitud = 0.
      self.longitud = 0.
      self.altitud = 0.
      self.pitch = 0.
      self.yaw = 0.
      self.roll = 0.

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
      buff.write(_struct_6f.pack(_x.latitud, _x.longitud, _x.altitud, _x.pitch, _x.yaw, _x.roll))
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
      end += 24
      (_x.latitud, _x.longitud, _x.altitud, _x.pitch, _x.yaw, _x.roll,) = _struct_6f.unpack(str[start:end])
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
      buff.write(_struct_6f.pack(_x.latitud, _x.longitud, _x.altitud, _x.pitch, _x.yaw, _x.roll))
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
      end += 24
      (_x.latitud, _x.longitud, _x.altitud, _x.pitch, _x.yaw, _x.roll,) = _struct_6f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6f = struct.Struct("<6f")