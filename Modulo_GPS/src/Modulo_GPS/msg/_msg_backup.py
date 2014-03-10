"""autogenerated by genpy from Modulo_GPS/msg_backup.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class msg_backup(genpy.Message):
  _md5sum = "ff8688787aa97766b5311a5e39e25a80"
  _type = "Modulo_GPS/msg_backup"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 throttle
uint8 brake
int8 steer
bool handbrake
uint8 gear
bool engine
uint8 speed
"""
  __slots__ = ['throttle','brake','steer','handbrake','gear','engine','speed']
  _slot_types = ['uint8','uint8','int8','bool','uint8','bool','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       throttle,brake,steer,handbrake,gear,engine,speed

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_backup, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.throttle is None:
        self.throttle = 0
      if self.brake is None:
        self.brake = 0
      if self.steer is None:
        self.steer = 0
      if self.handbrake is None:
        self.handbrake = False
      if self.gear is None:
        self.gear = 0
      if self.engine is None:
        self.engine = False
      if self.speed is None:
        self.speed = 0
    else:
      self.throttle = 0
      self.brake = 0
      self.steer = 0
      self.handbrake = False
      self.gear = 0
      self.engine = False
      self.speed = 0

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
      buff.write(_struct_2Bb4B.pack(_x.throttle, _x.brake, _x.steer, _x.handbrake, _x.gear, _x.engine, _x.speed))
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
      end += 7
      (_x.throttle, _x.brake, _x.steer, _x.handbrake, _x.gear, _x.engine, _x.speed,) = _struct_2Bb4B.unpack(str[start:end])
      self.handbrake = bool(self.handbrake)
      self.engine = bool(self.engine)
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
      buff.write(_struct_2Bb4B.pack(_x.throttle, _x.brake, _x.steer, _x.handbrake, _x.gear, _x.engine, _x.speed))
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
      end += 7
      (_x.throttle, _x.brake, _x.steer, _x.handbrake, _x.gear, _x.engine, _x.speed,) = _struct_2Bb4B.unpack(str[start:end])
      self.handbrake = bool(self.handbrake)
      self.engine = bool(self.engine)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2Bb4B = struct.Struct("<2Bb4B")
