"""autogenerated by genpy from CITIUS_Control_Communication/msg_ctrlRearCamera.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class msg_ctrlRearCamera(genpy.Message):
  _md5sum = "ff23505ed4df1f3d3844a0151a537e6b"
  _type = "CITIUS_Control_Communication/msg_ctrlRearCamera"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool isZoom
bool isPan
bool isTilt
float32 zoom
float32 pan
float32 tilt
"""
  __slots__ = ['isZoom','isPan','isTilt','zoom','pan','tilt']
  _slot_types = ['bool','bool','bool','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       isZoom,isPan,isTilt,zoom,pan,tilt

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_ctrlRearCamera, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.isZoom is None:
        self.isZoom = False
      if self.isPan is None:
        self.isPan = False
      if self.isTilt is None:
        self.isTilt = False
      if self.zoom is None:
        self.zoom = 0.
      if self.pan is None:
        self.pan = 0.
      if self.tilt is None:
        self.tilt = 0.
    else:
      self.isZoom = False
      self.isPan = False
      self.isTilt = False
      self.zoom = 0.
      self.pan = 0.
      self.tilt = 0.

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
      buff.write(_struct_3B3f.pack(_x.isZoom, _x.isPan, _x.isTilt, _x.zoom, _x.pan, _x.tilt))
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
      end += 15
      (_x.isZoom, _x.isPan, _x.isTilt, _x.zoom, _x.pan, _x.tilt,) = _struct_3B3f.unpack(str[start:end])
      self.isZoom = bool(self.isZoom)
      self.isPan = bool(self.isPan)
      self.isTilt = bool(self.isTilt)
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
      buff.write(_struct_3B3f.pack(_x.isZoom, _x.isPan, _x.isTilt, _x.zoom, _x.pan, _x.tilt))
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
      end += 15
      (_x.isZoom, _x.isPan, _x.isTilt, _x.zoom, _x.pan, _x.tilt,) = _struct_3B3f.unpack(str[start:end])
      self.isZoom = bool(self.isZoom)
      self.isPan = bool(self.isPan)
      self.isTilt = bool(self.isTilt)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3B3f = struct.Struct("<3B3f")
