"""autogenerated by genpy from CITIUS_Control_Driving/msg_vehicleInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class msg_vehicleInfo(genpy.Message):
  _md5sum = "f5ad468e30e0eec9c9f9d0323c8e4eca"
  _type = "CITIUS_Control_Driving/msg_vehicleInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int16 steering
int16 thottle
int16 brake
bool parkingBrake
uint8 gear
uint16 speed
int16 motorRPM
int16 motorTemperature
bool lights
bool blinkerLeft
bool blinkerRight
bool dipss
bool dipsr
bool dipsp
bool klaxon
"""
  __slots__ = ['steering','thottle','brake','parkingBrake','gear','speed','motorRPM','motorTemperature','lights','blinkerLeft','blinkerRight','dipss','dipsr','dipsp','klaxon']
  _slot_types = ['int16','int16','int16','bool','uint8','uint16','int16','int16','bool','bool','bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       steering,thottle,brake,parkingBrake,gear,speed,motorRPM,motorTemperature,lights,blinkerLeft,blinkerRight,dipss,dipsr,dipsp,klaxon

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(msg_vehicleInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.steering is None:
        self.steering = 0
      if self.thottle is None:
        self.thottle = 0
      if self.brake is None:
        self.brake = 0
      if self.parkingBrake is None:
        self.parkingBrake = False
      if self.gear is None:
        self.gear = 0
      if self.speed is None:
        self.speed = 0
      if self.motorRPM is None:
        self.motorRPM = 0
      if self.motorTemperature is None:
        self.motorTemperature = 0
      if self.lights is None:
        self.lights = False
      if self.blinkerLeft is None:
        self.blinkerLeft = False
      if self.blinkerRight is None:
        self.blinkerRight = False
      if self.dipss is None:
        self.dipss = False
      if self.dipsr is None:
        self.dipsr = False
      if self.dipsp is None:
        self.dipsp = False
      if self.klaxon is None:
        self.klaxon = False
    else:
      self.steering = 0
      self.thottle = 0
      self.brake = 0
      self.parkingBrake = False
      self.gear = 0
      self.speed = 0
      self.motorRPM = 0
      self.motorTemperature = 0
      self.lights = False
      self.blinkerLeft = False
      self.blinkerRight = False
      self.dipss = False
      self.dipsr = False
      self.dipsp = False
      self.klaxon = False

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
      buff.write(_struct_3h2BH2h7B.pack(_x.steering, _x.thottle, _x.brake, _x.parkingBrake, _x.gear, _x.speed, _x.motorRPM, _x.motorTemperature, _x.lights, _x.blinkerLeft, _x.blinkerRight, _x.dipss, _x.dipsr, _x.dipsp, _x.klaxon))
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
      end += 21
      (_x.steering, _x.thottle, _x.brake, _x.parkingBrake, _x.gear, _x.speed, _x.motorRPM, _x.motorTemperature, _x.lights, _x.blinkerLeft, _x.blinkerRight, _x.dipss, _x.dipsr, _x.dipsp, _x.klaxon,) = _struct_3h2BH2h7B.unpack(str[start:end])
      self.parkingBrake = bool(self.parkingBrake)
      self.lights = bool(self.lights)
      self.blinkerLeft = bool(self.blinkerLeft)
      self.blinkerRight = bool(self.blinkerRight)
      self.dipss = bool(self.dipss)
      self.dipsr = bool(self.dipsr)
      self.dipsp = bool(self.dipsp)
      self.klaxon = bool(self.klaxon)
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
      buff.write(_struct_3h2BH2h7B.pack(_x.steering, _x.thottle, _x.brake, _x.parkingBrake, _x.gear, _x.speed, _x.motorRPM, _x.motorTemperature, _x.lights, _x.blinkerLeft, _x.blinkerRight, _x.dipss, _x.dipsr, _x.dipsp, _x.klaxon))
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
      end += 21
      (_x.steering, _x.thottle, _x.brake, _x.parkingBrake, _x.gear, _x.speed, _x.motorRPM, _x.motorTemperature, _x.lights, _x.blinkerLeft, _x.blinkerRight, _x.dipss, _x.dipsr, _x.dipsp, _x.klaxon,) = _struct_3h2BH2h7B.unpack(str[start:end])
      self.parkingBrake = bool(self.parkingBrake)
      self.lights = bool(self.lights)
      self.blinkerLeft = bool(self.blinkerLeft)
      self.blinkerRight = bool(self.blinkerRight)
      self.dipss = bool(self.dipss)
      self.dipsr = bool(self.dipsr)
      self.dipsp = bool(self.dipsp)
      self.klaxon = bool(self.klaxon)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3h2BH2h7B = struct.Struct("<3h2BH2h7B")
