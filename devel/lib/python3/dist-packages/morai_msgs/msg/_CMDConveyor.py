# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from morai_msgs/CMDConveyor.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CMDConveyor(genpy.Message):
  _md5sum = "dff78b314b2d7216c66dfddb645260b4"
  _type = "morai_msgs/CMDConveyor"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool position_sensor
bool palette_sensor1
bool palette_sensor2
int32 limit_sensor"""
  __slots__ = ['position_sensor','palette_sensor1','palette_sensor2','limit_sensor']
  _slot_types = ['bool','bool','bool','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       position_sensor,palette_sensor1,palette_sensor2,limit_sensor

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CMDConveyor, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.position_sensor is None:
        self.position_sensor = False
      if self.palette_sensor1 is None:
        self.palette_sensor1 = False
      if self.palette_sensor2 is None:
        self.palette_sensor2 = False
      if self.limit_sensor is None:
        self.limit_sensor = 0
    else:
      self.position_sensor = False
      self.palette_sensor1 = False
      self.palette_sensor2 = False
      self.limit_sensor = 0

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
      buff.write(_get_struct_3Bi().pack(_x.position_sensor, _x.palette_sensor1, _x.palette_sensor2, _x.limit_sensor))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 7
      (_x.position_sensor, _x.palette_sensor1, _x.palette_sensor2, _x.limit_sensor,) = _get_struct_3Bi().unpack(str[start:end])
      self.position_sensor = bool(self.position_sensor)
      self.palette_sensor1 = bool(self.palette_sensor1)
      self.palette_sensor2 = bool(self.palette_sensor2)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3Bi().pack(_x.position_sensor, _x.palette_sensor1, _x.palette_sensor2, _x.limit_sensor))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 7
      (_x.position_sensor, _x.palette_sensor1, _x.palette_sensor2, _x.limit_sensor,) = _get_struct_3Bi().unpack(str[start:end])
      self.position_sensor = bool(self.position_sensor)
      self.palette_sensor1 = bool(self.palette_sensor1)
      self.palette_sensor2 = bool(self.palette_sensor2)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3Bi = None
def _get_struct_3Bi():
    global _struct_3Bi
    if _struct_3Bi is None:
        _struct_3Bi = struct.Struct("<3Bi")
    return _struct_3Bi