# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from morai_msgs/FaultInjection_Response.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import morai_msgs.msg

class FaultInjection_Response(genpy.Message):
  _md5sum = "62056bf4fc5f4a1c260169ca104b9ebf"
  _type = "morai_msgs/FaultInjection_Response"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool result

int32 unique_id
FaultStatusInfo_Vehicle vehicle
FaultStatusInfo_Sensor[] sensors

================================================================================
MSG: morai_msgs/FaultStatusInfo_Vehicle
FaultStatusInfo_Overall accel
FaultStatusInfo_Overall brake
FaultStatusInfo_Overall steer
FaultStatusInfo_Overall[] tires


================================================================================
MSG: morai_msgs/FaultStatusInfo_Overall
bool status
int32[] fault_subclass

================================================================================
MSG: morai_msgs/FaultStatusInfo_Sensor
int32 sensor_id
FaultStatusInfo_Overall sensor

"""
  __slots__ = ['result','unique_id','vehicle','sensors']
  _slot_types = ['bool','int32','morai_msgs/FaultStatusInfo_Vehicle','morai_msgs/FaultStatusInfo_Sensor[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result,unique_id,vehicle,sensors

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FaultInjection_Response, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = False
      if self.unique_id is None:
        self.unique_id = 0
      if self.vehicle is None:
        self.vehicle = morai_msgs.msg.FaultStatusInfo_Vehicle()
      if self.sensors is None:
        self.sensors = []
    else:
      self.result = False
      self.unique_id = 0
      self.vehicle = morai_msgs.msg.FaultStatusInfo_Vehicle()
      self.sensors = []

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
      buff.write(_get_struct_BiB().pack(_x.result, _x.unique_id, _x.vehicle.accel.status))
      length = len(self.vehicle.accel.fault_subclass)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.vehicle.accel.fault_subclass))
      _x = self.vehicle.brake.status
      buff.write(_get_struct_B().pack(_x))
      length = len(self.vehicle.brake.fault_subclass)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.vehicle.brake.fault_subclass))
      _x = self.vehicle.steer.status
      buff.write(_get_struct_B().pack(_x))
      length = len(self.vehicle.steer.fault_subclass)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.vehicle.steer.fault_subclass))
      length = len(self.vehicle.tires)
      buff.write(_struct_I.pack(length))
      for val1 in self.vehicle.tires:
        _x = val1.status
        buff.write(_get_struct_B().pack(_x))
        length = len(val1.fault_subclass)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(struct.Struct(pattern).pack(*val1.fault_subclass))
      length = len(self.sensors)
      buff.write(_struct_I.pack(length))
      for val1 in self.sensors:
        _x = val1.sensor_id
        buff.write(_get_struct_i().pack(_x))
        _v1 = val1.sensor
        _x = _v1.status
        buff.write(_get_struct_B().pack(_x))
        length = len(_v1.fault_subclass)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(struct.Struct(pattern).pack(*_v1.fault_subclass))
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
      if self.vehicle is None:
        self.vehicle = morai_msgs.msg.FaultStatusInfo_Vehicle()
      if self.sensors is None:
        self.sensors = None
      end = 0
      _x = self
      start = end
      end += 6
      (_x.result, _x.unique_id, _x.vehicle.accel.status,) = _get_struct_BiB().unpack(str[start:end])
      self.result = bool(self.result)
      self.vehicle.accel.status = bool(self.vehicle.accel.status)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vehicle.accel.fault_subclass = s.unpack(str[start:end])
      start = end
      end += 1
      (self.vehicle.brake.status,) = _get_struct_B().unpack(str[start:end])
      self.vehicle.brake.status = bool(self.vehicle.brake.status)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vehicle.brake.fault_subclass = s.unpack(str[start:end])
      start = end
      end += 1
      (self.vehicle.steer.status,) = _get_struct_B().unpack(str[start:end])
      self.vehicle.steer.status = bool(self.vehicle.steer.status)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vehicle.steer.fault_subclass = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.vehicle.tires = []
      for i in range(0, length):
        val1 = morai_msgs.msg.FaultStatusInfo_Overall()
        start = end
        end += 1
        (val1.status,) = _get_struct_B().unpack(str[start:end])
        val1.status = bool(val1.status)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.fault_subclass = s.unpack(str[start:end])
        self.vehicle.tires.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sensors = []
      for i in range(0, length):
        val1 = morai_msgs.msg.FaultStatusInfo_Sensor()
        start = end
        end += 4
        (val1.sensor_id,) = _get_struct_i().unpack(str[start:end])
        _v2 = val1.sensor
        start = end
        end += 1
        (_v2.status,) = _get_struct_B().unpack(str[start:end])
        _v2.status = bool(_v2.status)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v2.fault_subclass = s.unpack(str[start:end])
        self.sensors.append(val1)
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
      buff.write(_get_struct_BiB().pack(_x.result, _x.unique_id, _x.vehicle.accel.status))
      length = len(self.vehicle.accel.fault_subclass)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.vehicle.accel.fault_subclass.tostring())
      _x = self.vehicle.brake.status
      buff.write(_get_struct_B().pack(_x))
      length = len(self.vehicle.brake.fault_subclass)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.vehicle.brake.fault_subclass.tostring())
      _x = self.vehicle.steer.status
      buff.write(_get_struct_B().pack(_x))
      length = len(self.vehicle.steer.fault_subclass)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.vehicle.steer.fault_subclass.tostring())
      length = len(self.vehicle.tires)
      buff.write(_struct_I.pack(length))
      for val1 in self.vehicle.tires:
        _x = val1.status
        buff.write(_get_struct_B().pack(_x))
        length = len(val1.fault_subclass)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(val1.fault_subclass.tostring())
      length = len(self.sensors)
      buff.write(_struct_I.pack(length))
      for val1 in self.sensors:
        _x = val1.sensor_id
        buff.write(_get_struct_i().pack(_x))
        _v3 = val1.sensor
        _x = _v3.status
        buff.write(_get_struct_B().pack(_x))
        length = len(_v3.fault_subclass)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(_v3.fault_subclass.tostring())
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
      if self.vehicle is None:
        self.vehicle = morai_msgs.msg.FaultStatusInfo_Vehicle()
      if self.sensors is None:
        self.sensors = None
      end = 0
      _x = self
      start = end
      end += 6
      (_x.result, _x.unique_id, _x.vehicle.accel.status,) = _get_struct_BiB().unpack(str[start:end])
      self.result = bool(self.result)
      self.vehicle.accel.status = bool(self.vehicle.accel.status)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vehicle.accel.fault_subclass = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 1
      (self.vehicle.brake.status,) = _get_struct_B().unpack(str[start:end])
      self.vehicle.brake.status = bool(self.vehicle.brake.status)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vehicle.brake.fault_subclass = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 1
      (self.vehicle.steer.status,) = _get_struct_B().unpack(str[start:end])
      self.vehicle.steer.status = bool(self.vehicle.steer.status)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vehicle.steer.fault_subclass = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.vehicle.tires = []
      for i in range(0, length):
        val1 = morai_msgs.msg.FaultStatusInfo_Overall()
        start = end
        end += 1
        (val1.status,) = _get_struct_B().unpack(str[start:end])
        val1.status = bool(val1.status)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.fault_subclass = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
        self.vehicle.tires.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sensors = []
      for i in range(0, length):
        val1 = morai_msgs.msg.FaultStatusInfo_Sensor()
        start = end
        end += 4
        (val1.sensor_id,) = _get_struct_i().unpack(str[start:end])
        _v4 = val1.sensor
        start = end
        end += 1
        (_v4.status,) = _get_struct_B().unpack(str[start:end])
        _v4.status = bool(_v4.status)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v4.fault_subclass = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
        self.sensors.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_BiB = None
def _get_struct_BiB():
    global _struct_BiB
    if _struct_BiB is None:
        _struct_BiB = struct.Struct("<BiB")
    return _struct_BiB
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i