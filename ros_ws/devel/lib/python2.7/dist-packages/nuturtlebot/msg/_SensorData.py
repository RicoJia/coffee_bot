# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from nuturtlebot/SensorData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class SensorData(genpy.Message):
  _md5sum = "5e11846fa8bcf550efab217e45ea9e19"
  _type = "nuturtlebot/SensorData"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """time stamp
float32 left_encoder
float32 right_encoder
"""
  __slots__ = ['stamp','left_encoder','right_encoder']
  _slot_types = ['time','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       stamp,left_encoder,right_encoder

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SensorData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.stamp is None:
        self.stamp = genpy.Time()
      if self.left_encoder is None:
        self.left_encoder = 0.
      if self.right_encoder is None:
        self.right_encoder = 0.
    else:
      self.stamp = genpy.Time()
      self.left_encoder = 0.
      self.right_encoder = 0.

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
      buff.write(_get_struct_2I2f().pack(_x.stamp.secs, _x.stamp.nsecs, _x.left_encoder, _x.right_encoder))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.stamp.secs, _x.stamp.nsecs, _x.left_encoder, _x.right_encoder,) = _get_struct_2I2f().unpack(str[start:end])
      self.stamp.canon()
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
      buff.write(_get_struct_2I2f().pack(_x.stamp.secs, _x.stamp.nsecs, _x.left_encoder, _x.right_encoder))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.stamp.secs, _x.stamp.nsecs, _x.left_encoder, _x.right_encoder,) = _get_struct_2I2f().unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I2f = None
def _get_struct_2I2f():
    global _struct_2I2f
    if _struct_2I2f is None:
        _struct_2I2f = struct.Struct("<2I2f")
    return _struct_2I2f
