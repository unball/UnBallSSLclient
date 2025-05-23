# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ssl_simulation_config.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import ssl_gc_common_pb2 as ssl__gc__common__pb2
from . import ssl_vision_geometry_pb2 as ssl__vision__geometry__pb2
from google.protobuf import any_pb2 as google_dot_protobuf_dot_any__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ssl_simulation_config.proto',
  package='',
  syntax='proto2',
  serialized_options=b'Z6github.com/RoboCup-SSL/ssl-simulation-protocol/pkg/sim',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1bssl_simulation_config.proto\x1a\x13ssl_gc_common.proto\x1a\x19ssl_vision_geometry.proto\x1a\x19google/protobuf/any.proto\"\xc2\x01\n\x0bRobotLimits\x12 \n\x18\x61\x63\x63_speedup_absolute_max\x18\x01 \x01(\x02\x12\x1f\n\x17\x61\x63\x63_speedup_angular_max\x18\x02 \x01(\x02\x12\x1e\n\x16\x61\x63\x63_brake_absolute_max\x18\x03 \x01(\x02\x12\x1d\n\x15\x61\x63\x63_brake_angular_max\x18\x04 \x01(\x02\x12\x18\n\x10vel_absolute_max\x18\x05 \x01(\x02\x12\x17\n\x0fvel_angular_max\x18\x06 \x01(\x02\"b\n\x10RobotWheelAngles\x12\x13\n\x0b\x66ront_right\x18\x01 \x02(\x02\x12\x12\n\nback_right\x18\x02 \x02(\x02\x12\x11\n\tback_left\x18\x03 \x02(\x02\x12\x12\n\nfront_left\x18\x04 \x02(\x02\"\xa1\x02\n\nRobotSpecs\x12\x14\n\x02id\x18\x01 \x02(\x0b\x32\x08.RobotId\x12\x14\n\x06radius\x18\x02 \x01(\x02:\x04\x30.09\x12\x14\n\x06height\x18\x03 \x01(\x02:\x04\x30.15\x12\x0c\n\x04mass\x18\x04 \x01(\x02\x12\x1d\n\x15max_linear_kick_speed\x18\x07 \x01(\x02\x12\x1b\n\x13max_chip_kick_speed\x18\x08 \x01(\x02\x12\x1a\n\x12\x63\x65nter_to_dribbler\x18\t \x01(\x02\x12\x1c\n\x06limits\x18\n \x01(\x0b\x32\x0c.RobotLimits\x12\'\n\x0cwheel_angles\x18\r \x01(\x0b\x32\x11.RobotWheelAngles\x12$\n\x06\x63ustom\x18\x0e \x01(\x0b\x32\x14.google.protobuf.Any\"5\n\rRealismConfig\x12$\n\x06\x63ustom\x18\x01 \x01(\x0b\x32\x14.google.protobuf.Any\"\x95\x01\n\x0fSimulatorConfig\x12#\n\x08geometry\x18\x01 \x01(\x0b\x32\x11.SSL_GeometryData\x12 \n\x0brobot_specs\x18\x02 \x03(\x0b\x32\x0b.RobotSpecs\x12&\n\x0erealism_config\x18\x03 \x01(\x0b\x32\x0e.RealismConfig\x12\x13\n\x0bvision_port\x18\x04 \x01(\rB8Z6github.com/RoboCup-SSL/ssl-simulation-protocol/pkg/sim'
  ,
  dependencies=[ssl__gc__common__pb2.DESCRIPTOR,ssl__vision__geometry__pb2.DESCRIPTOR,google_dot_protobuf_dot_any__pb2.DESCRIPTOR,])




_ROBOTLIMITS = _descriptor.Descriptor(
  name='RobotLimits',
  full_name='RobotLimits',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='acc_speedup_absolute_max', full_name='RobotLimits.acc_speedup_absolute_max', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='acc_speedup_angular_max', full_name='RobotLimits.acc_speedup_angular_max', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='acc_brake_absolute_max', full_name='RobotLimits.acc_brake_absolute_max', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='acc_brake_angular_max', full_name='RobotLimits.acc_brake_angular_max', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vel_absolute_max', full_name='RobotLimits.vel_absolute_max', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vel_angular_max', full_name='RobotLimits.vel_angular_max', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=107,
  serialized_end=301,
)


_ROBOTWHEELANGLES = _descriptor.Descriptor(
  name='RobotWheelAngles',
  full_name='RobotWheelAngles',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='front_right', full_name='RobotWheelAngles.front_right', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='back_right', full_name='RobotWheelAngles.back_right', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='back_left', full_name='RobotWheelAngles.back_left', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='front_left', full_name='RobotWheelAngles.front_left', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=303,
  serialized_end=401,
)


_ROBOTSPECS = _descriptor.Descriptor(
  name='RobotSpecs',
  full_name='RobotSpecs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='RobotSpecs.id', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radius', full_name='RobotSpecs.radius', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(0.09),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='RobotSpecs.height', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(0.15),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mass', full_name='RobotSpecs.mass', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_linear_kick_speed', full_name='RobotSpecs.max_linear_kick_speed', index=4,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_chip_kick_speed', full_name='RobotSpecs.max_chip_kick_speed', index=5,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center_to_dribbler', full_name='RobotSpecs.center_to_dribbler', index=6,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='limits', full_name='RobotSpecs.limits', index=7,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wheel_angles', full_name='RobotSpecs.wheel_angles', index=8,
      number=13, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom', full_name='RobotSpecs.custom', index=9,
      number=14, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=404,
  serialized_end=693,
)


_REALISMCONFIG = _descriptor.Descriptor(
  name='RealismConfig',
  full_name='RealismConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='custom', full_name='RealismConfig.custom', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=695,
  serialized_end=748,
)


_SIMULATORCONFIG = _descriptor.Descriptor(
  name='SimulatorConfig',
  full_name='SimulatorConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='geometry', full_name='SimulatorConfig.geometry', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='robot_specs', full_name='SimulatorConfig.robot_specs', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='realism_config', full_name='SimulatorConfig.realism_config', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vision_port', full_name='SimulatorConfig.vision_port', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=751,
  serialized_end=900,
)

_ROBOTSPECS.fields_by_name['id'].message_type = ssl__gc__common__pb2._ROBOTID
_ROBOTSPECS.fields_by_name['limits'].message_type = _ROBOTLIMITS
_ROBOTSPECS.fields_by_name['wheel_angles'].message_type = _ROBOTWHEELANGLES
_ROBOTSPECS.fields_by_name['custom'].message_type = google_dot_protobuf_dot_any__pb2._ANY
_REALISMCONFIG.fields_by_name['custom'].message_type = google_dot_protobuf_dot_any__pb2._ANY
_SIMULATORCONFIG.fields_by_name['geometry'].message_type = ssl__vision__geometry__pb2._SSL_GEOMETRYDATA
_SIMULATORCONFIG.fields_by_name['robot_specs'].message_type = _ROBOTSPECS
_SIMULATORCONFIG.fields_by_name['realism_config'].message_type = _REALISMCONFIG
DESCRIPTOR.message_types_by_name['RobotLimits'] = _ROBOTLIMITS
DESCRIPTOR.message_types_by_name['RobotWheelAngles'] = _ROBOTWHEELANGLES
DESCRIPTOR.message_types_by_name['RobotSpecs'] = _ROBOTSPECS
DESCRIPTOR.message_types_by_name['RealismConfig'] = _REALISMCONFIG
DESCRIPTOR.message_types_by_name['SimulatorConfig'] = _SIMULATORCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RobotLimits = _reflection.GeneratedProtocolMessageType('RobotLimits', (_message.Message,), {
  'DESCRIPTOR' : _ROBOTLIMITS,
  '__module__' : 'ssl_simulation_config_pb2'
  # @@protoc_insertion_point(class_scope:RobotLimits)
  })
_sym_db.RegisterMessage(RobotLimits)

RobotWheelAngles = _reflection.GeneratedProtocolMessageType('RobotWheelAngles', (_message.Message,), {
  'DESCRIPTOR' : _ROBOTWHEELANGLES,
  '__module__' : 'ssl_simulation_config_pb2'
  # @@protoc_insertion_point(class_scope:RobotWheelAngles)
  })
_sym_db.RegisterMessage(RobotWheelAngles)

RobotSpecs = _reflection.GeneratedProtocolMessageType('RobotSpecs', (_message.Message,), {
  'DESCRIPTOR' : _ROBOTSPECS,
  '__module__' : 'ssl_simulation_config_pb2'
  # @@protoc_insertion_point(class_scope:RobotSpecs)
  })
_sym_db.RegisterMessage(RobotSpecs)

RealismConfig = _reflection.GeneratedProtocolMessageType('RealismConfig', (_message.Message,), {
  'DESCRIPTOR' : _REALISMCONFIG,
  '__module__' : 'ssl_simulation_config_pb2'
  # @@protoc_insertion_point(class_scope:RealismConfig)
  })
_sym_db.RegisterMessage(RealismConfig)

SimulatorConfig = _reflection.GeneratedProtocolMessageType('SimulatorConfig', (_message.Message,), {
  'DESCRIPTOR' : _SIMULATORCONFIG,
  '__module__' : 'ssl_simulation_config_pb2'
  # @@protoc_insertion_point(class_scope:SimulatorConfig)
  })
_sym_db.RegisterMessage(SimulatorConfig)


DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)
