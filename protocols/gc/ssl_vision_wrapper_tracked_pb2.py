# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ssl_vision_wrapper_tracked.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import protocols.gc.ssl_vision_detection_tracked_pb2 as ssl__vision__detection__tracked__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ssl_vision_wrapper_tracked.proto',
  package='',
  syntax='proto2',
  serialized_options=_b('Z?github.com/RoboCup-SSL/ssl-game-controller/internal/app/tracker'),
  serialized_pb=_b('\n ssl_vision_wrapper_tracked.proto\x1a\"ssl_vision_detection_tracked.proto\"_\n\x14TrackerWrapperPacket\x12\x0c\n\x04uuid\x18\x01 \x02(\t\x12\x13\n\x0bsource_name\x18\x02 \x01(\t\x12$\n\rtracked_frame\x18\x03 \x01(\x0b\x32\r.TrackedFrameBAZ?github.com/RoboCup-SSL/ssl-game-controller/internal/app/tracker')
  ,
  dependencies=[ssl__vision__detection__tracked__pb2.DESCRIPTOR,])




_TRACKERWRAPPERPACKET = _descriptor.Descriptor(
  name='TrackerWrapperPacket',
  full_name='TrackerWrapperPacket',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uuid', full_name='TrackerWrapperPacket.uuid', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='source_name', full_name='TrackerWrapperPacket.source_name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tracked_frame', full_name='TrackerWrapperPacket.tracked_frame', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=72,
  serialized_end=167,
)

_TRACKERWRAPPERPACKET.fields_by_name['tracked_frame'].message_type = ssl__vision__detection__tracked__pb2._TRACKEDFRAME
DESCRIPTOR.message_types_by_name['TrackerWrapperPacket'] = _TRACKERWRAPPERPACKET
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrackerWrapperPacket = _reflection.GeneratedProtocolMessageType('TrackerWrapperPacket', (_message.Message,), dict(
  DESCRIPTOR = _TRACKERWRAPPERPACKET,
  __module__ = 'ssl_vision_wrapper_tracked_pb2'
  # @@protoc_insertion_point(class_scope:TrackerWrapperPacket)
  ))
_sym_db.RegisterMessage(TrackerWrapperPacket)


DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)