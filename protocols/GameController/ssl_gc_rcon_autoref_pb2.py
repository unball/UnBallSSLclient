# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ssl_gc_rcon_autoref.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import ssl_gc_game_event_pb2 as ssl__gc__game__event__pb2
import ssl_gc_rcon_pb2 as ssl__gc__rcon__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ssl_gc_rcon_autoref.proto',
  package='',
  syntax='proto2',
  serialized_options=b'Z<github.com/RoboCup-SSL/ssl-game-controller/internal/app/rcon',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x19ssl_gc_rcon_autoref.proto\x1a\x17ssl_gc_game_event.proto\x1a\x11ssl_gc_rcon.proto\"H\n\x13\x41utoRefRegistration\x12\x12\n\nidentifier\x18\x01 \x02(\t\x12\x1d\n\tsignature\x18\x02 \x01(\x0b\x32\n.Signature\"`\n\x13\x41utoRefToController\x12\x1d\n\tsignature\x18\x01 \x01(\x0b\x32\n.Signature\x12\x1e\n\ngame_event\x18\x02 \x01(\x0b\x32\n.GameEventJ\x04\x08\x03\x10\x04J\x04\x08\x04\x10\x05\"J\n\x13\x43ontrollerToAutoRef\x12,\n\x10\x63ontroller_reply\x18\x01 \x01(\x0b\x32\x10.ControllerReplyH\x00\x42\x05\n\x03msgB>Z<github.com/RoboCup-SSL/ssl-game-controller/internal/app/rcon'
  ,
  dependencies=[ssl__gc__game__event__pb2.DESCRIPTOR,ssl__gc__rcon__pb2.DESCRIPTOR,])




_AUTOREFREGISTRATION = _descriptor.Descriptor(
  name='AutoRefRegistration',
  full_name='AutoRefRegistration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='identifier', full_name='AutoRefRegistration.identifier', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='signature', full_name='AutoRefRegistration.signature', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
  serialized_start=73,
  serialized_end=145,
)


_AUTOREFTOCONTROLLER = _descriptor.Descriptor(
  name='AutoRefToController',
  full_name='AutoRefToController',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='signature', full_name='AutoRefToController.signature', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='game_event', full_name='AutoRefToController.game_event', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
  serialized_start=147,
  serialized_end=243,
)


_CONTROLLERTOAUTOREF = _descriptor.Descriptor(
  name='ControllerToAutoRef',
  full_name='ControllerToAutoRef',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='controller_reply', full_name='ControllerToAutoRef.controller_reply', index=0,
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
    _descriptor.OneofDescriptor(
      name='msg', full_name='ControllerToAutoRef.msg',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=245,
  serialized_end=319,
)

_AUTOREFREGISTRATION.fields_by_name['signature'].message_type = ssl__gc__rcon__pb2._SIGNATURE
_AUTOREFTOCONTROLLER.fields_by_name['signature'].message_type = ssl__gc__rcon__pb2._SIGNATURE
_AUTOREFTOCONTROLLER.fields_by_name['game_event'].message_type = ssl__gc__game__event__pb2._GAMEEVENT
_CONTROLLERTOAUTOREF.fields_by_name['controller_reply'].message_type = ssl__gc__rcon__pb2._CONTROLLERREPLY
_CONTROLLERTOAUTOREF.oneofs_by_name['msg'].fields.append(
  _CONTROLLERTOAUTOREF.fields_by_name['controller_reply'])
_CONTROLLERTOAUTOREF.fields_by_name['controller_reply'].containing_oneof = _CONTROLLERTOAUTOREF.oneofs_by_name['msg']
DESCRIPTOR.message_types_by_name['AutoRefRegistration'] = _AUTOREFREGISTRATION
DESCRIPTOR.message_types_by_name['AutoRefToController'] = _AUTOREFTOCONTROLLER
DESCRIPTOR.message_types_by_name['ControllerToAutoRef'] = _CONTROLLERTOAUTOREF
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AutoRefRegistration = _reflection.GeneratedProtocolMessageType('AutoRefRegistration', (_message.Message,), {
  'DESCRIPTOR' : _AUTOREFREGISTRATION,
  '__module__' : 'ssl_gc_rcon_autoref_pb2'
  # @@protoc_insertion_point(class_scope:AutoRefRegistration)
  })
_sym_db.RegisterMessage(AutoRefRegistration)

AutoRefToController = _reflection.GeneratedProtocolMessageType('AutoRefToController', (_message.Message,), {
  'DESCRIPTOR' : _AUTOREFTOCONTROLLER,
  '__module__' : 'ssl_gc_rcon_autoref_pb2'
  # @@protoc_insertion_point(class_scope:AutoRefToController)
  })
_sym_db.RegisterMessage(AutoRefToController)

ControllerToAutoRef = _reflection.GeneratedProtocolMessageType('ControllerToAutoRef', (_message.Message,), {
  'DESCRIPTOR' : _CONTROLLERTOAUTOREF,
  '__module__' : 'ssl_gc_rcon_autoref_pb2'
  # @@protoc_insertion_point(class_scope:ControllerToAutoRef)
  })
_sym_db.RegisterMessage(ControllerToAutoRef)


DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)
