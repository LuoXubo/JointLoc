# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: imageInfo.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='imageInfo.proto',
  package='imagebag',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0fimageInfo.proto\x12\x08imagebag\"2\n\timageInfo\x12\x11\n\ttimestamp\x18\x01 \x01(\t\x12\x12\n\nimage_data\x18\x02 \x01(\x0c\x62\x06proto3'
)




_IMAGEINFO = _descriptor.Descriptor(
  name='imageInfo',
  full_name='imagebag.imageInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='imagebag.imageInfo.timestamp', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='image_data', full_name='imagebag.imageInfo.image_data', index=1,
      number=2, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=29,
  serialized_end=79,
)

DESCRIPTOR.message_types_by_name['imageInfo'] = _IMAGEINFO
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

imageInfo = _reflection.GeneratedProtocolMessageType('imageInfo', (_message.Message,), {
  'DESCRIPTOR' : _IMAGEINFO,
  '__module__' : 'imageInfo_pb2'
  # @@protoc_insertion_point(class_scope:imagebag.imageInfo)
  })
_sym_db.RegisterMessage(imageInfo)


# @@protoc_insertion_point(module_scope)
