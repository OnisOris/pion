# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: datagram.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""

from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(
    b'\n\x0e\x64\x61tagram.proto\x12\x04pion"\x87\x01\n\x08\x44\x61tagram\x12\r\n\x05token\x18\x01 \x01(\x05\x12\n\n\x02id\x18\x02 \x01(\x03\x12\x0e\n\x06source\x18\x03 \x01(\x05\x12\x0f\n\x07\x63ommand\x18\x04 \x01(\x05\x12\x0c\n\x04\x64\x61ta\x18\x05 \x03(\x01\x12\x11\n\ttarget_id\x18\x06 \x01(\t\x12\x10\n\x08group_id\x18\x07 \x01(\x05\x12\x0c\n\x04hash\x18\x08 \x01(\tb\x06proto3'
)

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, "datagram_pb2", _globals)
if not _descriptor._USE_C_DESCRIPTORS:
    DESCRIPTOR._loaded_options = None
    _globals["_DATAGRAM"]._serialized_start = 25
    _globals["_DATAGRAM"]._serialized_end = 160
# @@protoc_insertion_point(module_scope)