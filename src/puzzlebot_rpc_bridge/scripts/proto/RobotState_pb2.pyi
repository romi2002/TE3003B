from google.api import annotations_pb2 as _annotations_pb2
from google.protobuf import empty_pb2 as _empty_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class ImageReply(_message.Message):
    __slots__ = ("img_b64", "width", "height")
    IMG_B64_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    img_b64: str
    width: int
    height: int
    def __init__(self, img_b64: _Optional[str] = ..., width: _Optional[int] = ..., height: _Optional[int] = ...) -> None: ...

class VelocityReply(_message.Message):
    __slots__ = ("encL", "encR")
    ENCL_FIELD_NUMBER: _ClassVar[int]
    ENCR_FIELD_NUMBER: _ClassVar[int]
    encL: float
    encR: float
    def __init__(self, encL: _Optional[float] = ..., encR: _Optional[float] = ...) -> None: ...

class LidarReply(_message.Message):
    __slots__ = ("angle_min", "angle_max", "range", "range_max")
    ANGLE_MIN_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MAX_FIELD_NUMBER: _ClassVar[int]
    RANGE_FIELD_NUMBER: _ClassVar[int]
    RANGE_MAX_FIELD_NUMBER: _ClassVar[int]
    angle_min: float
    angle_max: float
    range: _containers.RepeatedScalarFieldContainer[float]
    range_max: float
    def __init__(self, angle_min: _Optional[float] = ..., angle_max: _Optional[float] = ..., range: _Optional[_Iterable[float]] = ..., range_max: _Optional[float] = ...) -> None: ...
