from enum import Enum
from typing import List, Optional

from pydantic import BaseModel


class SensorType(str, Enum):
    RadCon = "radcon"
    Gas = "gas"


class SensorConfiguration(BaseModel):
    name: str
    baudrate: int
    topic: str
    stype: SensorType
    manufacturer: Optional[str] = None
    serial_number: Optional[str] = None


class Configuration(BaseModel):
    sensors: List[SensorConfiguration]


def validate_configuration(**args) -> Optional[Configuration]:
    return Configuration(**args)
