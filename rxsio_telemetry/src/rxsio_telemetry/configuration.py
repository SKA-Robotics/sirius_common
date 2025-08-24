from enum import Enum
from typing import Dict, List, Optional

from pydantic import BaseModel, Field


class MeasurementField(BaseModel):
    field: str
    value: str
    tags: Dict[str, str] = {}
    for_each: Optional[str] = None


class Measurement(BaseModel):
    name: str
    measurement_fields: List[MeasurementField] = Field(
        alias="fields", default=[])


class TopicFailMode(str, Enum):
    PerTopic = "per_topic"
    PerMeasurement = "per_measurement"
    PerField = "per_field"


class Topic(BaseModel):
    name: str
    type: Optional[str] = None
    fail_mode: TopicFailMode = TopicFailMode.PerTopic
    measurements: List[Measurement] = []


class Influx(BaseModel):
    url: str
    token: str
    bucket: str
    organization: str


class Outputs(BaseModel):
    influx: Optional[Influx] = None


class Configuration(BaseModel):
    outputs: Outputs
    topics: List[Topic] = []


def validate_configuration(**args) -> Optional[Configuration]:
    return Configuration(**args)
