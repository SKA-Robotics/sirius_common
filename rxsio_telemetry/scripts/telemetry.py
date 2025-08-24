#!/usr/bin/env python

import datetime
import sys
from typing import Any, Dict, List, TypedDict, TypeVar, Union

import roslib.message
import rospy
import rostopic
from pydantic import ValidationError
import influxdb_client
from influxdb_client import Point
from influxdb_client.client.write_api import SYNCHRONOUS
from collections import deque
import threading

from rxsio_telemetry.configuration import (
    Configuration,
    Measurement,
    MeasurementField,
    Topic,
    TopicFailMode,
    validate_configuration,
)

T = TypeVar("T")


class WriteField(TypedDict):
    field: str
    value: Union[str, int, float, bool]
    tags: Dict[str, str]


class WriteMeasurement(TypedDict):
    measurement: str
    fields: List[WriteField]


class EvaluationError(Exception):

    def __init__(self, message: str):
        super().__init__(message)


def evaluate_message_expression(msg: T,
                                expression: str,
                                context: Dict[str, Any] = None) -> Any:
    context = {} if context is None else context
    context["$msg"] = msg

    if expression.startswith("$"):
        reference, *parts = expression.split(".")

        if reference not in context:
            raise EvaluationError(f"Unknown reference to {reference}")

        value = context.get(reference)
        consumed = [reference]
    else:
        return expression

    for part in parts:
        try:
            if isinstance(value, (list, tuple)):
                if not part.startswith("[") or not part.endswith("]"):
                    raise EvaluationError(
                        "Excepted index got '{part}' in '{'.'.join(consumed)}'"
                    )

                try:
                    identifier = part.replace("[", "").replace("]", "")

                    if identifier.startswith("$"):
                        if identifier not in context:
                            raise EvaluationError(
                                f"Unknown reference to {identifier} in '{'.'.join(consumed)}'"
                            )

                        idx = context.get(identifier)
                    else:
                        idx = int(identifier)
                except ValueError as e:
                    raise EvaluationError(
                        f"Excepted index got '{part}' in '{'.'.join(consumed)}'"
                    )

                value = value[idx]
            elif isinstance(value, dict):
                value = value.get(part)
            else:
                value = getattr(value, part)
        except AttributeError as e:
            print(e)
            raise EvaluationError(
                f"Cannot find field '{part}' in '{'.'.join(consumed)}'")

        consumed.append(part)

    return value


class Telemetry:
    _topics: Dict[str, rospy.Subscriber] = {}
    _configuration: Configuration = None

    def __init__(self):
        rospy.loginfo("Initializing node...")
        rospy.init_node("listener", anonymous=True)

        rospy.loginfo("Loading configuration...")
        self.load_configuration()
        rospy.loginfo("Configuration loaded")

        self.influx = influxdb_client.InfluxDBClient(
            url=self._configuration.outputs.influx.url,
            token=self._configuration.outputs.influx.token,
            org=self._configuration.outputs.influx.organization)
        self.influx_write = self.influx.write_api(
            write_options=SYNCHRONOUS,
            error_callback=lambda x, y, z: rospy.logwarn(f"{x} {y} {z}"))

        rospy.loginfo(f"Connected to influx, version: {self.influx.ping()}")

        self.record_queue = deque()
        self.record_lock = threading.Lock()

    def shutdown(self, reason: str) -> None:
        rospy.logerr(reason)
        rospy.signal_shutdown(reason)
        sys.exit(0)

    def load_configuration(self) -> None:
        """
        Loads configuration
        """
        if not rospy.has_param("topics"):
            self.shutdown("Missing rosparam 'topics'")

        if not rospy.has_param("outputs"):
            self.shutdown("Missing rosparam 'outputs'")

        try:
            self._configuration = validate_configuration(
                topics=rospy.get_param("topics"),
                outputs=rospy.get_param("outputs"))
        except ValidationError as e:
            self.shutdown(f"Configuration validation failed with error: {e}")

    def discover_topics(self, _event) -> None:
        """
        Discovers and subscribes to available topics
        """
        for topic in self._configuration.topics:
            if topic.name in self._topics.keys():
                continue

            topic_type = rostopic.get_topic_type(topic.name)[0]

            if topic_type is None:
                continue

            rospy.loginfo(f"Discovered topic '{topic.name}'")
            self.register_topic(topic.name, topic_type, topic)

    def register_topic(self, name: str, message_type: str,
                       configuration: Topic) -> None:
        """
        Registers the topic for subscription
        """
        rospy.loginfo(
            f"Registering topic '{name}' with message type '{message_type}'..."
        )

        message_class = roslib.message.get_message_class(message_type)

        if message_class is None:
            rospy.logwarn(
                f"Cannot subscribe topic '{name}', because cannot get message class for type '{message_type}'"
            )
            return

        try:
            self._topics[name] = rospy.Subscriber(
                name,
                message_class,
                self.on_topic_message,
                callback_args={"configuration": configuration})
            rospy.loginfo(f"Registered successfully topic '{name}'")
        except (ValueError, rospy.ROSException) as e:
            rospy.logwarn(
                f"Cannot subscribe topic '{name}' with message type '{message_type}'"
            )
            rospy.logdebug(f"Details: {e}")

    def evaluate_measurement_field(
            self,
            field: MeasurementField,
            msg: T,
            configuration: Topic,
            context: Dict[str, Any] = None) -> WriteField:
        value = evaluate_message_expression(msg, field.value, context)
        tags = {}

        for tag, tag_expression in field.tags.items():
            tag_value = evaluate_message_expression(msg, tag_expression,
                                                    context)
            tags[tag] = tag_value

        return {"field": field.field, "value": value, "tags": tags}

    def evaluate_measurement(self, measurement: Measurement, msg: T,
                             configuration: Topic) -> List[WriteField]:
        fields = []

        for field in measurement.measurement_fields:
            try:
                if field.for_each is not None:
                    for idx, each in enumerate(
                            evaluate_message_expression(msg, field.for_each)):
                        fields.append(
                            self.evaluate_measurement_field(
                                field, msg, configuration, {
                                    "$id": idx,
                                    "$each": each
                                }))
                else:
                    fields.append(
                        self.evaluate_measurement_field(
                            field, msg, configuration))
            except EvaluationError as e:
                if configuration.fail_mode != TopicFailMode.PerField:
                    raise e

                rospy.logwarn(
                    f"{e} in topic '{configuration.name}' for measurement field '{field.field}'"
                )

        return fields

    def evaluate_measurements(self, msg: T,
                              configuration: Topic) -> List[WriteMeasurement]:
        measurements = []

        for measurement in configuration.measurements:
            try:
                measurements.append({
                    "measurement":
                    measurement.name,
                    "fields":
                    self.evaluate_measurement(measurement, msg, configuration)
                })
            except EvaluationError as e:
                if configuration.fail_mode == TopicFailMode.PerTopic:
                    raise e

                rospy.logerr(
                    f"{e} in topic '{configuration.name}' for measurement '{measurement.name}'"
                )

        return measurements

    def on_topic_message(self, msg: T, args) -> None:
        """
        Topic callback
        """
        configuration: Topic = args.get("configuration")

        if configuration is None:
            self.shutdown("Invalid topic callback handler")

        try:
            measurements = self.evaluate_measurements(msg, configuration)
        except EvaluationError as e:
            rospy.logerr(f"{e} in topic '{configuration.name}'")
        else:
            for measurement in measurements:
                self.write_measurement(measurement)

    def write_measurement(self, measurement: WriteMeasurement):
        name = measurement.get("measurement")
        fields = measurement.get("fields")

        records = []

        records = [
            Point.from_dict({
                "measurement": name,
                "tags": field.get("tags"),
                "fields": {
                    field.get("field"): field.get("value")
                }
            }) for field in fields
        ]

        with self.record_lock:
            self.record_queue.extend(records)
        """
        self.influx_write.write(
            bucket=self._configuration.outputs.influx.bucket,
            org=self._configuration.outputs.influx.organization,
            record=records)
        """
        # self.influx_write.flush()

    def influx_push(self):
        with self.record_lock:
            to_save = list(self.record_queue)
            self.record_queue.clear()

        self.influx_write.write(
            bucket=self._configuration.outputs.influx.bucket,
            org=self._configuration.outputs.influx.organization,
            record=to_save)

        self.influx_write.flush()

    def run(self) -> None:
        rospy.loginfo("Running...")

        refresh_delay = rospy.get_param("refresh_delay", 10)
        rospy.Timer(rospy.Duration(refresh_delay), self.discover_topics)
        rospy.Timer(rospy.Duration(0.1), lambda _event: self.influx_push())

        rospy.loginfo("Registering topics with known message type...")
        for topic in self._configuration.topics:
            if topic.type is not None:
                self.register_topic(topic.name, topic.type, topic)
        rospy.loginfo("Registered topics with known message type")

        rospy.spin()


def main():
    telemetry = Telemetry()
    telemetry.run()


if __name__ == "__main__":
    main()
