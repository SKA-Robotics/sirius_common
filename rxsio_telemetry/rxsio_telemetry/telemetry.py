#!/usr/bin/env python3
import importlib
import sys
import threading
from collections import deque
from typing import Any, Dict, List, Optional, TypedDict, TypeVar, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from pydantic import ValidationError
import influxdb_client
from influxdb_client import Point
from influxdb_client.client.write_api import SYNCHRONOUS

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
                        f"Expected index got '{part}' in '{'.'.join(consumed)}'"
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
                except ValueError:
                    raise EvaluationError(
                        f"Expected index got '{part}' in '{'.'.join(consumed)}'"
                    )
                value = value[idx]
            elif isinstance(value, dict):
                value = value.get(part)
            else:
                value = getattr(value, part)
        except AttributeError as e:
            raise EvaluationError(
                f"Cannot find field '{part}' in '{'.'.join(consumed)}'") from e
        consumed.append(part)
    return value


def get_message_class(msg_type: str):
    """
    Dynamically import a ROS2 message class from a type string
    like 'std_msgs/msg/Int32' or 'std_msgs/Int32' (ROS1 style).

    ROS2 convention: 'package/msg/Type'
    ROS1 convention: 'package/Type'
    """
    parts = msg_type.split("/")
    if len(parts) == 2:
        # ROS1-style: 'std_msgs/Int32' -> 'std_msgs/msg/Int32'
        package, msg_name = parts
        module_path = f"{package}.msg"
    elif len(parts) == 3:
        # ROS2-style: 'std_msgs/msg/Int32'
        package, _, msg_name = parts
        module_path = f"{package}.msg"
    else:
        return None
    try:
        module = importlib.import_module(module_path)
        return getattr(module, msg_name)
    except (ImportError, AttributeError):
        return None


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry')
        self._subscriptions: Dict[str, Any] = {}
        self._configuration: Configuration = None
        self.record_queue = deque()
        self.record_lock = threading.Lock()

        self.get_logger().info("Loading configuration...")
        self._load_configuration()
        self.get_logger().info("Configuration loaded")

        self.influx = influxdb_client.InfluxDBClient(
            url=self._configuration.outputs.influx.url,
            token=self._configuration.outputs.influx.token,
            org=self._configuration.outputs.influx.organization,
        )
        self.influx_write = self.influx.write_api(
            write_options=SYNCHRONOUS,
            error_callback=lambda x, y, z: self.get_logger().warn(
                f"{x} {y} {z}"))
        self.get_logger().info(
            f"Connected to influx, version: {self.influx.ping()}")

    def _load_configuration(self) -> None:
        self.declare_parameter('topics', rclpy.Parameter.Type.STRING)
        self.declare_parameter('outputs', rclpy.Parameter.Type.STRING)
        self.declare_parameter('refresh_delay', 5)

        import yaml

        def _get_param(name: str):
            raw = self.get_parameter(name).value
            if isinstance(raw, str):
                return yaml.safe_load(raw)
            return raw

        try:
            self._configuration = validate_configuration(
                topics=_get_param('topics'),
                outputs=_get_param('outputs'),
            )
        except ValidationError as e:
            raise SystemExit(
                f"Configuration validation failed with error: {e}")

    def discover_topics(self) -> None:
        """
        Discovers and subscribes to available topics.
        Uses ROS2 get_topic_names_and_types() instead of rostopic.
        """
        available = dict(self.get_topic_names_and_types())
        for topic in self._configuration.topics:
            if topic.name in self._subscriptions:
                continue
            if topic.name not in available:
                continue
            type_strings = available[topic.name]
            if not type_strings:
                continue
            # Take the first type if multiple publishers exist
            topic_type = type_strings[0]
            self.get_logger().info(f"Discovered topic '{topic.name}'")
            self.register_topic(topic.name, topic_type, topic)

    def register_topic(self, name: str, message_type: str,
                       configuration: Topic) -> None:
        self.get_logger().info(
            f"Registering topic '{name}' with message type '{message_type}'..."
        )
        message_class = get_message_class(message_type)
        if message_class is None:
            self.get_logger().warn(
                f"Cannot subscribe to '{name}': unknown message type '{message_type}'"
            )
            return

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        try:
            self._subscriptions[name] = self.create_subscription(
                message_class,
                name,
                lambda msg, cfg=configuration: self.on_topic_message(msg, cfg),
                qos,
            )
            self.get_logger().info(f"Registered successfully topic '{name}'")
        except Exception as e:
            self.get_logger().warn(
                f"Cannot subscribe to '{name}' with type '{message_type}': {e}"
            )

    def evaluate_measurement_field(self,
                                   field: MeasurementField,
                                   msg: T,
                                   configuration: Topic,
                                   context: Dict[str, Any] = None
                                   ) -> WriteField:
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
                        self.evaluate_measurement_field(field, msg,
                                                        configuration))
            except EvaluationError as e:
                if configuration.fail_mode != TopicFailMode.PerField:
                    raise e
                self.get_logger().warn(
                    f"{e} in topic '{configuration.name}' "
                    f"for measurement field '{field.field}'"
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
                self.get_logger().error(
                    f"{e} in topic '{configuration.name}' "
                    f"for measurement '{measurement.name}'"
                )
        return measurements

    def on_topic_message(self, msg: T, configuration: Topic) -> None:
        try:
            measurements = self.evaluate_measurements(msg, configuration)
        except EvaluationError as e:
            self.get_logger().error(
                f"{e} in topic '{configuration.name}'")
        else:
            for measurement in measurements:
                self._enqueue_measurement(measurement)

    def _enqueue_measurement(self, measurement: WriteMeasurement) -> None:
        name = measurement.get("measurement")
        fields = measurement.get("fields")
        records = [
            Point.from_dict({
                "measurement": name,
                "tags": field.get("tags"),
                "fields": {field.get("field"): field.get("value")},
            }) for field in fields
        ]
        with self.record_lock:
            self.record_queue.extend(records)

    def influx_push(self) -> None:
        with self.record_lock:
            to_save = list(self.record_queue)
            self.record_queue.clear()
        if not to_save:
            return
        self.influx_write.write(
            bucket=self._configuration.outputs.influx.bucket,
            org=self._configuration.outputs.influx.organization,
            record=to_save,
        )
        self.influx_write.flush()

    def run(self) -> None:
        self.get_logger().info("Running...")

        refresh_delay = self.get_parameter('refresh_delay').value

        # Timer for topic discovery
        self.create_timer(float(refresh_delay),
                          lambda: self.discover_topics())

        # Timer for InfluxDB flush
        self.create_timer(0.1, lambda: self.influx_push())

        # Register topics with known type immediately
        self.get_logger().info(
            "Registering topics with known message type...")
        for topic in self._configuration.topics:
            if topic.type is not None:
                self.register_topic(topic.name, topic.type, topic)
        self.get_logger().info("Registered topics with known message type")


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    node.run()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()