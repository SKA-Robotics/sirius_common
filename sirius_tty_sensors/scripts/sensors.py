#!/usr/bin/env python3
import sys
import threading
import time
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from std_msgs.msg import Int32
from pydantic import ValidationError

from sirius_tty_sensors.configuration import (SensorConfiguration, SensorType,
                                               Configuration,
                                               validate_configuration)


def safe_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (ValueError, TypeError):
        return None


class SensorWorker:
    def __init__(self, config: SensorConfiguration, node: Node,
                 on_die_callback: Callable):
        self._config = config
        self._node = node
        self._on_die_callback = on_die_callback
        self._serial = None
        self._alive = True
        self._port = None

    def stop(self):
        self._alive = False

    def die(self, reason: str):
        self._alive = False
        self._node.get_logger().error(
            f"As worker of sensor {self._config.name} I announce that I died. Reason: {reason}"
        )
        time.sleep(1)  # Explicit wait to slow down potential die loop
        self._on_die_callback(self._config)

    def run(self) -> None:
        self._publisher = self._node.create_publisher(Int32,
                                                       self._config.topic,
                                                       10)
        self._port = self._get_port()
        try:
            self._serial = serial.Serial(port=self._port,
                                         baudrate=self._config.baudrate,
                                         timeout=1)
        except Exception as e:
            self.die(f"{e}")
            return

        line = ""
        try:
            while rclpy.ok() and self._alive:
                byte = self._serial.read(1)
                if byte != b'':
                    line += byte.decode("ascii")
                else:
                    continue
                if line.endswith("\r\n"):
                    self.process_line(line)
                    line = ""
        except Exception as e:
            self.die(f"{e}")

    def process_line(self, line: str) -> None:
        if self._config.stype == SensorType.RadCon:
            segments = line.split(" ")
            segments = [segment.strip() for segment in segments]
            if len(segments) != 2:
                self._node.get_logger().warn(
                    f"RadCon response should contain two values. Got {len(segments)} with '{line}'"
                )
                return
            values = [safe_float(segment) for segment in segments]
            if any(value is None for value in values):
                self._node.get_logger().warn(
                    "RadCon response should contain two floats")
                return
            _hw_timestamp, pulse_length = values
            self._publisher.publish(Int32(data=int(pulse_length)))
        else:
            if line.startswith("M/"):
                line = line.replace("M/", "").replace(";", "").strip()
                segments = line.split(" ")
                if len(segments) != 2:
                    self._node.get_logger().warn(
                        f"Sensor response should contain two values. Got {len(segments)} with '{line}'"
                    )
                    return
                values = [safe_float(segment) for segment in segments]
                if any(value is None for value in values):
                    self._node.get_logger().warn(
                        "Sensor response should contain two floats")
                    return
                _first, second = values
                self._publisher.publish(Int32(data=int(second)))
            else:
                self._node.get_logger().info(f"GAS SENSOR: {line}")

    def _get_port(self) -> str:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if self._config.manufacturer is not None and \
                    port.manufacturer != self._config.manufacturer:
                continue
            if self._config.serial_number is not None and \
                    port.serial_number != self._config.serial_number:
                continue
            return port.device
        raise RuntimeError(
            f"No serial port found for sensor '{self._config.name}' "
            f"(manufacturer={self._config.manufacturer}, "
            f"serial_number={self._config.serial_number})"
        )


class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors')
        self._workers: dict = {}
        self._configuration: Configuration = None

        self.get_logger().info("Loading configuration...")
        self._load_configuration()
        self.get_logger().info("Configuration loaded")

    def _load_configuration(self) -> None:
        self.declare_parameter('sensors', rclpy.Parameter.Type.STRING)

        if not self.has_parameter('sensors'):
            self._shutdown("Missing parameter 'sensors'")
            return

        raw = self.get_parameter('sensors').value
        try:
            # Parameter comes in as a YAML string or dict depending on how it's loaded
            import yaml
            if isinstance(raw, str):
                data = yaml.safe_load(raw)
            else:
                data = raw
            self._configuration = validate_configuration(sensors=data)
        except ValidationError as e:
            self._shutdown(f"Configuration validation failed: {e}")

    def _shutdown(self, reason: str) -> None:
        for worker in self._workers.values():
            worker.stop()
        self.get_logger().error(reason)
        raise SystemExit(reason)

    def spawn_sensor(self, sensor_config: SensorConfiguration) -> None:
        self.get_logger().info(
            f"Spawning worker for sensor {sensor_config.name}")
        worker = SensorWorker(sensor_config, self, self.on_sensor_die)
        thread = threading.Thread(target=worker.run, daemon=True)
        thread.start()
        self._workers[sensor_config.name] = worker

    def on_sensor_die(self, sensor_config: SensorConfiguration) -> None:
        self.get_logger().info(
            f"Worker for sensor {sensor_config.name} died, restarting...")
        if sensor_config.name in self._workers:
            self._workers[sensor_config.name].stop()
        self.spawn_sensor(sensor_config)

    def run(self) -> None:
        self.get_logger().info("Running...")
        for sensor_config in self._configuration.sensors:
            self.spawn_sensor(sensor_config)
        self.get_logger().info("Spinning...")


def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()
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