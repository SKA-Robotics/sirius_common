import sys
import threading
import time
from typing import Any, Callable, Optional
import rospy
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

    def __init__(self, config: SensorConfiguration, on_die_callback: Callable):
        self._config = config
        self._on_die_callback = on_die_callback
        self._serial = None
        self._alive = True
        self._port = None

    def stop(self):
        self._alive = False

    def die(self, reason: str):
        self._alive = False
        rospy.logerr(
            f"As worker of sensor {self._config.name} I announce that I died. Reason: {reason}"
        )
        time.sleep(1)  # Explicite wait to slow down potential die loop
        self._on_die_callback(self._config)

    def run(self) -> None:
        self._publisher = rospy.Publisher(self._config.topic,
                                          Int32,
                                          queue_size=10)

        self._port = self._get_port()

        try:
            self._serial = serial.Serial(port=self._port,
                                         baudrate=self._config.baudrate,
                                         timeout=1)
        except Exception as e:
            self.die(f"{e}")

        line = ""

        try:
            while not rospy.is_shutdown() and self._alive:
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
                rospy.logwarn(
                    f"RadCon response should contain two values. Get {len(segments)} with '{line}'"
                )
                return

            values = [safe_float(segment) for segment in segments]
            if any([value is None for value in values]):
                rospy.logwarn(f"RadCon response should contain two floats")
                return

            hw_timestamp, pulse_length = values
            pulse_length = int(pulse_length)
            self._publisher.publish(pulse_length)
        else:
            if line.startswith("M/"):
                line = line.replace("M/", "").replace(";", "").strip()
                segments = line.split(" ")

                if len(segments) != 2:
                    rospy.logwarn(
                        f"Sensor response should contain two values. Get {len(segments)} with '{line}'"
                    )
                    return

                values = [safe_float(segment) for segment in segments]
                if any([value is None for value in values]):
                    rospy.logwarn(f"Sensor response should contain two floats")
                    return

                first, second = values
                second = int(second)
                self._publisher.publish(second)
            else:
                rospy.loginfo(f"GAS SENSOR: {line}")

    def _get_port(self) -> str:
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if self._config.manufacturer is not None and \
                port.manufacturer != self._config.manufacturer:
                continue

            if self._config.serial_number is not None and \
                port.serial_number != self._config.serial_number:
                continue

            break

        return port.device


class Sensors:
    _workers: dict = {}
    _configuration: Configuration = None

    def __init__(self):
        rospy.loginfo("Initializing node...")
        rospy.init_node("sensors", anonymous=True)

        rospy.loginfo("Loading configuration...")
        self.load_configuration()
        rospy.loginfo("Configuration loaded")

    def shutdown(self, reason: str) -> None:
        for worker in self._workers.items():
            worker.stop()

        rospy.logerr(reason)
        rospy.signal_shutdown(reason)
        sys.exit(0)

    def load_configuration(self) -> None:
        """
        Loads configuration
        """
        if not rospy.has_param("sensors"):
            self.shutdown("Missing rosparam 'sensors'")

        try:
            self._configuration = validate_configuration(
                sensors=rospy.get_param("sensors"))
        except ValidationError as e:
            self.shutdown(f"Configuration validation failed with error: {e}")

    def spawn_sensor(self, sensor_config: SensorConfiguration) -> None:
        rospy.loginfo(f"Spawning worker for sensor {sensor_config.name}")
        worker = SensorWorker(sensor_config, self.on_sensor_die)
        thread = threading.Thread(target=worker.run)
        thread.daemon = True
        thread.start()
        self._workers[sensor_config.name] = worker

    def on_sensor_die(self, sensor_config: SensorConfiguration) -> None:
        rospy.loginfo(
            f"Worker for sensor {sensor_config.name} died")  # @TO_DELETE
        self._workers[sensor_config.name].stop()
        self.spawn_sensor(sensor_config)

    def run(self) -> None:
        rospy.loginfo("Running...")

        for sensor_config in self._configuration.sensors:
            self.spawn_sensor(sensor_config)

        rospy.loginfo("Spinning...")
        rospy.spin()


def main():
    sensors = Sensors()
    sensors.run()


if __name__ == "__main__":
    main()
