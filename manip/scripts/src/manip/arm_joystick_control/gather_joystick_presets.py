import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RosJointStateReceiver:

    def __init__(self, topic: str = '/manip/joint_states', node: Node = None):
        self.joint_cache = {}
        self.expected_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        if node is not None:
            node.create_subscription(JointState, topic,
                                     self._joint_callback, 10)

    def _joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_cache[name] = msg.position[i]

    def has_all_joints(self) -> bool:
        return all(j in self.joint_cache for j in self.expected_joints)

    def get_current_positions(self):
        return [self.joint_cache[j] for j in self.expected_joints]


def write_json(new_data, filename='presets.json'):
    with open(filename, 'r+') as f:
        file_data = json.load(f)
        file_data.update(new_data)
        f.seek(0)
        json.dump(file_data, f, indent=4)
        f.truncate()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gather_joystick_presets')
    receiver = RosJointStateReceiver('/manip/joint_states', node)

    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    time.sleep(1.0)
    print('Starting measuring.')

    try:
        while rclpy.ok():
            clicked = input("'w' to gather, 'q' to quit: ")
            if clicked.lower() == 'q':
                break
            if clicked.lower() == 'w':
                if not receiver.has_all_joints():
                    print('Error: not all 6 joints received.')
                    continue
                positions = receiver.get_current_positions()
                name = input('Preset name: ')
                if not name.strip():
                    print('Cancelled: name cannot be empty.')
                    continue
                write_json({'presets': {name: positions}})
                print('Saved.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
