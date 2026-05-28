import json
import rospy
from sensor_msgs.msg import JointState


class RosJointStateReceiver:

    def __init__(self, topic: str = "/manip/joint_states"):
        self.joint_cache = {}
        self.expected_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        rospy.Subscriber(topic, JointState, self.joint_callback, queue_size=10)

    def joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_cache[name] = msg.position[i]

    def has_all_joints(self):
        return all(joint in self.joint_cache for joint in self.expected_joints)

    def get_current_positions(self):
        return [self.joint_cache[joint] for joint in self.expected_joints]


def write_json(new_data, filename='presets.json'):
    with open(filename, 'r+') as file:
        # Load existing data into a dictionary
        file_data = json.load(file)

        file_data.update(new_data)

        # Move the cursor to the beginning of the file
        file.seek(0)

        # Write the updated data back to the file
        json.dump(file_data, file, indent=4)

        file.truncate()


def main():
    rospy.init_node('listener', anonymous=True)
    jointReceiver = RosJointStateReceiver()
    print("Starting measuring.\n")

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        clicked = input(
            "Click 'w' to gather joint states or click 'q' to quit.\n")
        if clicked.lower() == 'q':
            break
        if clicked.lower() == 'w':

            if not jointReceiver.has_all_joints():
                print("Error: there aren't all 6 joints!")
                continue

            joint_states = jointReceiver.get_current_positions()
            name = input("Gathered joint states. Write name of the preset. \n")

            if not name.strip():
                print("Cancelled: Preset name cannot be empty. \n")
                continue

            data = {"presets": {name: joint_states}}
            write_json(data)
            print("New data written to file. \n")


if __name__ == "__main__":
    main()
