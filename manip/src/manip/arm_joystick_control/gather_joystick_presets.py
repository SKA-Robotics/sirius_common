import json
import rospy
from sensor_msgs.msg import JointState


class RosJointStateReceiver:
    def __init__(self, topic: str = "/joint_states"):
        self.current_positions = []
        rospy.Subscriber(topic, JointState, self.joint_callback, queue_size=10)

    def joint_callback(self, msg: JointState):
        self.current_positions = list(msg.position)

    def get_current_positions(self):
        return self.current_positions


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

    while not rospy.is_shutdown():
        clicked = input(
            "Click 'w' to gather joint states or click 'q' to quit.\n")
        if clicked.lower() == 'q':
            break
        if clicked.lower() == 'w':
            joint_states = jointReceiver.get_current_positions()
            name = input("Gathered joint states. Write name of the preset.")

            if not name.strip():
                print("Cancelled: Preset name cannot be empty.")
                continue

            data = {name: joint_states}
            write_json(data)
            print("New data written to file.")


if __name__ == "__main__":
    main()
