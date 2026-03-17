from joystick_control.msg import Gamepad


class JoystickTranslator:
    def __init__(self, node):
        # node is passed in so we can read parameters
        joystick_type = node.get_parameter('joystick_type').value or 'STANDARD'
        joystick_data = node.get_parameter(joystick_type).value
        self.AXES_ID: dict = joystick_data['axes']
        self.BUTTONS_ID: dict = joystick_data['buttons']

    def translate(self, data: Gamepad):
        inputs = {name: data.buttons[id]
                  for name, id in self.BUTTONS_ID.items()}
        inputs.update({name: data.axes[id]
                       for name, id in self.AXES_ID.items()})
        return inputs