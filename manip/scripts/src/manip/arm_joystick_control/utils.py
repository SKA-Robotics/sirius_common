from joystick_control.msg import Gamepad

def max_abs(a, b):
    if abs(a) > abs(b):
        return a
    else:
        return b

def trig_to_axis(trig):
    return - (trig - 1) / 2


JOYSTICK_DATA = {
    "axes": {
        "left_stick_horizontal": 0,
        "left_stick_vertical": 1,
        "right_stick_horizontal": 2,
        "right_stick_vertical": 3,
    },
    "buttons": {
        "A_button": 0,
        "B_button": 1,
        "X_button": 2,
        "Y_button": 3,
        "left_bumper": 4,
        "right_bumper": 5,
        "left_trigger": 6,
        "right_trigger": 7,
        "back_button": 8,
        "start_button": 9,
        "left_stick_button": 10,
        "right_stick_button": 11,
        "up_cross": 12,
        "down_cross": 13,
        "left_cross": 14,
        "right_cross": 15,
        "power_button": 16,
    }
}


class JoystickTranslator:

    def __init__(self):
        self.AXES_ID: dict = JOYSTICK_DATA["axes"]
        self.BUTTONS_ID: dict = JOYSTICK_DATA["buttons"]

    def translate(self, data):
        inputs = dict((name, data["buttons"][id])
                      for name, id in self.BUTTONS_ID.items())
        inputs.update(
            dict(
                (name, data["axes"][id]) for name, id in self.AXES_ID.items()))

        return inputs


class Debouncing:
    def __init__(self, threshold_low=0.45, threshold_high=0.55):
        self.state = False
        self.threshold_low = threshold_low
        self.threshold_high = threshold_high
        self.is_rising_edge = False
        self.is_falling_edge = False
    
    def update(self, value):
        if self.state == False and value > self.threshold_high:
            self.state = True
            self.is_rising_edge = True
            self.is_falling_edge = False
        elif self.state == True and value < self.threshold_low:
            self.state = False
            self.is_rising_edge = False
            self.is_falling_edge = True
        else:
            self.is_rising_edge = False
            self.is_falling_edge = False
