from manip.arm_joystick_control.joystick import JoystickControl
import os
import time
import curses
from typing import Tuple, List

class TerminalGui:
    def __init__(self, window: curses.window):
        self.update_interval = 0.1
        self.last_update = time.monotonic()
        self.window = window
        self._draw_canvas()

    def _draw_canvas(self):
        self.window.addstr(0, 0, f"Joystick manip controller v1.0", curses.color_pair(1) | curses.A_BOLD)
        self.window.addstr(1, 0, f"------------------------------", curses.color_pair(1))
        self.window.addstr(2, 0, f"- mode:   ", curses.color_pair(1))
        self.window.addstr(3, 0, f"- control:", curses.color_pair(1))
        self.window.addstr(6, 0, f"Waiting for gamepad input...", curses.color_pair(1))
        self.window.addstr(13, 0, "        Y - FK mode           \n" +
                                  "     X     B - IK mode - base \n" +
                                  "        A - IK mode - tool    \n\n" +
                                  " use bumpers for more DOFs    \n" +
                                  " use triggers for gripper ctrl", curses.color_pair(1))
        self.window.refresh()

    def update(self, axes: List[float], buttons: List[int], space_mode: JoystickControl.SpaceMode, movement_mode: JoystickControl.MovementMode, 
              frame_id: str):
        if time.monotonic() - self.last_update < self.update_interval:
            return
        space_mode_str = "pose servo (IK)" if space_mode == JoystickControl.SpaceMode.CARTESIAN else "joint control (FK)"
        if space_mode == JoystickControl.SpaceMode.CARTESIAN:
            frame_id_str = f"- frame id: {frame_id}"
            movement_mode_str = "linear" if movement_mode == JoystickControl.MovementMode.LINEAR else "angular"
        else:
            frame_id_str = ""
            movement_mode_str = "joints 1-4" if movement_mode == JoystickControl.MovementMode.LINEAR else "joints 3-6"
        

        self.window.addstr(2, 12, f"{space_mode_str:<20}", curses.color_pair(3))
        self.window.addstr(3, 12, f"{movement_mode_str:<20}", curses.color_pair(3))
        self.window.addstr(4, 0, f"{frame_id_str:<25}", curses.color_pair(1))
        help_left_args, help_right_args = self._get_help_args(space_mode, movement_mode)
        left_marker_pos, right_marker_pos = self._get_marker_pos(axes)
        self._draw_help((6, 0), *help_left_args, left_marker_pos)
        self._draw_help((6, 16), *help_right_args, right_marker_pos)
        self.window.refresh()
        self.last_update = time.monotonic()

    def _draw_help(self, pos: Tuple[int, int], color_v, str_v_up, str_v_down, color_h, str_h_left, str_h_right, marker_pos: Tuple[int, int]):
        x = pos[1]
        y = pos[0]
        self.window.addstr(y, x,   f"     A {str_v_up}        ", curses.color_pair(color_v))
        self.window.addstr(y+1, x, f"     |      ", curses.color_pair(color_v))
        self.window.addstr(y+2, x, f" <---+--->  ", curses.color_pair(color_h))
        self.window.addstr(y+3, x, f"     |      ", curses.color_pair(color_v))
        self.window.addstr(y+4, x, f"     V {str_v_down}   ", curses.color_pair(color_v))
        self.window.addstr(y+2, x+5, f"+", curses.color_pair(1))
        self.window.addstr(y+marker_pos[1], x+marker_pos[0], f"o", curses.color_pair(1) | curses.A_BOLD)

        self.window.addstr(y+1, x, f"{str_h_left}", curses.color_pair(color_h))
        self.window.addstr(y+1, x+9, f"{str_h_right}", curses.color_pair(color_h))

    
    def _get_help_args(self, space_mode: JoystickControl.SpaceMode, movement_mode: JoystickControl.MovementMode):
        if space_mode == JoystickControl.SpaceMode.CARTESIAN:
            if movement_mode == JoystickControl.MovementMode.LINEAR:
                return ((2, "+x", "-x", 3, "+y", "-y"), (4, "+z", "-z", 3, "+y", "-y"))
            else:
                return ((3, "+y", "-y", 2, "-x", "+x"), (3, "+y", "-y", 4, "+z", "-z"))
        else:
            if movement_mode == JoystickControl.MovementMode.LINEAR:
                return ((1, "+j2", "-j2", 1, "+j1", "-j1"), (1, "-j3", "+j3", 1, "-j4", "+j4"))
            else:
                return ((1, "-j5", "+j5", 1, "-j6", "+j6"), (1, "-j3", "+j3", 1, "-j4", "+j4"))
    
    def _get_marker_pos(self, axes: List[float]):
        size_x = 4
        size_y = 2
        left_x = size_x - int(axes[0] * size_x) + 1
        left_y = size_y - int(axes[1] * size_y)
        right_x = size_x - int(axes[3] * size_x) + 1
        right_y = size_y - int(axes[4] * size_y)
        return (left_x, left_y), (right_x, right_y)


def main(window: curses.window):
    curses.start_color()
    curses.use_default_colors()
    curses.curs_set(0)
    curses.init_pair(1, curses.COLOR_WHITE, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_GREEN, -1)
    curses.init_pair(4, curses.COLOR_BLUE, -1)
    
    joystick_control = JoystickControl()
    joystick_control.add_gui(TerminalGui(window))
    joystick_control.run()

if __name__ == "__main__":
    curses.wrapper(main)