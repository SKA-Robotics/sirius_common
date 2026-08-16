import curses
import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math

actual_covariance = [999.0]
actual_position = [0.0, 0.0, 0.0]

def create_window(stdscr):
    stdscr.clear()
    stdscr.box()

def gps_callback(msg):
    if len(msg.position_covariance) > 0:
        actual_covariance[0] = msg.position_covariance[0]

        actual_position[0] = msg.latitude
        actual_position[1] = msg.longitude
        actual_position[2] = msg.altitude

def gps(stdscr):
    create_window(stdscr)
    stdscr.addstr(2, 2, "Step 1 - gps startup, q to quit")
    subprocess.Popen(["roslaunch", "sirius_navigation", "gnss.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
    )
    gps_sub = rospy.Subscriber('gps/fix', NavSatFix, gps_callback)

    stdscr.timeout(100) 
    ready = False

    while True:

        if actual_covariance[0] <= 0.002 and not ready:
            ready = True
            gps_sub.unregister()

        if ready:
            stdscr.addstr(6, 2, 'covariance is ok, press s to proceed to slam')
        else:
            stdscr.addstr(6, 2, f'covariance: {actual_covariance[0]}')

        try:
            key = stdscr.getkey().lower()
            if ready and key == 's':
                return 's'
            elif key == 'q':
                return
        except curses.error:
            pass

last_pos = [None, None]
driven_distance = [0.0]

def odometry_callback(msg, stdscr):
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    if last_pos[0] == None:
        last_pos[0] = current_x
        last_pos[1] = current_y
    else:
        dx = current_x - last_pos[0]
        dy = current_y - last_pos[1]

        vector = math.sqrt(dx**2 + dy**2)
        driven_distance[0] += vector

        last_pos[0] = current_x
        last_pos[1] = current_y


def slam(stdscr):
    stdscr.timeout(100)
    create_window(stdscr)
    stdscr.addstr(2, 2, "Step 2 - slam startup, q to quit")
    slam_launched = False

    while True:

        if not slam_launched:
            create_window(stdscr)
            stdscr.addstr(4, 2, "choose a reference point:")
            stdscr.addstr(5, 4, " 1 - current position")
            stdscr.addstr(6, 4, "2 - saved position")

        elif slam_launched:
            if driven_distance[0] >= 40.0:
                stdscr.addstr(14, 2, "drived 40m, press l")
                stdscr.refresh()


        try:
            key = stdscr.getkey().lower()

            if key == 'q':
                return #wyjscie
            
            if not slam_launched:
                # pkt referencyjny == gps
                if key == '1':
                    create_window(stdscr)
                    rospy.set_param('NAZWA_PARAMETRU_PKT', actual_position)
                    stdscr.addstr(10, 2, "go shake the rover, then drive for 40m")
                    stdscr.addstr(12, 2, "next press l -> localization")
                    subprocess.Popen(["roslaunch", "sirius_spectacularai", "slam.launch"],
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL
                        )
                    odom_sub = rospy.Subscriber('slam/global_odometry', Odometry, odometry_callback, callback_args=stdscr)
                    slam_launched = True

                # pkt referencyjny == wybierasz
                elif key == '2':
                    create_window(stdscr)
                    stdscr.timeout(-1)
                    curses.echo()
                    stdscr.addstr(10, 2, "type the name of the reference point: ")
                    default = stdscr.getstr(10, 35).decode('utf-8')
                    rospy.get_param(default)
                    curses.noecho()
                    stdscr.timeout(100)
                    stdscr.addstr(12, 2, "go shake the rover, then drive for 40m")
                    stdscr.addstr(14, 2, "next press l -> localization")
                    subprocess.Popen(["roslaunch", "sirius_spectacularai", "slam.launch"],
                                                            stdout=subprocess.DEVNULL,
                                                            stderr=subprocess.DEVNULL
                                    )
                    odom_sub = rospy.Subscriber('slam/global_odometry', Odometry, odometry_callback, callback_args=stdscr)
                    slam_launched = True

            else:
                if key == 'l' and driven_distance[0] >= 40.0:
                    odom_sub.unregister()
                    return 'l'

        except curses.error:
            pass

def localization(stdscr):
    create_window(stdscr)
    stdscr.timeout(100)
    stdscr.addstr(2, 2, "Step 3 - localization startup, q to quit")
    subprocess.Popen(["roslaunch", "sirius_navigation", "localization.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
                    )
    while True:
        try:
            key = stdscr.getkey().lower()

            if key == 'q':
                return
            elif key == 'm':
                return 'm'

        except curses.error:
            pass

def mapping(stdscr):
    create_window(stdscr)
    stdscr.timeout(100)
    stdscr.addstr(2, 2, "Step 4 - mapping startup, q to quit")
    subprocess.Popen(["roslaunch", "sirius_mapping", "sirius_mapping.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
                    )
    while True:
        try:
            key = stdscr.getkey().lower()

            if key == 'q':
                return
            elif key == 'n':
                return 'n'

        except curses.error:
            pass

def navigation(stdscr):
    create_window(stdscr)
    stdscr.timeout(100)
    stdscr.addstr(2, 2, "Krok 5 - naviagtion startup, q to quit")
    subprocess.Popen(["roslaunch", "sirius_navigation", "navigation.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
                    )
    while True:
        try:
            key = stdscr.getkey().lower()

            if key == 'q':
                return

        except curses.error:
            pass

def main(stdscr):
    create_window(stdscr)
    stdscr.timeout(100)
    state = None

    while True:
        if state == None:
            try:
                state = stdscr.getkey().lower()
            except curses.error():
                continue

        try:
            if state == 'g':
                state = gps(stdscr)
            elif state == 's':
                state = slam(stdscr)
            elif state == 'l':
                state = localization(stdscr)
            elif state == 'm':
                state = mapping(stdscr)
            elif state == 'n':
                state = navigation(stdscr)
            elif state == 'q':
                return
            else:
                state == None
        except curses.error():
            pass
                

if __name__ == '__main__':
    curses.wrapper(main)