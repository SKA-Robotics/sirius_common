import curses
import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
import yaml
import rospkg
import os

actual_covariance = [999.0]
actual_position = [0.0, 0.0, 0.0]
covariance_status = "none"
gps_ready = False
gps_sub = None

def gps_callback(msg):
    if len(msg.position_covariance) > 0:
        actual_covariance[0] = msg.position_covariance[0]

        actual_position[0] = msg.latitude
        actual_position[1] = msg.longitude
        actual_position[2] = msg.altitude

def gps():
    global covariance_status, gps_ready, gps_sub
    subprocess.Popen(["roslaunch", "sirius_navigation", "gnss.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
    )
    gps_sub = rospy.Subscriber('gps/fix', NavSatFix, gps_callback)

    covariance_status = "waiting for data"
    gps_ready = False
                                          

status_slam = "inactive"
slam_launched = False
odom_sub = None
last_pos = [None, None]
driven_distance = [0.0]

def odometry_callback(msg):
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

def slam(mode):
    global status_slam, slam_launched, odom_sub, actual_position
    
    if mode == 'gps':
        rospy.set_param('latitude', actual_position[0])
        rospy.set_param('longitude', actual_position[1])
        rospy.set_param('altitude', actual_position[2])
        status_slam = "active (pos from gps)-> drive for 40m"
        
    elif mode == 'yaml':
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('sirius_spectacularai')
        slam_params_path = os.path.join(pkg_path, "config", "slam_params.yaml")
        with open(slam_params_path, 'r') as file:
            coords = yaml.safe_load(file)

        yaml_lat = coords['latitude']
        yaml_lon = coords['longitude']
        yaml_alt = coords['altitude']
        
        status_slam = "active (yaml) -> drive for 40m"
        
    subprocess.Popen(["roslaunch", "sirius_spectacularai", "slam.launch"], 
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL)
                     
    odom_sub = rospy.Subscriber('slam/global_odometry', Odometry, odometry_callback)
    
    slam_launched = True

status_localization = "inactive"
def localization():
    global status_localization
    subprocess.Popen(["roslaunch", "sirius_navigation", "localization.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
                    )
    status_localization = "active"

status_mapping = "inactive"
mapping_launched = False

def mapping():
    global status_mapping
    subprocess.Popen(["roslaunch", "sirius_mapping", "sirius_mapping.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
                    )

    status_mapping = "active"
    global mapping_launched
    mapping_launched = True


status_navigation = "inactive"

def navigation():
    global status_navigation
    subprocess.Popen(["roslaunch", "sirius_navigation", "navigation.launch"],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL
                    )
    
    status_navigation = "active"

def main(stdscr):
    global covariance_status, actual_covariance, gps_ready, gps_sub, status_gps
    global status_slam, slam_launched, driven_distance
    global status_localization
    global status_mapping, mapping_launched
    global status_navigation

    curses.curs_set(0)
    stdscr.timeout(100)

    options = [
        "1. GPS startup",
        "2. SLAM",
        "   - current pos (gps)",
        "   - itc",
        "3. localization",
        "4. mapping",
        "5. navigation",
        "4. quit programm"
    ]
    current_row = 0

    while True:
        stdscr.clear()

        if gps_sub is not None and not gps_ready:
            if actual_covariance[0] != 999.0: 
                if actual_covariance[0] <= 0.002:
                    gps_ready = True
                    gps_sub.unregister()
                    covariance_status = "ready"
                else:
                    covariance_status = f"{round(actual_covariance[0], 4)}"

        stdscr.addstr(1, 2, "AUTONOMY STARTUP", curses.A_BOLD)
        stdscr.addstr(3, 2, f"status gps = [{covariance_status}]")
        stdscr.addstr(4, 2, f"status slam = [{status_slam}]")
        stdscr.addstr(5, 2, f"status localization = [{status_localization}]")
        stdscr.addstr(6, 2, f"status mapping = [{status_mapping}]")
        stdscr.addstr(7, 2, f"status navigation = [{status_navigation}]")
        stdscr.addstr(8, 2, "-" * 40)
        
        stdscr.addstr(10, 2, "functions:", curses.A_BOLD)
        for index, option_txt in enumerate(options):
            x = 4
            y = 12 + index
            
            if index == current_row:
                stdscr.addstr(y, x, f"> {option_txt} <", curses.A_REVERSE)
            else:
                stdscr.addstr(y, x, f"  {option_txt}  ")

        stdscr.refresh()

        klawisz = stdscr.getch()

        if klawisz == curses.KEY_UP and current_row > 0:
            current_row -= 1
        if klawisz == curses.KEY_DOWN and current_row < len(options) - 1:
            current_row += 1


        if klawisz in [curses.KEY_ENTER, 10, 13]: # Enter
            if current_row == 0 and gps_sub is None:
                gps()

            if current_row == 1:
                pass

            if current_row == 2:
                if not gps_ready:
                    status_slam = "error: gps not active"
                else:
                    slam('gps')

            if current_row == 3:
                if not gps_ready:
                    status_slam = "error: gps not active"
                else:
                    slam('yaml')

            if current_row == 4:
                if not slam_launched:
                    status_localization = "error start slam first"
                elif driven_distance[0] < 40.0:
                    status_localization = f"drive 40m, now: {round(driven_distance[0], 1)}"
                else:
                    localization()
            
            if current_row == 5:
                if status_localization != "active":
                    status_mapping = "error: start localization first"
                else:
                    mapping()

            if current_row == 6:
                if status_mapping != "active":
                    status_navigation = "error: start mapping first"
                else:
                    navigation()
            if current_row == 7:
                break

if __name__ == '__main__':
    curses.wrapper(main)