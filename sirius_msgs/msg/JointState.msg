# This is a message that holds data to describe or order the state of an actuator. 
# It is intended only for internal use. Should be used only when standard messages
# are not sufficient.
#
# The state of the actuator (revolute or prismatic) is defined by:
#  * the position of the actuator (rad or m),
#  * the velocity of the actuator (rad/s or m/s),
#  * the acceleration of the actuator (rad/s^2 or m/s^2) and 
#  * the effort that is applied in the actuator (Nm or N).
#  * the duty driving the actuator (-100% ... 100%)

# The header specifies the time at which the actuator states were recorded/ordered. 
#
# This message consists of a multiple arrays, one for each part of the actuator state. 
# The goal is to make each of the fields optional. When e.g. your actuator have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays should contain just one value or be empty.

Header header

float64[] position
float64[] velocity
float64[] acceleration
float64[] effort
float64[] duty