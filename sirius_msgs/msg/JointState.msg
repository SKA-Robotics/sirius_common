# Custom actuator state/command message for the Sirius rover.
# All arrays should contain one value or be empty (fields are optional).

std_msgs/Header header

float64[] position       # rad or m
float64[] velocity       # rad/s or m/s
float64[] acceleration   # rad/s^2 or m/s^2
float64[] effort         # Nm or N
float64[] duty           # -100 % ... 100 %
