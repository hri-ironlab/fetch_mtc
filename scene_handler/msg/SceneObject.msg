# Define box, sphere, cylinder, cone 
# All shapes are defined to have their bounding boxes centered around 0,0,0.
uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type

byte ADD=0
byte REMOVE=1
byte APPEND=2
byte MOVE=3

# Operation to be performed
byte operation

string id

geometry_msgs/Vector3 dimensions

# Place Position
geometry_msgs/Pose spawn_pose