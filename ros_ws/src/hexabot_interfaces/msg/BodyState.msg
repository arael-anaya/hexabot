std_msgs/Header header
uint8 mode #0=SIT 1=STAND 2=WALK 3=ESTOP
geometry_msgs/Twist velocity # body-frame command, limited already (?)
geometry_msgs/Pose pose_offset # body height, roll, pitch, yaw offset
string gait_type

