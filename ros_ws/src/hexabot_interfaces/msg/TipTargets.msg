std_msgs/Header header
geometry_msgs/Point[6] positions # tip of link 3 per leg, body frame (legs 1..6)
bool[6] contact # true = tip on the ground (stance)
float32[6] phase # 0 to 1 per leg
