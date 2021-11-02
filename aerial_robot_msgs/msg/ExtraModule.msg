int8 ADD = 1
int8 REMOVE = -1
int8 CLEAR = -2 # "clear" means removing all

Header header
int8 action # 1: add; -1: remove; -2 clear (all)
string module_name
string parent_link_name
geometry_msgs/Transform transform
geometry_msgs/Inertia inertia
