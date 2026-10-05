# predicted x and u for NMPC
Header header

string child_frame_id

# ith round, jth state:
# x(i, j) = data[dim_stride[1]*i + j]
std_msgs/Float64MultiArray x
std_msgs/Float64MultiArray u
