Header header

geometry_msgs/Vector3 accelerometer
geometry_msgs/Vector3 gyrometer
geometry_msgs/Vector3 magnetometer 

geometry_msgs/Vector3 acceleration_body_frame
#geometry_msgs/Vector3 acceleration_intermidate_frame
geometry_msgs/Vector3 acceleration_world_frame
geometry_msgs/Vector3 acceleration_non_bias_world_frame

geometry_msgs/Vector3 angles # roll, pitch, yaw
float32 height


#tmp for prediction of kalman filter
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
