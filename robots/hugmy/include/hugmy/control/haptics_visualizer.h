#pragma once

# include <hugmy/control/haptics_controller.h>
# include <visualization_msgs/MarkerArray.h>
# include <std_msgs/Float32.h>

class HapticsVisualizer : public HapticsController {
public:
    HapticsVisualizer(ros::NodeHandle& nh);
    void updateRviz();

private:
    void publishVectorArrow(const Eigen::Vector2d& vec, const geometry_msgs::Point& start_point,
                            const std::string& ns, int id, float r, float g, float b);
    ros::Publisher marker_array_pub_;
    visualization_msgs::MarkerArray arrow_array_;
    ros::Publisher norm_pub_;
    ros::Publisher thrust_0_pub_;
    ros::Publisher thrust_1_pub_;
    ros::Publisher thrust_2_pub_;
    ros::Publisher thrust_3_pub_;  

};

