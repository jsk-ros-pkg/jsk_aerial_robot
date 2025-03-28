#!/usr/bin/python3

import rospy


from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import ObstacleArray
from aerial_robot_msgs.msg import FlightNav
# from sensor_msgs.msg import Image

import torch
import numpy as np

from scipy.spatial.transform import Rotation as R 
from stable_baselines3.common.utils import get_device
from stable_baselines3.ppo.policies import MlpPolicy

# from squaternion import Quaternion


class AgileQuadState:
    def __init__(self, quad_state):
        # self.t = quad_state.t

        self.pos = np.array([quad_state.pose.pose.position.x,
                             quad_state.pose.pose.position.y,
                             quad_state.pose.pose.position.z], dtype=np.float32)
        self.att = np.array([quad_state.pose.pose.orientation.x,
                             quad_state.pose.pose.orientation.y,
                             quad_state.pose.pose.orientation.z,
                             quad_state.pose.pose.orientation.w], dtype=np.float32)
        self.vel = np.array([quad_state.twist.twist.linear.x,
                             quad_state.twist.twist.linear.y,
                             quad_state.twist.twist.linear.z], dtype=np.float32)
        self.omega = np.array([quad_state.twist.twist.angular.x,
                               quad_state.twist.twist.angular.y,
                               quad_state.twist.twist.angular.z], dtype=np.float32)


class AgilePilotNode:
    def __init__(self, vision_based=False):
        print("Initializing agile_pilot_node...")
        rospy.init_node('policy_navigator', anonymous=False)

        self.vision_based = vision_based
        self.ppo_path = rospy.get_param("~ppo_path")
        self.publish_commands = False
        self.state = None
        self.goal_lin_vel = np.array([5,0,0],dtype="float32")
        self.world_box = np.array([-0.2, 5.0 ,-1.5, 1.5, 0.0, 2.0],dtype="float32")
        # should change depending on world flame's origin

        quad_name = 'multirotor'

        
        # Logic subscribers
        self.start_sub = rospy.Subscriber("/" + quad_name + "/start_navigation", Empty, self.start_callback,
                                          queue_size=1, tcp_nodelay=True)
        

        # Observation subscribers
        self.odom_sub = rospy.Subscriber("/" + quad_name + "/uav/cog/odom", Odometry, self.state_callback,
                                         queue_size=1, tcp_nodelay=True)

        self.obstacle_sub = rospy.Subscriber("/" + quad_name + "/polar_pixel", ObstacleArray,
                                             self.obstacle_callback, queue_size=1, tcp_nodelay=True)

        # Command publishers
        self.linvel_pub = rospy.Publisher("/" + quad_name + "/uav/nav", FlightNav,
                                          queue_size=1)
        print("Initialization completed!")


    def state_callback(self, state_data):
        self.state = AgileQuadState(state_data)

    def obstacle_callback(self, obs_data):
        if self.vision_based:
            return
        if self.state is None:
            return
        rl_policy = None
        if self.ppo_path is not None:
            rl_policy = self.load_rl_policy(self.ppo_path)
        vel_msg = self.rl_example(state=self.state, obstacles=obs_data, rl_policy=rl_policy)

        if self.publish_commands:
            self.linvel_pub.publish(vel_msg)

    def rl_example(self, state, obstacles, rl_policy=None):
        policy, obs_mean, obs_var, act_mean, act_std = rl_policy
        obs_vec = np.array(obstacles.boxel)

        # Convert state to vector observation
        goal_vel = self.goal_lin_vel
        world_box = self.world_box

        att_aray = state.att

        rotation_matrix = R.from_quat(att_aray)
        euler = rotation_matrix.as_euler('xyz')
        rotation_matrix = rotation_matrix.as_matrix().reshape((9,), order="F")
        
        obs = np.concatenate([
            goal_vel, rotation_matrix, state.pos, state.vel, obs_vec, 
            np.array([world_box[2] - state.pos[1], world_box[3] - state.pos[1], 
            world_box[4] - state.pos[2] , world_box[5] - state.pos[2]]),
    state.omega], axis=0).astype(np.float64)

        obs = obs.reshape(-1, obs.shape[0])
        norm_obs = self.normalize_obs(obs, obs_mean, obs_var)
        #  compute action
        action, _ = policy.predict(norm_obs, deterministic=True)
        action = (action * act_std + act_mean)[0, :]

        # command_mode = 1
        # command = AgileCommand(command_mode)
        # command.t = state.t
        # command.collective_thrust = action[0] 
        # command.bodyrates = action[1:4]

        # cmd freq is same as simulator? cf. in RL dt = 0.02
        command = FlightNav()
        command.target = 1
        command.target_xy_nav_mode = 4
        command.target_z_nav_mode = 4
        command.pos_x = state.pos[0] + action[0]
        command.pos_y = state.pos[1] + action[1]
        command.pos_z = state.pos[2] + action[2]

        command.vel_x = state.vel[0] + action[3]
        command.vel_y = state.vel[1] + action[4]
        command.vel_z = state.vel[2] + action[5]

        # set yaw cmd from state based (in learning, controller is set by diff of yaw angle)
        command.target_yaw = euler[2] + action[6]


        return command
    
    def load_rl_policy(self, policy_path):
        print("============ policy_path: ", policy_path)
        policy_dir = policy_path  + "/policy.pth"
        rms_dir = policy_path + "/rms.npz"

        act_mean = np.array([0.0,0.0,0.0,0.0, 0.0, 0.0, 0.0])[np.newaxis, :] 
        act_std = np.array([0.6, 0.6, 0.3, 1.0, 1.0, 1.0, 0.1])[np.newaxis, :] 

        rms_data = np.load(rms_dir)
        obs_mean = np.mean(rms_data["mean"], axis=0)
        obs_var = np.mean(rms_data["var"], axis=0)

        # # -- load saved varaiables 
        device = get_device("auto")
        saved_variables = torch.load(policy_dir, map_location=device)
        print("type of saved_variables[data]", type(saved_variables["data"]),'\n')
        for key in saved_variables["data"]:
            print("key",key)

        # Create policy object
        policy = MlpPolicy(**saved_variables["data"])
        #
        policy.action_net = torch.nn.Sequential(policy.action_net, torch.nn.Tanh())
        # Load weights
        policy.load_state_dict(saved_variables["state_dict"], strict=False)
        policy.to(device)

        return policy, obs_mean, obs_var, act_mean, act_std


    def start_callback(self, data):
        print("Start publishing commands!")
        self.publish_commands = True
    
    def normalize_obs(self, obs, obs_mean, obs_var):
        return (obs - obs_mean) / np.sqrt(obs_var + 1e-8)


if __name__ == '__main__':
    agile_pilot_node = AgilePilotNode()
    rospy.spin()