#!/usr/bin/env python

import rospy
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import pandas as pd
import numpy as np
import ros_numpy as ros_np
import threading
from copy import deepcopy

class ObstacleWorld:
    def __init__(self):
        self.sdf_version = rospy.get_param('~gazebo/sdf_version', 1.4)

        self.m = rospy.get_param('~gazebo/obs/mass', 50.0)
        self.h = rospy.get_param('~gazebo/obs/height', 2.0)

        self.quadrotor_pos = None
        self.quadrotor_rot = None
        self.quadrotor_r = rospy.get_param('~quadrotor/radius', 0.3)
        self.odom_sub = rospy.Subscriber('uav/cog/odom', Odometry, self.odomCb)
        self.collision_pub = rospy.Publisher('collision_flag', Empty, queue_size = 1)

        self.lock = threading.Lock()

        rate = rospy.get_param('~rate', 40.0) # Hz
        shift_x = rospy.get_param('~shift_x', 0) #init x
        shift_y = rospy.get_param('~shift_y', 0) #init y
        print("shift_x: ",shift_x, "shift_y: ",shift_y)

        wall_y_position = 1.75
        # debug
        # r = 0.5
        # p = [0, 0, 0] # x, y, z
        # name = 'obj1'
        # self.spwan_obstacle(name, self.m, p, r, self.h)
        # return

        file = rospy.get_param('~obstacle_world_file')
        if file is None:
            rospy.logerr("no valid obsracle world file")
            return

        try:
            df = pd.read_csv(file, header=None)
            self.obs = dict()
            for i in range(len(df)):
                name = 'obj' + str(i+1)
                self.obs[name] = {'p': np.array(df.loc[i, 1:3].tolist())+np.array([shift_x,shift_y,0]), 'r': df.at[0,8]}
                self.obs[name]['p'][2] = self.h / 2
                self.spwanObstacle(name, self.m, self.obs[name]['p'], self.obs[name]['r'], self.h)
            rospy.Timer(rospy.Duration(1.0/rate), self.mainProcess)
        except pd.errors.EmptyDataError as e:
            print("tree data is empty")
            # print("self.obs: ",self.obs)
        # self.spawnWall("right_wall", 0.5, np.array([40,wall_y_position,2]) + np.array([shift_x,shift_y,0]), 90, 0.01, 4)
        # self.spawnWall("left_wall", 0.5, np.array([40,-wall_y_position,2]) + np.array([shift_x,shift_y,0]), 90, 0.01, 4)
        # self.spawnWall("back_wall", 0.5, np.array([-0.7,-0.4,2]) + np.array([shift_x,shift_y,0]), 0.01, 1.5, 4)

    def odomCb(self, msg):
        self.lock.acquire()
        self.quadrotor_pos = ros_np.numpify(msg.pose.pose.position)
        self.quadrotor_rot = ros_np.numpify(msg.pose.pose.orientation)
        self.lock.release()

    def mainProcess(self, event):
        if self.quadrotor_pos is None:
            return

        self.lock.acquire()
        quad_pos = deepcopy(self.quadrotor_pos)
        quad_rot = deepcopy(self.quadrotor_rot)
        self.lock.release()

        # check the collision
        for n, conf in self.obs.items():
            obs_pos = conf['p']
            obs_r = conf['r']
            cog_dist = np.linalg.norm(obs_pos[:2] - quad_pos[:2])
            min_dist = cog_dist - obs_r - self.quadrotor_r
            rospy.logdebug("{}, pos: {}, quad pos: {}, cog dist: {}, min_dist: {}".format(n, obs_pos, quad_pos, cog_dist, min_dist))
            if min_dist < 0:
                rospy.logwarn("{}, collision!! quadrotor: {}, obstacle: {} ".format(n, quad_pos, obs_pos))
                self.collision_pub.publish(Empty())

        # publish the obstacle position using LaserScan

    def spwanObstacle(self, name, m, p, r, h):

        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            # resp = delete_model(name)
            print("try works well")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        model_xml =  "<sdf version='{}'>".format(self.sdf_version) + \
                    "<model name='{}'>".format(name)  + \
                    "<allow_auto_disable>true</allow_auto_disable>" + \
                    "<pose> 0 0 0 0 0 0 </pose>" + \
                    "<static>true</static>" + \
                    "<link name='link'>" + \
                    "<velocity_decay>" + \
                    "<linear>0.01</linear>" + \
                    "<angular>0.01</angular>" + \
                    "</velocity_decay>" + \
                    "<inertial><mass> {} </mass>".format(m) + \
                    "<inertia>" + \
                    "<ixx> {} </ixx>".format(r**2/4.0 + h**2/12.0 * m) +\
                    "<iyy> {} </iyy>".format(r**2/4.0 + h**2/12.0 * m) + \
                    "<izz> {} </izz>".format(r**2/2.0 * m) + \
                    "<ixy> 0.0 </ixy>" + \
                    "<ixz> 0.0 </ixz>" + \
                    "<iyz> 0.0 </iyz>" + \
                    "</inertia>" + \
                    "</inertial>"  + \
                    "<collision name='collision'>" + \
                    "<geometry>" + \
                    "<cylinder>"  + \
                    "<radius> {} </radius>".format(r) + \
                    "<length> {} </length>".format(h) + \
                    "</cylinder>" + \
                    "</geometry>" + \
                    "</collision>" + \
                    "<visual name='visual'>" + \
                    "<geometry>" + \
                    "<cylinder>"  + \
                    "<radius> {} </radius>".format(r) + \
                    "<length> {} </length>".format(h) + \
                    "</cylinder>" + \
                    "</geometry>" + \
                    "<material>"  + \
                    "<ambient> 0.4 0.2 0.0 1 </ambient>"  + \
                    "<diffuse> 0.4 0.2 0.0 1 </diffuse>"  + \
                    "<specular>0.1 0.1 0.1 1</specular>"  + \
                    "<emissive>0 0 0 0</emissive>"  + \
                    "</material>" + \
                    "<cast_shadows>false</cast_shadows>" + \
                    "</visual>" + \
                    "</link>"  + \
                    "</model>" + \
                    "</sdf>"

        ret = gazebo_interface.spawn_sdf_model_client(name, model_xml, "obstacle", Pose(Point(p[0], p[1], p[2]), Quaternion(0,0,0,1)), '', '/gazebo')
        # set obstacle in model_xml setting (we cannot change obstacle shape now...)

        return ret

    def spawnWall(self, name, m, p, w, t, h):

        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            # resp = delete_model(name)
            print("try works well")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        model_xml =  "<sdf version='{}'>".format(self.sdf_version) + \
                    "<model name='{}'>".format(name)  + \
                    "<allow_auto_disable>true</allow_auto_disable>" + \
                    "<pose> 0 0 0 0 0 0 </pose>" + \
                    "<static>true</static>" + \
                    "<link name='link'>" + \
                    "<velocity_decay>" + \
                    "<linear>0.01</linear>" + \
                    "<angular>0.01</angular>" + \
                    "</velocity_decay>" + \
                    "<inertial><mass> {} </mass>".format(m) + \
                    "<inertia>" + \
                    "<ixx> {} </ixx>".format(m*(t**2+h**2)/12) +\
                    "<iyy> {} </iyy>".format(m*(h**2+w**2)/12) + \
                    "<izz> {} </izz>".format(m*(w**2+t**2)/12) + \
                    "<ixy> 0.0 </ixy>" + \
                    "<ixz> 0.0 </ixz>" + \
                    "<iyz> 0.0 </iyz>" + \
                    "</inertia>" + \
                    "</inertial>"  + \
                    "<collision name='collision'>" + \
                    "<geometry>" + \
                    "<box>"  + \
                    "<size>{} {} {} </size>".format(w, t, h) + \
                    "</box>" + \
                    "</geometry>" + \
                    "</collision>" + \
                    "<visual name='visual'>" + \
                    "<geometry>" + \
                    "<box>"  + \
                    "<size>{} {} {} </size>".format(w, t, h) + \
                    "</box>" + \
                    "</geometry>" + \
                    "<material>"  + \
                    "<ambient> 0.4 0.2 0.0 0.3 </ambient>"  + \
                    "<diffuse> 0.4 0.2 0.0 0.3 </diffuse>"  + \
                    "<specular>0.1 0.1 0.1 0.3</specular>"  + \
                    "<emissive>0 0 0 0</emissive>"  + \
                    "</material>" + \
                    "</visual>" + \
                    "</link>"  + \
                    "</model>" + \
                    "</sdf>"

        ret = gazebo_interface.spawn_sdf_model_client(name, model_xml, "obstacle", Pose(Point(p[0], p[1], p[2]), Quaternion(0,0,0,1)), '', '/gazebo')
        # set obstacle in model_xml setting (we cannot change obstacle shape now...)

        return ret

if __name__=="__main__":
    rospy.init_node("obsracle_world")

    world = ObstacleWorld()
    rospy.spin()
