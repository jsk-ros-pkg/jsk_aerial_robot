#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import tf, math, warnings, heapq
from nav_msgs.msg import Odometry
from enum import Enum
warnings.filterwarnings('ignore')

class Labels(Enum):
    FIX = 1
    NEAR = 2
    FAR = 3
class Labeled_Heap:
    def __init__(self):
        self.heap_list = [] #[(x,y),T]

    def add(self, item):
        self.heap_list.append(item)
        # self._heap_up()

    def extract_min(self):
        if len(self.heap_list) == 0:
            return None

        min_value = self.heap_list[0]
        last_item = self.heap_list.pop()

        if len(self.heap_list) > 0:
            self.heap_list[0] = last_item
            self._heap_down()

        return min_value

    def _heap_up(self):
        index = len(self.heap_list) - 1
        while index > 0:
            parent_index = (index - 1) // 2
            if self.heap_list[index][1] < self.heap_list[parent_index][1]:
                self.heap_list[index], self.heap_list[parent_index] = (
                    self.heap_list[parent_index],
                    self.heap_list[index],
                )
                index = parent_index
            else:
                break

    def _heap_down(self):
        index = 0
        while index * 2 + 1 < len(self.heap_list):
            left_child_index = index * 2 + 1
            right_child_index = index * 2 + 2

            smaller_child_index = left_child_index
            if (
                right_child_index < len(self.heap_list)
                and self.heap_list[right_child_index][1] < self.heap_list[left_child_index][1]
            ):
                smaller_child_index = right_child_index

            if self.heap_list[index][1] > self.heap_list[smaller_child_index][1]:
                self.heap_list[index], self.heap_list[smaller_child_index] = (
                    self.heap_list[smaller_child_index],
                    self.heap_list[index],
                )
                index = smaller_child_index
            else:
                break
    def is_empty(self):
        return len(self.heap_list) == 0

    def size(self):
        return len(self.heap_list)

    def get_heap(self):
        return self.heap_list


class FMM():
    def __init__(self):
        #node init
        rospy.init_node("FMM_node")
        #subscriber
        self.module_2_odom_sub = rospy.Subscriber("beetle2/uav/cog/odom", Odometry, self.obstacleCb)
        self.module_1_odom_sub = rospy.Subscriber("beetle1/uav/cog/odom", Odometry, self.agentPosCb)
        #transformations
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #param
        self.obs_x = 0
        self.obs_y = 0
        self.agt_x = 0
        self.agt_y = 0
        self.start_index = (0,0)
        self.goal_x = 0
        self.goal_y = 0
        self.goal_index = (0,0)
        self.obs_size =0.73 #diagonal dimension
        self.frame_size = 0.52
        self.v_max = 0.5
        self.x_offset = 0.2
        self.y_offset = 0
        self.z_offset = 0
        self.target_offset = np.array([self.frame_size/2.0 + self.x_offset, self.y_offset, self.z_offset])
        self.root_fc_dis = 0.129947
        self.leader = "beetle2"
        #heap
        self.heap = Labeled_Heap()
        #grid
        self.step = 0.1
        self.grid_size= 40
        self.x_range = np.arange(-self.step * self.grid_size/2.0, self.step * self.grid_size/2.0+self.step, self.step)
        self.y_range = np.arange(-self.step * self.grid_size/2.0, self.step * self.grid_size/2.0+self.step, self.step)
        self.discrete_coord = np.full((len(self.x_range), len(self.y_range),2),(0,0),dtype = object)
        self.velo_grid = np.zeros((self.grid_size, self.grid_size))
        self.fnn_grid = np.full((self.grid_size, self.grid_size,2),(10e5,"far"), dtype = object) #(time, label)
        self.time_grid = np.zeros((self.grid_size, self.grid_size))
        ##route
        self.route_x = []
        self.route_y = []
        #plot
        plt.ion()

    def obstacleCb(self, msg):
        pose = msg.pose.pose
        position = pose.position
        self.obs_x = position.x
        self.obs_y = position.y

   
    def agentPosCb(self, msg):
        pose = msg.pose.pose
        position = pose.position
        self.agt_x = position.x
        self.agt_y = position.y

    def velocityFunc(self,dis):
        if dis >=self.obs_size/2.0: # supposing obstacle shape as circle
            f = self.v_max * (math.tanh((dis-self.obs_size/2.0)-math.e) + 1)/2
        else:
            f = 10e-5
        return f

    def setGoalPos(self):
        self.br.sendTransform((self.target_offset[0], self.target_offset[1] , self.target_offset[2] + self.root_fc_dis),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "follower_target_odom",
                              self.leader+"/root")

        try:
            homo_transformed_target_odom = self.listener.lookupTransform('/world', '/follower_target_odom', rospy.Time(0))
            self.goal_x = homo_transformed_target_odom[0][0]
            self.goal_y = homo_transformed_target_odom[0][1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("")
        d_min = 100
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                coord_x = self.discrete_coord[x][y][0]
                coord_y = self.discrete_coord[x][y][1]              
                d = np.sqrt((coord_x - self.goal_x)**2 + (coord_y - self.goal_y)**2)
                if d < d_min:
                    d_min = d
                    self.goal_index = (x,y)

    def calcDiscretPos(self,pos_x,pos_y):
        discrete_x = math.floor(pos_x * 10.0)*0.1
        discrete_y = math.floor(pos_y * 10.0)*0.1
        if discrete_x < -2.0:
            discrete_x = -2.0
        elif discrete_x >2.0:
            discrete_x = 2.0
        if discrete_y < -2.0:
            discrete_y = -2.0
        elif discrete_y >2.0:
            discrete_y = 2.0 
        # index = (np.sum(self.x_range < discrete_x),np.sum(self.y_range < discrete_y))
        index_x = (int)(-(discrete_y-2.0)/self.step) -1
        index_y = (int)((discrete_x+2.0)/self.step) - 1
        if discrete_x > pos_x:
            index_x = index_x -1
        if discrete_y > pos_y:
            index_y = index_y +1
        index = (index_x,index_y)
        return discrete_x, discrete_y, index
        
    def initFnnGrid(self):
        self.fnn_grid[self.goal_index[0]][self.goal_index[1]] = (0,"fix")
        self.FixGrid(self.goal_index[0],self.goal_index[1])

    def FixGrid(self,fix_i,fix_j):
        self.fnn_grid[fix_i][fix_j][1] = 'fix'
        for (i,j) in {(fix_i,fix_j+1),(fix_i,fix_j-1),(fix_i+1,fix_j),(fix_i-1,fix_j)}:
            if i >=self.grid_size or j >=self.grid_size or i <0 or j<0:
                continue
            if self.fnn_grid[i][j][1] == 'fix':
                continue
            self.updateGrid(i,j)
            if self.fnn_grid[i][j][1] == 'far':
                self.fnn_grid[i][j][1] = 'near'
                self.heap.add([(i,j),float(self.fnn_grid[i][j][0])])
            else:
                self.heap._heap_up()
    def updateGrid(self,i,j):
        if i-1 <0:
            t_h = float(self.fnn_grid[i+1][j][0])
        elif i+1 >= self.grid_size:
            t_h = float(self.fnn_grid[i-1][j][0])
        else:
            t_h = min(float(self.fnn_grid[i-1][j][0]),float(self.fnn_grid[i+1][j][0]))
        if j-1 <0:
            t_v = float(self.fnn_grid[i][j+1][0])
        elif j+1 >= self.grid_size:
            t_v = float(self.fnn_grid[i][j-1][0])
        else:
            t_v = min(float(self.fnn_grid[i][j-1][0]),float(self.fnn_grid[i][j+1][0]))
        f_ij = 1.0/self.velo_grid[i][j]
        if f_ij > abs(t_h - t_v):
            t_ij = (t_h+t_v+math.sqrt(pow(f_ij,2) - pow((t_h-t_v),2)))/2.0
        else:
            t_ij = f_ij + min(t_h,t_v)
        self.fnn_grid[i][j][0] = float(t_ij)

    def calcFnn(self):
        while not self.heap.is_empty():
            item = self.heap.extract_min()
            self.FixGrid(item[0][0],item[0][1])

    def calcRoute(self):
        self.route_x = []
        self.route_y = []
        x,y,index =  self.calcDiscretPos(self.agt_x, self.agt_y)
        current_t = self.fnn_grid[index[0]][index[1]][0]
        count = 80000
        while current_t > 100 and count > 0:
            count = count-1
            '''
            t4 t3 t2
            t5 t0 t1
            t6 t7 t8
            '''
            i = index[0]
            j = index[1]
            # t_0 = self.fnn_grid[i][j][0]
            # if i + 1 >= self.grid_size:
            #     t_1 = t_2 = 10e5
            # else:
            #     t_1 = self.fnn_grid[i+1][j][0]
            # if j+1 >=self.grid_size:
            #     t_2 = t_3 = 10e5
            # else:
            #     t_3 = self.fnn_grid[i][j+1][0]
            # if i+1 < self.grid_size and j+1 < self.grid_size:
            #     t_2 = self.fnn_grid[i+1][j+1][0]
            
            # dt_dx = ((i+1-y) * (t_3 - t_0)+(y-i) * (t_2-t_1))
            # dt_dy = ((j+1-x) * (t_1 - t_0)+(x-j) * (t_2-t_3))
            # dl = np.array([dt_dx,dt_dy])/np.linalg.norm(np.array([dt_dx,dt_dy])) * self.step * 0.2
            # print(current_t)
            # x = x + dl[0]
            # y =  y+ dl[1]
            # x_d,y_d,index = self.calcDiscretPos(x, y) #update discrete pos and index
            # current_t = self.fnn_grid[i][j][0] #update time

            t_min = 10e5
            for (k,l) in {(i+1,j),(i+1,j+1),(i,j+1),(i-1,j+1),(i-1,j),(i-1,j-1),(i,j-1),(i+1,j-1)}:
                if k >= self.grid_size or k < 0 or l >= self.grid_size or l < 0 :
                    continue
                t = self.fnn_grid[k][l][0]
                if t < t_min:
                    t_min = t
                    index = (k,l)
            x = self.discrete_coord[index[0]][index[1]][0]
            y = self.discrete_coord[index[0]][index[1]][1]
            current_t = t_min

            self.route_x.append(x)
            self.route_y.append(y)
        
    def main(self):
        r = rospy.Rate(40)
        #init discrete coord
        for i, y in enumerate(self.y_range):
            for j, x in enumerate(self.x_range):
                self.discrete_coord[i, j] = (x,-y)
        while not rospy.is_shutdown():
            self.fnn_grid = np.full((self.grid_size, self.grid_size,2),(10e5,"far"), dtype = object)
            self.setGoalPos()
            for x in range(self.grid_size):
                for y in range(self.grid_size):
                    coord_x = self.discrete_coord[x][y][0]
                    coord_y = self.discrete_coord[x][y][1]
                    distance = np.sqrt((coord_x - self.obs_x)**2 + (coord_y - self.obs_y)**2)
                    self.velo_grid[x, y] = self.velocityFunc(distance)
            self.initFnnGrid()
            self.calcFnn()
            self.calcRoute()
            self.time_grid = self.fnn_grid[:,:,0].astype(np.float32)
            #plot
            plt.clf()
            plt.subplot(1, 2, 1)
            plt.imshow(self.velo_grid, cmap='jet', extent=[-2, 2, -2, 2], vmin=0, vmax=np.max(self.velo_grid))
            plt.colorbar(label='Velocity')

            plt.subplot(1, 2, 2)
            plt.imshow(self.time_grid, cmap='jet', extent=[-2, 2, -2, 2],  vmin=0, vmax=np.max(self.time_grid))
            plt.colorbar(label='Time')

            plt.subplot(1, 2, 1)
            plt.title('Velocity Field')
            plt.xlabel('X')
            plt.ylabel('Y')

            plt.subplot(1, 2, 2)
            plt.scatter(self.route_x,self.route_y,color = '#66ff00',marker = 'o', s = 10)
            plt.scatter([self.agt_x],[self.agt_y],color = 'red',marker = 'o', s = 10)
            plt.scatter([self.goal_x],[self.goal_y],color = 'red',marker = 'o', s = 10)
            plt.title('Time Field')
            plt.xlabel('X')
            plt.ylabel('Y')

            plt.tight_layout()
            plt.pause(0.1)

            r.sleep()
if __name__ == '__main__':
    try:
        fmm = FMM();
        fmm.main()
    except rospy.ROSInterruptException: pass
