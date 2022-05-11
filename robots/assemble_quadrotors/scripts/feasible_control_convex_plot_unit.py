#!/usr/bin/env python

import sys
import time
import rospy
import tf
import math
import threading
from std_msgs.msg import Float32
from aerial_robot_msgs.msg import FeasibleControlConvexInfo
import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3
import matplotlib.patches as patches
from sympy import Plane, Point3D
import networkx as nx
from copy import deepcopy


# https://stackoverflow.com/questions/49098466/plot-3d-convex-closed-regions-in-matplot-lib
def simplify(triangles):
    """
    Simplify an iterable of triangles such that adjacent and coplanar triangles form a single face.
    Each triangle is a set of 3 points in 3D space.
    """

    # create a graph in which nodes represent triangles;
    # nodes are connected if the corresponding triangles are adjacent and coplanar
    G = nx.Graph()
    G.add_nodes_from(range(len(triangles)))
    for ii, a in enumerate(triangles):
        for jj, b in enumerate(triangles):
            if (ii < jj): # test relationships only in one way as adjacency and co-planarity are bijective
                if is_adjacent(a, b):
                    if is_coplanar(a, b, np.pi / 180.):
                        G.add_edge(ii,jj)

                        # triangles that belong to a connected component can be combined
                        components = list(nx.connected_components(G))
                        simplified = [set(flatten(triangles[index] for index in component)) for component in components]

                        # need to reorder nodes so that patches are plotted correctly
                        reordered = [reorder(face) for face in simplified]

                        return reordered


def is_adjacent(a, b):
    return len(set(a) & set(b)) == 2 # i.e. triangles share 2 points and hence a side


def is_coplanar(a, b, tolerance_in_radians=0):
    a1, a2, a3 = a
    b1, b2, b3 = b
    plane_a = Plane(Point3D(a1), Point3D(a2), Point3D(a3))
    plane_b = Plane(Point3D(b1), Point3D(b2), Point3D(b3))
    if not tolerance_in_radians: # only accept exact results
        return plane_a.is_coplanar(plane_b)
    else:
        angle = plane_a.angle_between(plane_b).evalf()
        angle %= np.pi # make sure that angle is between 0 and np.pi
        return (angle - tolerance_in_radians <= 0.) or  ((np.pi - angle) - tolerance_in_radians <= 0.)


flatten = lambda l: [item for sublist in l for item in sublist]


def reorder(vertices):
    """
    Reorder nodes such that the resulting path corresponds to the "hull" of the set of points.

    Note:
    -----
    Not tested on edge cases, and likely to break.
    Probably only works for convex shapes.

    """
    if len(vertices) <= 3: # just a triangle
        return vertices
    else:
        # take random vertex (here simply the first)
        reordered = [vertices.pop()]
        # get next closest vertex that is not yet reordered
        # repeat until only one vertex remains in original list
        vertices = list(vertices)
        while len(vertices) > 1:
            idx = np.argmin(get_distance(reordered[-1], vertices))
            v = vertices.pop(idx)
            reordered.append(v)
            # add remaining vertex to output
            reordered += vertices
            return reordered

def get_distance(v1, v2):
    v2 = np.array(list(v2))
    difference = v2 - v1
    ssd = np.sum(difference**2, axis=1)
    return np.sqrt(ssd)


class FeasibleControlConvexPlot():
    def __init__(self, name="feasible_control_convex_plot"):


        self.xyz_equal_scale = rospy.get_param("~xyz_equal_scale", True)

        rospy.Subscriber("feasible_control_convex_info_unit", FeasibleControlConvexInfo, self.cb)

    def cb(self, msg):

        force_radius = msg.fc_f_min
        torque_radius = msg.fc_t_min
        gravity_force = msg.total_mass * 9.8

        cnt = 0
        force_vertices = np.empty((0,3), float)
        while cnt < 2 ** len(msg.u):
            vertice = np.array([0.0, 0.0, -gravity_force])
            for i in range(len(msg.u)):
                if 2 ** i & cnt:
                    vertice += [msg.u[i].x, msg.u[i].y, msg.u[i].z]

            force_vertices = np.append(force_vertices, np.array([vertice]), axis=0)
            cnt += 1

        cnt = 0
        torque_vertices = np.empty((0,3), float)
        while cnt < 2 ** len(msg.v):
            vertice = np.array([0.0, 0.0, 0.0])
            for i in range(len(msg.v)):
                if 2 ** i & cnt:
                    vertice += [msg.v[i].x, msg.v[i].y, msg.v[i].z]

            torque_vertices = np.append(torque_vertices, np.array([vertice]), axis=0)
            cnt += 1


        ax = a3.Axes3D(plt.figure("feasible_control_force_convex_unit"))
        x_min = np.amin(force_vertices, axis = 0)[0] - 1
        y_min = np.amin(force_vertices, axis = 0)[1] - 1
        z_min = np.amin(force_vertices, axis = 0)[2] - 1
        x_max = np.amax(force_vertices, axis = 0)[0] + 1
        y_max = np.amax(force_vertices, axis = 0)[1] + 1
        z_max = np.amax(force_vertices, axis = 0)[2] + 1
        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        ax.set_zlim([z_min, z_max])
        ax.xaxis.set_tick_params(labelsize=20)
        ax.yaxis.set_tick_params(labelsize=20)
        ax.zaxis.set_tick_params(labelsize=20)

        if self.xyz_equal_scale:
            xyz_min = np.amin(np.amin(force_vertices, axis = 0), axis = 0) - 1
            xyz_max = np.amax(np.amax(force_vertices, axis = 0), axis = 0) + 1

            ax.set_xlim([xyz_min, xyz_max])
            ax.set_ylim([xyz_min, xyz_max])
            ax.set_zlim([xyz_min, xyz_max])

        hull = ConvexHull(force_vertices)
        triangles = []

        for s in hull.simplices:

            sq = [
                (force_vertices[s[0], 0], force_vertices[s[0], 1], force_vertices[s[0], 2]),
                (force_vertices[s[1], 0], force_vertices[s[1], 1], force_vertices[s[1], 2]),
                (force_vertices[s[2], 0], force_vertices[s[2], 1], force_vertices[s[2], 2])
            ]
            triangles.append(sq)

            f = a3.art3d.Poly3DCollection([sq], alpha=0.5, linewidth=0.5)
            f.set_facecolor('c')
            f.set_edgecolor('k')
            ax.add_collection3d(f)


        # force minimum sphere
        u,v=np.mgrid[0:2*np.pi:50j, 0:np.pi:25j]
        x=np.cos(u)*np.sin(v) * force_radius
        y=np.sin(u)*np.sin(v) * force_radius
        z=np.cos(v) * force_radius
        ax.plot_wireframe(x, y, z, color='darkblue', linewidth=1.0)


        ax = a3.Axes3D(plt.figure("feasible_control_torque_convex_unit"))
        x_min = np.amin(torque_vertices, axis = 0)[0] - 1
        y_min = np.amin(torque_vertices, axis = 0)[1] - 1
        z_min = np.amin(torque_vertices, axis = 0)[2] - 1
        x_max = np.amax(torque_vertices, axis = 0)[0] + 1
        y_max = np.amax(torque_vertices, axis = 0)[1] + 1
        z_max = np.amax(torque_vertices, axis = 0)[2] + 1
        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        ax.set_zlim([z_min, z_max])
        ax.xaxis.set_tick_params(labelsize=20)
        ax.yaxis.set_tick_params(labelsize=20)
        ax.zaxis.set_tick_params(labelsize=20)

        if self.xyz_equal_scale:
            xyz_min = np.amin(np.amin(torque_vertices, axis = 0), axis = 0) - 1
            xyz_max = np.amax(np.amax(torque_vertices, axis = 0), axis = 0) + 1

            ax.set_xlim([xyz_min, xyz_max])
            ax.set_ylim([xyz_min, xyz_max])
            ax.set_zlim([xyz_min, xyz_max])

        hull = ConvexHull(torque_vertices)
        triangles = []

        for s in hull.simplices:

            sq = [
                (torque_vertices[s[0], 0], torque_vertices[s[0], 1], torque_vertices[s[0], 2]),
                (torque_vertices[s[1], 0], torque_vertices[s[1], 1], torque_vertices[s[1], 2]),
                (torque_vertices[s[2], 0], torque_vertices[s[2], 1], torque_vertices[s[2], 2])
            ]
            triangles.append(sq)

            f = a3.art3d.Poly3DCollection([sq], alpha=0.5, linewidth=0.5)
            f.set_facecolor('c')
            f.set_edgecolor('k')
            ax.add_collection3d(f)


        # torque minimum sphere
        u,v=np.mgrid[0:2*np.pi:50j, 0:np.pi:25j]
        x=np.cos(u)*np.sin(v) * torque_radius
        y=np.sin(u)*np.sin(v) * torque_radius
        z=np.cos(v) * torque_radius
        ax.plot_wireframe(x, y, z, color='darkblue', linewidth=1.0)

        plt.show()

if __name__=="__main__":
    rospy.init_node("feasible_control_conex_plot_unit")
    FeasibleControlConvexPlot()
    rospy.spin()
