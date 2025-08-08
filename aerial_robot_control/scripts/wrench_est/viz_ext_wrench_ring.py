#!/usr/bin/env python3
"""
 Created by jinjie on 25/08/08.
"""


import math
import rospy
from geometry_msgs.msg import Point, WrenchStamped
from visualization_msgs.msg import Marker


class RingMarker:
    def __init__(self):
        rospy.init_node("ring_marker_node")

        # Parameters
        self.radius = rospy.get_param("~radius", 0.45)  # [m]
        self.line_width = rospy.get_param("~line_width", 0.05)
        self.points_num = rospy.get_param("~points_num", 16)
        self.max_force = rospy.get_param("~max_force", 10.0)  # [N] for colour scaling

        # Publishers / Subscribers
        self.marker_pub = rospy.Publisher("ext_wrench_ring", Marker, queue_size=1, latch=True)
        rospy.Subscriber("beetle1/disturbance_wrench", WrenchStamped, self.wrench_callback)

        # Cache the most recent colour (start as green / no disturbance)
        self.colour = (0.0, 1.0, 0.0, 0.8)

        # Pre-compute ring geometry once
        self.ring_points = self._compute_ring_points()

        # Publish continuously so late-starting RViz instances still see the ring
        rate = rospy.Rate(20)  # [Hz]
        while not rospy.is_shutdown():
            self.publish_ring()
            rate.sleep()

    # ------------------------- Callbacks ------------------------- #
    def wrench_callback(self, msg: WrenchStamped):
        """Update ring colour according to force magnitude."""
        f = msg.wrench.force
        mag = math.sqrt(f.x**2 + f.y**2 + f.z**2)

        # Clamp and normalise
        ratio = max(0.0, min(mag / self.max_force, 1.0))

        # Simple green→red gradient
        r = ratio
        g = 1.0 - ratio
        b = 0.0
        a = 0.8
        self.colour = (r, g, b, a)

    # ----------------------- Publishing -------------------------- #
    def publish_ring(self):
        marker = Marker()
        # marker.header.frame_id = rospy.get_namespace() + "cog"
        marker.header.frame_id = "beetle1/cog"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ring"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.line_width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.colour
        marker.points = self.ring_points

        self.marker_pub.publish(marker)

    # ----------------------- Helpers ----------------------------- #
    def _compute_ring_points(self):
        """Return a closed list of Point objects forming a circle."""
        pts = []
        for i in range(self.points_num + 1):  # +1 to close the loop
            angle = 2.0 * math.pi * i / self.points_num
            p = Point(self.radius * math.cos(angle), self.radius * math.sin(angle), 0.0)
            pts.append(p)
        return pts


if __name__ == "__main__":
    try:
        RingMarker()
    except rospy.ROSInterruptException:
        pass
