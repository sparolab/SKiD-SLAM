#!/usr/bin/env python3
"""Cinematic RViz camera for SKiD-SLAM videos.

Drives the `rviz_animated_view_controller/Animated` view through /rviz/camera_placement:
  * follow : slowly orbits the robots and zooms out as they spread apart.
  * finale : (set ~finale to true) pulls back to frame every place the robots have visited
             and orbits the fused map. Runs on wall time, so it also works after the bag ends.
"""
import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Vector3
from view_controller_msgs.msg import CameraPlacement


def main():
    rospy.init_node("skid_slam_camera_orbit")
    robots = rospy.get_param("~robots", ["jackal0", "jackal1", "jackal2"])
    frame = rospy.get_param("~frame", "map")
    base = rospy.get_param("~base_frame", "base_link")
    pitch = rospy.get_param("~pitch", 0.85)                # rad above the horizon
    yaw = rospy.get_param("~yaw", -math.pi / 2)
    yaw_rate = rospy.get_param("~yaw_rate", 0.03)          # rad per second of bag time
    finale_yaw_rate = rospy.get_param("~finale_yaw_rate", 0.12)  # rad per wall second
    finale_pitch = rospy.get_param("~finale_pitch", 1.0)
    min_dist = rospy.get_param("~min_distance", 25.0)      # m
    max_dist = rospy.get_param("~max_distance", 150.0)
    zoom = rospy.get_param("~zoom", 1.8)                   # distance = zoom * spread + min_distance
    smooth = rospy.get_param("~smoothing", 0.03)           # low-pass factor per frame

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size=1)

    focus, dist, cur_pitch = None, min_dist, pitch
    visited_min, visited_max = [math.inf] * 3, [-math.inf] * 3
    last_sim, last_wall = None, time.time()
    while not rospy.is_shutdown():
        pts = []
        for r in robots:
            try:
                t = tf_buffer.lookup_transform(frame, "%s/%s" % (r, base), rospy.Time(0)).transform.translation
                pts.append((t.x, t.y, t.z))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

        wall = time.time()
        dt_wall, last_wall = wall - last_wall, wall
        finale = rospy.get_param("~finale", False)

        if pts:
            for p in pts:
                visited_min = [min(a, b) for a, b in zip(visited_min, p)]
                visited_max = [max(a, b) for a, b in zip(visited_max, p)]

            if finale:
                c = [(a + b) / 2 for a, b in zip(visited_min, visited_max)]
                extent = max(b - a for a, b in zip(visited_min[:2], visited_max[:2]))
                target_dist = max(1.3 * extent + 2 * min_dist, min_dist)
                target_pitch = finale_pitch
                yaw += finale_yaw_rate * dt_wall
                k = 1 - (1 - smooth) ** (dt_wall * 30)  # frame-rate independent smoothing
            else:
                c = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
                spread = max(math.dist(p, c) for p in pts)
                target_dist = min(zoom * spread + min_dist, max_dist)
                target_pitch = pitch
                now = rospy.Time.now().to_sec()
                if last_sim is not None and now > last_sim:
                    yaw += yaw_rate * min(now - last_sim, 1.0)
                last_sim = now
                k = smooth

            if focus is None:
                focus, dist = c, target_dist
            focus = [f + k * (ci - f) for f, ci in zip(focus, c)]
            dist += k * (target_dist - dist)
            cur_pitch += k * (target_pitch - cur_pitch)

            msg = CameraPlacement()
            msg.target_frame = frame
            msg.interpolation_mode = CameraPlacement.LINEAR
            msg.time_from_start = rospy.Duration(0)
            msg.focus.header.frame_id = frame
            msg.focus.point = Point(*focus)
            msg.eye.header.frame_id = frame
            msg.eye.point = Point(focus[0] + dist * math.cos(cur_pitch) * math.cos(yaw),
                                  focus[1] + dist * math.cos(cur_pitch) * math.sin(yaw),
                                  focus[2] + dist * math.sin(cur_pitch))
            msg.up.header.frame_id = frame
            msg.up.vector = Vector3(0, 0, 1)
            pub.publish(msg)

        time.sleep(1.0 / 30)


if __name__ == "__main__":
    main()
