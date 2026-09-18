"""Fake GO2 for OFF-ROBOT rehearsal of the hardware stages. Not a GO2 model.

Consumes the real unitree_api/msg/Request on /api/sport/request, exactly as
the sink publishes it to the robot, answers on /api/sport/response with
status.code 0 (the documented robot behaviour, GO2_FIELD_NOTES.md section 4),
and publishes the topics HELIX's adapter monitors so HELIX sees a healthy
robot: /utlidar/robot_odom (Odometry, 150 Hz), /utlidar/imu (Imu, 250 Hz),
/utlidar/cloud (empty PointCloud2, 15 Hz), /utlidar/robot_pose (PoseStamped,
20 Hz, rate not measured on the GO2), /gnss and /multiplestate (JSON String,
1 Hz). Odom/IMU/cloud rates follow GO2_FIELD_NOTES.md section 3.

Dynamics (a rehearsal stand-in, deliberately simple and NOT measured):
first-order velocity lag toward the last Move target (tau_s), StopMove sets
the target to zero, and a Move older than move_timeout_s decays to zero.
Api ids other than 1003/1008 are counted and answered with code -1.
Evidence produced against this node is always labelled REHEARSAL.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String


class FakeGo2(Node):

    def __init__(self) -> None:
        super().__init__('fake_go2')
        from unitree_api.msg import Request, Response
        self._Response = Response
        self.declare_parameter('tau_s', 0.15)
        self.declare_parameter('move_timeout_s', 1.0)
        self.declare_parameter('clock_skew_s', 0.0)
        self._tau = self.get_parameter('tau_s').value
        self._move_timeout = self.get_parameter('move_timeout_s').value
        self._skew = self.get_parameter('clock_skew_s').value
        self._target = (0.0, 0.0, 0.0)
        self._target_t = 0.0
        self._v = [0.0, 0.0, 0.0]
        self._pose = [0.0, 0.0, 0.0]
        self._t = time.monotonic()
        self.bad_api = 0
        self.create_subscription(Request, '/api/sport/request', self._on_req, 50)
        self._pub_resp = self.create_publisher(Response, '/api/sport/response', 50)
        self._pub_odom = self.create_publisher(Odometry, '/utlidar/robot_odom', 10)
        self.create_timer(1.0 / 150.0, self._step)
        self._pub_imu = self.create_publisher(Imu, '/utlidar/imu', qos_profile_sensor_data)
        self._pub_cloud = self.create_publisher(PointCloud2, '/utlidar/cloud',
                                                qos_profile_sensor_data)
        self._pub_pose = self.create_publisher(PoseStamped, '/utlidar/robot_pose', 10)
        self._pub_gnss = self.create_publisher(String, '/gnss', 10)
        self._pub_multi = self.create_publisher(String, '/multiplestate', 10)
        self.create_timer(1.0 / 250.0, lambda: self._pub_imu.publish(self._stamped(Imu())))
        self.create_timer(1.0 / 15.0, lambda: self._pub_cloud.publish(self._stamped(PointCloud2())))
        self.create_timer(1.0 / 20.0, self._pose_tick)
        self.create_timer(1.0, self._json_tick)

    def _stamped(self, m):
        stamp = time.time() + self._skew
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp % 1) * 1e9)
        m.header.frame_id = 'base_link'
        return m

    def _pose_tick(self) -> None:
        m = self._stamped(PoseStamped())
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y = self._pose[0], self._pose[1]
        m.pose.orientation.w = 1.0
        self._pub_pose.publish(m)

    def _json_tick(self) -> None:
        self._pub_gnss.publish(String(data=json.dumps(
            {'satellite_total': 0, 'satellite_inuse': 0, 'hdop': 0.0})))
        self._pub_multi.publish(String(data=json.dumps(
            {'volume': 5, 'brightness': 0, 'obstaclesAvoidSwitch': False, 'uwbSwitch': False})))

    def _on_req(self, req) -> None:
        api = req.header.identity.api_id
        code = 0
        if api == 1008:
            p = json.loads(req.parameter)
            self._target = (float(p['x']), float(p['y']), float(p['z']))
            self._target_t = time.monotonic()
        elif api == 1003:
            self._target = (0.0, 0.0, 0.0)
        else:
            self.bad_api += 1
            code = -1
            self.get_logger().error(f'fake_go2: unexpected api_id {api}')
        r = self._Response()
        r.header.identity.id = req.header.identity.id
        r.header.identity.api_id = api
        r.header.status.code = code
        self._pub_resp.publish(r)

    def _step(self) -> None:
        now = time.monotonic()
        dt, self._t = now - self._t, now
        tgt = self._target
        if now - self._target_t > self._move_timeout:
            tgt = (0.0, 0.0, 0.0)
        a = 1.0 - math.exp(-dt / self._tau)
        for i in range(3):
            self._v[i] += a * (tgt[i] - self._v[i])
            if abs(self._v[i]) < 1e-4 and tgt[i] == 0.0:
                self._v[i] = 0.0
        yaw = self._pose[2]
        self._pose[0] += (self._v[0] * math.cos(yaw) - self._v[1] * math.sin(yaw)) * dt
        self._pose[1] += (self._v[0] * math.sin(yaw) + self._v[1] * math.cos(yaw)) * dt
        self._pose[2] += self._v[2] * dt
        m = Odometry()
        stamp = time.time() + self._skew
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp % 1) * 1e9)
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self._pose[0], self._pose[1]
        m.pose.pose.orientation.z = math.sin(self._pose[2] / 2)
        m.pose.pose.orientation.w = math.cos(self._pose[2] / 2)
        m.twist.twist.linear.x, m.twist.twist.linear.y = self._v[0], self._v[1]
        m.twist.twist.angular.z = self._v[2]
        self._pub_odom.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    n = FakeGo2()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
