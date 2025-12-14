#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def quat_to_R(q):
    x, y, z, w = q
    # 标准四元数转旋转矩阵
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ], dtype=float)
    return R

def R_to_quat(R):
    # 旋转矩阵转四元数（返回 x,y,z,w）
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
    return np.array([x,y,z,w], dtype=float)

def T_from(t, q):
    R = quat_to_R(q)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = np.array(t, dtype=float)
    return T

def T_inv(T):
    R = T[:3,:3]
    p = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti

class Calc(Node):
    def __init__(self):
        super().__init__('calc_body_to_base')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 你测量的 base_link -> livox_frame
        self.base_to_livox_t = (0.645, 0.0, 0.525)
        self.base_to_livox_q = (0.0, 0.0, 0.0, 1.0)

        self.timer = self.create_timer(1.0, self.once)

    def once(self):
        try:
            # 取 lidar -> body（来自 fast-lio TF 树）
            tf = self.tf_buffer.lookup_transform('lidar', 'body', rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            T_lidar_body = T_from((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

            # 假设 fast-lio 的 lidar 坐标系就是“雷达原点坐标系”
            # 那 base_link -> lidar 就等于你测量的 base_link -> livox_frame（轴向一致时成立）
            T_base_lidar = T_from(self.base_to_livox_t, self.base_to_livox_q)

            # body -> base = inv(lidar->body) * inv(base->lidar)
            T_body_base = T_inv(T_lidar_body) @ T_inv(T_base_lidar)

            t_out = T_body_base[:3, 3]
            q_out = R_to_quat(T_body_base[:3,:3])

            print("\n=== body -> base_link 静态TF（用四元数方式发布）===")
            print("ros2 run tf2_ros static_transform_publisher "
                  f"{t_out[0]:.6f} {t_out[1]:.6f} {t_out[2]:.6f} "
                  f"{q_out[0]:.6f} {q_out[1]:.6f} {q_out[2]:.6f} {q_out[3]:.6f} "
                  "body base_link")
            print("==============================================\n")

            rclpy.shutdown()
        except Exception as e:
            # TF 还没准备好就再等 1 秒
            pass

def main():
    rclpy.init()
    node = Calc()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
