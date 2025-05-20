
# practica3/dibujar.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
import math

class Dibujar(Node):
    def __init__(self):
        super().__init__('dibujar')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.cb_pose, 10)
        self.coord_sub = self.create_subscription(Point, 'coordenada_objetivo', self.cb_coord, 10)
        self.timer = self.create_timer(0.1, self.cb_timer)

        self.pose = None
        self.x_obj = None
        self.y_obj = None
        self.dibujando = False
        self.pen_down = False

    def cb_pose(self, msg):
        self.pose = msg

    def cb_coord(self, msg):
        self.x_obj = msg.x
        self.y_obj = msg.y
        self.pen_down = (msg.z == 1.0)
        self.dibujando = True

        if self.pen_down:
            self.set_pen(0, 255, 0, 3, 0)  # Verde y ON
        else:
            self.set_pen(0, 0, 0, 3, 1)    # OFF (levanta lápiz)

    def set_pen(self, r, g, b, width=3, off=0):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /turtle1/set_pen...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        client.call_async(req)

    def cb_timer(self):
        if self.pose is None or not self.dibujando:
            return

        dx = self.x_obj - self.pose.x
        dy = self.y_obj - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        twist = Twist()
        if distance > 0.05:
            angle_to_goal = math.atan2(dy, dx)
            angle_diff = angle_to_goal - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) > 0.1:
                twist.angular.z = 2.0 * angle_diff
            else:
                twist.linear.x = 2.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.dibujando = False

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = Dibujar()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



