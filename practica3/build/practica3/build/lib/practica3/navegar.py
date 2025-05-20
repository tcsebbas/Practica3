import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class NavegarNode(Node):
    def __init__(self):
        super().__init__('navegar')
        # Suscripción al topic de la pose de la tortuga
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # Publicador al topic de velocidad
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = Pose()

    def pose_callback(self, msg):
        self.pose = msg
        # Movemos la tortuga a una nueva posición (ejemplo: avanzar 2 unidades hacia adelante)
        self.mover_tortuga()

    def mover_tortuga(self):
        # Crear un mensaje de velocidad para mover la tortuga
        twist = Twist()
        twist.linear.x = 2.0  # Avanzar en línea recta
        twist.angular.z = 0.0  # Sin rotación
        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavegarNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

