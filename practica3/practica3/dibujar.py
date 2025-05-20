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
        self.pen_down = False
        self.dibujando = False

        # Para controlar la secuencia
        self.estado = "MOVIENDO_A_PUNTO"  # estados: MOVIENDO_A_PUNTO, DIBUJANDO_CIRCULO, TERMINADO
        self.circulo_angle = 0.0

    def cb_pose(self, msg):
        self.pose = msg

    def cb_coord(self, msg):
        # Sólo actualizamos objetivo y estado si estamos moviendo a punto
        if self.estado == "MOVIENDO_A_PUNTO":
            self.x_obj = msg.x
            self.y_obj = msg.y
            self.pen_down = (msg.z == 1.0)
            self.dibujando = True

            if self.pen_down:
                self.set_pen(0, 255, 0, 3, 0)  # verde y pen down
            else:
                self.set_pen(0, 0, 0, 3, 1)    # pen up

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
        if self.pose is None:
            return

        twist = Twist()

        if self.estado == "MOVIENDO_A_PUNTO":
            if not self.dibujando:
                return

            dx = self.x_obj - self.pose.x
            dy = self.y_obj - self.pose.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance > 0.05:
                angle_to_goal = math.atan2(dy, dx)
                angle_diff = angle_to_goal - self.pose.theta
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

                if abs(angle_diff) > 0.1:
                    twist.angular.z = 2.0 * angle_diff
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = 2.0
                    twist.angular.z = 0.0
            else:
                # Llegó al punto objetivo
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.dibujando = False

                # Si el punto actual es el centro con pen_up => empezamos a dibujar círculo blanco
                if abs(self.x_obj - 5.5) < 0.1 and abs(self.y_obj - 5.5) < 0.1 and not self.pen_down:
                    self.estado = "DIBUJANDO_CIRCULO"
                    self.circulo_angle = 0.0
                    self.set_pen(255, 255, 255, 3, 0)  # blanco y pen down

            self.cmd_pub.publish(twist)

        elif self.estado == "DIBUJANDO_CIRCULO":
            # Dibuja un círculo girando a velocidad constante
            velocidad_lineal = 1.5  # velocidad hacia adelante
            velocidad_angular = 1.5  # velocidad angular

            twist.linear.x = velocidad_lineal
            twist.angular.z = velocidad_angular
            self.cmd_pub.publish(twist)

            # Incrementar el ángulo estimado del círculo
            self.circulo_angle += velocidad_angular * 0.1  # 0.1s es el periodo del timer

            # Cuando completa un círculo (2*pi radianes), termina
            if self.circulo_angle >= 2 * math.pi:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.estado = "TERMINADO"
                self.set_pen(0, 0, 0, 3, 1)  # levantar lápiz
                self.get_logger().info("Dibujo completado.")

        elif self.estado == "TERMINADO":
            # No hace nada, tortuga queda quieta
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = Dibujar()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
