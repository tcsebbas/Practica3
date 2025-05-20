import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen, Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class DibujoNode(Node):
    def __init__(self):
        super().__init__('dibujar')
        
        # Servicio para cambiar el color de la pluma
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        
        # Servicio para crear la tortuga
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
        # Esperamos que los servicios estén listos
        self.set_pen_client.wait_for_service()
        self.spawn_client.wait_for_service()

        self.dibujar()

    def dibujar(self):
        # Cambiar el color de la pluma a verde para el césped
        request = SetPen.Request()
        request.r = 0
        request.g = 255
        request.b = 0
        request.width = 10
        request.off = 0
        self.set_pen_client.call_async(request)
        
        # Crear la tortuga en una posición inicial
        spawn_request = Spawn.Request()
        spawn_request.x = 5.0
        spawn_request.y = 5.0
        spawn_request.theta = 0.0
        spawn_request.name = 'turtle1'
        self.spawn_client.call_async(spawn_request)

def main(args=None):
    rclpy.init(args=args)
    node = DibujoNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

