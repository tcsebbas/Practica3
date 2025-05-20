import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class Navegar(Node):
    def __init__(self):
        super().__init__('navegar')
        self.publisher_ = self.create_publisher(Point, 'coordenada_objetivo', 10) # Publicador
        self.timer = self.create_timer(4.0, self.timer_callback) # Timer
        
        self.puntos = [
            Point(x=0.5, y=5.5, z=0.0), #Ir sin dibujar al borde izquierdo
            Point(x=10.5, y=5.5, z=1.0), # Dibujar hacia el borde derecho
            Point(x=5.5, y=5.5, z=0.0) # Volver al centro sin dibujar
        ]
        self.index = 0

    def timer_callback(self):
        if self.index >= len(self.puntos):
            return

        punto = self.puntos[self.index] # Publicamos el punto actual como objetivo para la tortuga
        self.publisher_.publish(punto)
        self.get_logger().info(f'Objetivo enviado: x={punto.x}, y={punto.y}, z={punto.z}')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    nodo = Navegar()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
