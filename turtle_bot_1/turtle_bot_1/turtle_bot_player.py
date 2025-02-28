#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtle_bot_player_interfaces.srv import FileRead
import os
import time
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

class FilePublisher(Node):
    def __init__(self):
        super().__init__("file_twist_publisher")
        self.publisher = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.status_publisher = self.create_publisher(String, "/route_status", 10)
        self.service = self.create_service(FileRead, "set_file", self.handle_file_request)
        self.get_logger().info("Nodo iniciado. Esperando solicitud de archivo.")
        
        # Variables para el control de la reproducción
        self.lines = []
        self.index = 0
        self.timer = None
        self.last_timestamp = None
        
    def handle_file_request(self, request, response):
        """Handles service calls to read a file and publish velocities."""
        self.get_logger().info("Received service request!")
        
        if request.input:  # Check if input is received
            self.get_logger().info(f"Received input: {request.input.strip()}")
            filename = request.input.strip()
            success = self.publish_from_file(filename)
            response.result = success
        else:
            self.get_logger().info("No se recibió ruta de archivo.")
            response.result = False
            
        return response
    
    def publish_next(self):
        """Publica el siguiente comando de velocidad desde el archivo."""
        if self.index >= len(self.lines):
            self.get_logger().info("Fin del archivo.")
            # Enviar comando de parada para asegurar que el robot se detiene
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            self.get_logger().info("Robot detenido")
            
            # Publicar mensaje de finalización
            status_msg = String()
            status_msg.data = "COMPLETED"
            self.status_publisher.publish(status_msg)
            self.get_logger().info("Ruta completada - Notificación enviada")
            
            # Cancelar el timer
            if self.timer:
                self.timer.cancel()
                self.timer = None
            return
        
        # Procesar la línea actual
        try:
            line = self.lines[self.index].strip().rstrip(';')  # Eliminar espacios y el `;` final
            values = line.split(',')  # Separar por comas
            
            if len(values) != 3:
                self.get_logger().error(f"Formato incorrecto en línea {self.index+1}. Cada línea debe contener exactamente tres valores separados por coma.")
                self.index += 1
                return
                
            linear_x = float(values[0].strip())
            angular_z = float(values[1].strip())
            timestamp = float(values[2].strip())
            
            # Crear y publicar el mensaje Twist
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.publisher.publish(msg)
            
            self.get_logger().info(f"Publicado: Linear X = {linear_x}, Angular Z = {angular_z}, Timestamp = {timestamp}")
            
            # Calcular el tiempo hasta el próximo comando
            self.index += 1
            if self.index < len(self.lines):
                next_line = self.lines[self.index].strip().rstrip(';')
                next_values = next_line.split(',')
                if len(next_values) == 3:
                    next_timestamp = float(next_values[2].strip())
                    # Calcular el tiempo a esperar entre este comando y el siguiente
                    wait_time = next_timestamp - timestamp
                    # Asegurar un tiempo mínimo entre comandos
                    wait_time = max(wait_time, 0.05)
                    
                    # Cancelar el timer actual y crear uno nuevo con el tiempo correcto
                    if self.timer:
                        self.timer.cancel()
                    self.timer = self.create_timer(wait_time, self.publish_next)
                    return
            
            # Si llegamos aquí, es el último comando o hubo un error
            if self.timer:
                self.timer.cancel()
                # Llamamos a publish_next nuevamente para activar la condición de finalización
                self.timer = self.create_timer(0.5, self.publish_next)
                
        except ValueError as e:
            self.get_logger().error(f"Error al procesar la línea {self.index+1}: {e}")
            self.index += 1
            
    def publish_from_file(self, filename):
        """Reads a file and publishes Twist messages."""
        try:
            # Publicar mensaje de inicio
            status_msg = String()
            status_msg.data = "STARTED"
            self.status_publisher.publish(status_msg)
            self.get_logger().info("Inicio de reproducción de ruta")
            
            with open(filename, 'r') as file:
                self.lines = file.readlines()
                
            if not self.lines:
                self.get_logger().error("El archivo está vacío")
                
                # Publicar mensaje de error
                status_msg = String()
                status_msg.data = "ERROR"
                self.status_publisher.publish(status_msg)
                
                return False
                
            self.get_logger().info(f"Archivo cargado: {filename} con {len(self.lines)} líneas")
            
            # Reiniciar variables
            self.index = 0
            if self.timer:
                self.timer.cancel()
                
            # Iniciar la reproducción inmediatamente
            self.timer = self.create_timer(0.1, self.publish_next)
            return True
            
        except FileNotFoundError:
            self.get_logger().error(f"Archivo no encontrado: {filename}")
            
            # Publicar mensaje de error
            status_msg = String()
            status_msg.data = "ERROR"
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error inesperado: {e}")
            
            # Publicar mensaje de error
            status_msg = String()
            status_msg.data = "ERROR"
            self.status_publisher.publish(status_msg)
            
        return False

def main(args=None):
    rclpy.init(args=args)
    node = FilePublisher()
    rclpy.spin(node)  # Keep node running to listen for service calls
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()