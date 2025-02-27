#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtle_bot_player_interfaces.srv import FileRead

import os
from ament_index_python.packages import get_package_share_directory


class FilePublisher(Node):
    def __init__(self):
        super().__init__("file_twist_publisher")
        self.publisher = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.service = self.create_service(FileRead, "set_file", self.handle_file_request)
        self.get_logger().info("Nodo iniciado. Esperando solicitud de archivo.")

    def handle_file_request(self, request, response):
        """Handles service calls to read a file and publish velocities."""
        self.get_logger().info("Received service request!")
        if request.input:  # Check if input is received
            self.get_logger().info(f"Received input: {request.input.strip()}")
            filename = request.input.strip()
            success = self.publish_from_file(filename)
            response.result = success
            #response.message = "Archivo leído y mensaje publicado." if success else "Error al leer el archivo."
        else:
            response.result = False
            #response.message = "No file path received."
        return response
        """
        if request.data:
            
            filename = filename = input("Ingrese la ruta completa del archivo (incluya la extensión .txt): ").strip()
            success = self.publish_from_file(filename)
            response.success = success
            response.message = "Archivo leído y mensaje publicado." if success else "Error al leer el archivo."
        else:
            response.success = False
            response.message = "Servicio llamado con 'False'. No se realizó ninguna acción."
        """
    
    def publish_next(self):
        global lines
        global index
        if index >= len(lines):
            self.get_logger().info("Fin del archivo.")
            self.timer.cancel()
            return
        line = lines[index].strip().rstrip(';')  # Eliminar espacios y el `;` final
        values = line.split(',')  # Separar por comas
        linear_x = float(values[0].strip())
        angular_z = float(values[1].strip())

        # Crear y publicar el mensaje Twist
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

        self.get_logger().info(f"Publicado: Linear X = {linear_x}, Angular Z = {angular_z}")

        if len(values) != 2:
            self.get_logger().error("Formato incorrecto. Cada línea debe contener exactamente dos valores separados por coma.")
        index = index + 1
        pass

    def publish_from_file(self, filename):
        """Reads a file and publishes Twist messages."""
        delay = 0.05
        global lines
        try:
            with open(filename, 'r') as file:
                lines = file.readlines()
        
            global index
            index = 0
            self.timer = self.create_timer(delay, self.publish_next)

        except FileNotFoundError:
            self.get_logger().error(f"Archivo no encontrado: {filename}")
        except ValueError:
            self.get_logger().error("Error en el formato del archivo. Asegúrese de que contiene números válidos.")
        except Exception as e:
            self.get_logger().error(f"Error inesperado: {e}")
        return False
        

def main(args=None):
    rclpy.init(args=args)
    node = FilePublisher()
    rclpy.spin(node)  # Keep node running to listen for service calls
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()