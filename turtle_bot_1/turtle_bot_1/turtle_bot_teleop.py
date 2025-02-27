#!usr/bin/env python3
import threading
import rclpy 
import os 
import select
import sys, termios, tty
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import signal

class TurtleBotTeleop(Node):
	def __init__(self):
		super().__init__("turtle_bot_teleop")
		self.get_logger().info("Inicializar en dev")
		self.file_sub = self.create_subscription(String, "/save_file_name", self.save_file_callback, 10)
		self.cmd_vel_pub = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
		self.settings =  termios.tcgetattr(sys.stdin)
		self.name_file = None
		self.linVel = float(input("Ingrese la velocidad lineal: "))
		self.angVel = float(input("Ingrese la velocidad angular: "))
  
		self.file_lock = threading.Lock()
  
		self.timer = self.create_timer(0.1, self.read_keyboard)
		
	def save_file_callback(self,msg:String):
		self.name_file = msg.data
		self.get_logger().info(f"Archivo de guardado configurado: {self.name_file}")
  
	def read_keyboard(self):
		""" Lee la tecla presionada y envía el comando de velocidad """
		key = self.getKey()
		if key == '\x03':
			raise KeyboardInterrupt
		if key != '':
			self.get_logger().info(f"Tecla presionada: {key}")
		self.send_velocity_command(key)
  
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist:
			key = sys.stdin.read(1)
			self.get_logger().info("holi")
	
		else:
			key = ''
		
		
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)	
		return key

	
	def send_velocity_command(self, key):
		msg = Twist()
		if key == 'a':
			msg.linear.x = 0.0
			msg.angular.z = self.angVel
		elif key == 'd':
			msg.linear.x = 0.0
			msg.angular.z = -1.0 * self.angVel
		elif key == 'w':
			msg.linear.x = self.linVel
			
		elif key == 's':
			msg.linear.x = -1.0 * self.linVel
		else:
			msg.linear.x = 0.0
			msg.angular.z = 0.0
		
		self.cmd_vel_pub.publish(msg)
  
		if self.name_file:
			threading.Thread(target=self.write_to_file, args=(msg,)).start()
  
	def write_to_file(self, msg):
		with self.file_lock:  # Sincronizar el acceso al archivo
			try:
				with open(self.name_file, "a") as f:
					f.write(f"{msg.linear.x},{msg.angular.z};\n")
					f.flush()  # Forzar la escritura en el disco
			except Exception as e:
				self.get_logger().error(f"Error al escribir en el archivo: {e}")
  	 
def main(args=None):
	rclpy.init(args=args)
	node = TurtleBotTeleop()
 
	def signal_handler(sig, frame):
		node.get_logger().info("Señal SIGINT recibida. Cerrando el nodo...")
		rclpy.shutdown()

	signal.signal(signal.SIGINT, signal_handler)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("Nodo detenido por el usuario.")
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()