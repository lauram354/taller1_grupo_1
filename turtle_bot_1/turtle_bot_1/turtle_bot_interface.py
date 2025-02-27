#!usr/bin/env python3
import rclpy 
from rclpy.node import Node
from tkinter import NW, filedialog, ttk, Label, PhotoImage, messagebox, BooleanVar
import tkinter as tk
from geometry_msgs.msg import Twist
import threading
from PIL import Image
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from turtle_bot_player_interfaces.srv import FileRead

class TurtleBotInterface(Node):
	def __init__(self, mainframe):
		super().__init__("turtle_bot_interface")
		self.get_logger().info("Hello Interface dev")

		self.position_subscriber = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)
		self.file_pub = self.create_publisher(String, "/save_file_name", 10)
		self.prev_x = 250
		self.prev_y = 250
		self.name_file = None
		self.mainframe = mainframe
  
	def set_file_name(self, file_name):
		self.name_file = file_name
		if file_name:
			msg = String()
			msg.data = file_name
			self.file_pub.publish(msg)
			self.get_logger().info(f"Saving positions to {file_name}")
		else:
			self.get_logger().info("Not saving positions")
		
	def position_callback(self, msg: Twist):
		x_og = round(float(msg.linear.x),  2)
		y_og = round(float(msg.linear.y),  2)
		self.get_logger().info(str(x_og) + "  " + str(y_og))
		x = 100*x_og + 250
		y = -100*y_og + 250
		if (x != self.prev_x) or  (y != self.prev_y):
			try:
				self.mainframe.drawing.create_line(self.prev_x, self.prev_y, x, y, fill="#60992D", width=2)
				self.mainframe.drawing.moveto(self.mainframe.turtle, x-10, y-10)
				self.prev_x = x
				self.prev_y = y
			except Exception as err:
				print(f"Error {err}")
		if self.name_file:
			try:
				with open (self.name_file,"a") as file:
					file.write(f"{msg.linear.x},{msg.angular.z},{x},{y};\n")
				self.get_logger().info(f"Saved position to {self.file_name}")
			except Exception as e:
				self.get_logger().error(f"Failed to write to file: {e}")
class Interface(ttk.Frame):
	def __init__(self, root=None):
		tk.Frame.__init__(self, master=root, bg="#D499B9")
		self.pack(side='top', fill='both', expand=True)
		self.img_path = os.path.join(get_package_share_directory('turtle_bot_1'), "resource", "turtle.png")
		self.drawing = None
		self.turtle = None
		self.img = None
		self.name = None
		self.node = None
		self.createInterface()
	

	def createInterface(self):
		# Frame superior con entrada de texto y botón
		top = tk.Frame(self, bg="#D499B9")
		top.pack(side=tk.TOP, padx=5, pady=10, fill='both')

		self.name = tk.StringVar()

		title = Label(top, text="Turtle Bot Interface",background="#D499B9", font=("Cambria", 18), fg = "#08415C")
		title.pack(side=tk.TOP, expand=True)
  
		fileName = Label(top, text="Nombre del archivo: ", background="#D499B9", font=("Cambria", 12), fg = "#08415C")
		fileName.pack(side=tk.LEFT, expand=True)

		name_entry = ttk.Entry(top, textvariable = self.name, width=20)
		name_entry.pack(side=tk.LEFT, expand=True)
  
		save_check = ttk.Button(top, text="Guardar recorrido", command=self.confirm_save).pack(side=tk.RIGHT, padx=5)

		activate_service_btn = ttk.Button(top, text="Servicio", command=self.activate_service).pack(side=tk.RIGHT, padx=5)
		save_button = ttk.Button(top, text="Guardar img", command=self.save)
		save_button.pack(side=tk.RIGHT, padx=5)
		# Frame principal con Canvas
		self.mainframe = ttk.Frame(self)
		self.mainframe.pack(side=tk.BOTTOM, expand=True)

		self.drawing = tk.Canvas(self.mainframe, width=500, height=500, bg="#DBEEFA")
		self.img = PhotoImage(file=self.img_path).subsample(10, 10)
		self.turtle = self.drawing.create_image(250, 250, anchor=NW, image= self.img)
		self.drawing.pack()
  
	def confirm_save(self):
		confirm = messagebox.askyesno("Guardar Recorrido", "¿Desea guardar el recorrido?")
		if confirm:
			file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Archivos de texto", "*.txt")],
            title="Guardar como"
        )
		if file_path:
			# Crear la carpeta si no existe
			os.makedirs(os.path.dirname(file_path), exist_ok=True)
			# Publicar la ruta del archivo
			with open(file_path, "w") as file:
				pass
			msg = String()
			msg.data = file_path
			self.node.file_pub.publish(msg)
			messagebox.showinfo("Guardado", f"El archivo se guardará en: {file_path}")
                
	def save(self):
		if self.name.get():
			file = self.name.get().strip()
		else:
			file = "TurtleBotRoute"
	
		filename = file + ".eps"
		self.drawing.postscript(file=filename)
		img = Image.open(filename)
		img.save((file+".png"), "png")
		os.remove(filename)

	def activate_service(self):
		root = tk.Tk()
		root.withdraw()
		filename = tk.filedialog.askopenfilename(title="Seleccione un archivo",
                               filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
		if not filename:
			print("Selección de archivo cancelada.")
			return
		if not rclpy.ok():
			rclpy.init()
		node = rclpy.create_node("interface_service_caller")
		client = node.create_client(FileRead, "/set_file")
		# Wait for the service to be available
		
		if not client.wait_for_service(timeout_sec=5.0):
			print("El servicio no está disponible.")
			node.destroy_node()
			return
		# Create and send the request
		
		request = FileRead.Request()
		request.input = filename  # Set the file path in the request
		future = client.call_async(request)
		# Wait for the result
		rclpy.spin_until_future_complete(node, future)
		# Process the response
		if future.result() is not None:
			print(f"Respuesta del servicio: {future.result().result}")
		else:
			print("No se recibió respuesta del servicio.")

		return None
   
def main(args=None):
	rclpy.init(args=args)
	
	root = tk.Tk()
	root.title("Turtlebot2")
	root.geometry("700x600")
	root.configure(bg="#D499B9")
	mainframe = Interface(root=root)
	node = TurtleBotInterface(mainframe)
	mainframe.node = node
	node = TurtleBotInterface(mainframe)	
	thread_spin = threading.Thread(target=rclpy.spin, args=(node, ))
	thread_spin.start()
	root.mainloop()

	node.destroy_node()
	rclpy.shutdown()
	thread_spin.join()

if __name__ == '__main__':
	main()
