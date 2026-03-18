import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk
import threading

# --- ROS 2 NODE ---
class DashboardNode(Node):
    def __init__(self, gui_app):
        super().__init__('dashboard_node')
        self.gui_app = gui_app
        
        # Listen to the camera safety topic you made
        self.create_subscription(
            Bool, 
            '/camera_stop_signal', 
            self.camera_callback, 
            10
        )

    def camera_callback(self, msg):
        # Update the GUI based on YOLO's output
        if msg.data:
            self.gui_app.update_camera_status("PERSON DETECTED!\nBRAKES ENGAGED", "red")
        else:
            self.gui_app.update_camera_status("PATH CLEAR", "green")

# --- TKINTER GUI ---
class KAU_Dashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("KAU Autonomous Golf Cart - Dashboard")
        self.root.geometry("400x250")
        self.root.configure(bg="#2b2b2b")

        # Header
        title = tk.Label(root, text="Safety System Status", fg="white", bg="#2b2b2b", font=("Arial", 16, "bold"))
        title.pack(pady=10)

        # Status Box
        self.status_label = tk.Label(root, text="WAITING FOR SENSORS...", bg="gray", fg="white", font=("Arial", 18, "bold"), width=25, height=4)
        self.status_label.pack(pady=20)

    def update_camera_status(self, text, color):
        # Tkinter is thread-safe enough for simple label config updates
        self.status_label.config(text=text, bg=color)

# --- MAIN EXECUTION ---
def ros_spin_thread(node):
    # ROS needs its own thread so it doesn't freeze the GUI
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    
    # Start the GUI
    root = tk.Tk()
    app = KAU_Dashboard(root)
    
    # Start the ROS Node
    node = DashboardNode(app)
    
    # Run ROS in the background
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()
    
    # Run the GUI loop in the main thread
    root.mainloop()
    
    # Cleanup when the window is closed
    rclpy.shutdown()

if __name__ == '__main__':
    main()