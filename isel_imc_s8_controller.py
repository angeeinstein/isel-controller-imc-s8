import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import sys
import subprocess
import threading
import queue
import time
import socket
import json
import os
import csv
from datetime import datetime

# --- Auto-Install Dependencies ---
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    # If pyserial is missing, spawn a temporary hidden window to show a message
    root = tk.Tk()
    root.withdraw()
    messagebox.showinfo("Initial Setup", "The required package 'pyserial' is not installed.\n\nThe application will now attempt to download and install it automatically. This might take a few seconds...")
    
    try:
        # sys.executable ensures we use the exact pip associated with the current Python environment
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
        messagebox.showinfo("Setup Complete", "Successfully installed 'pyserial'. The application will now start.")
    except Exception as e:
        messagebox.showerror("Setup Failed", f"Could not install 'pyserial' automatically.\n\nPlease open your command prompt/terminal and run:\npip install pyserial\n\nError: {e}")
        sys.exit(1)
        
    root.destroy()
    
    # Import again after successful installation
    import serial
    import serial.tools.list_ports

# --- Constants and Error Codes ---
ERROR_CODES = {
    '1': "Invalid number parameter",
    '2': "Limit switch hit (Requires Init & Home)",
    '3': "Invalid axis specification",
    '4': "No axes defined (Run Init first)",
    '5': "Syntax error",
    '6': "Memory full",
    '7': "Invalid parameter count",
    '8': "Invalid command",
    '9': "System error (Estop, Drive offline, Hood open)",
    'D': "Invalid speed",
    'F': "User stop",
    'G': "Invalid data field",
    'H': "Hood error",
    'R': "Reference error (Requires Homing)"
}

class IselController:
    """
    Handles serial communication with the isel iMC-S8 controller.
    Ensures that commands are sent sequentially and waits for hardware handshake.
    """
    def __init__(self):
        self.serial_port = None
        self.cmd_queue = queue.Queue()
        self.is_running = False
        self.is_moving = False
        self.current_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.status_msg = "Disconnected"
        self.worker_thread = None
        
        # Grid Automation State Variables
        self.grid_running = False
        self.grid_points = []
        self.grid_point_index = 0
        self.grid_wait_time = 0.0
        self.grid_speed = 0.0
        self.grid_next_move_time = 0.0
        self.grid_waiting_for_move_to_finish = False
        self.grid_progress_callback = None
        
        self.log_callback = None
        
        # Default settings
        self.steps_per_mm = {"x": 319.8529, "y": 319.8529, "z": 319.8529}
        self.axis_limits = {"x": 500.0, "y": 500.0, "z": 500.0}
        
        # Save to user's home directory to avoid Windows permission issues
        self.settings_file = os.path.join(os.path.expanduser("~"), ".isel_settings.json")
        self.load_settings()

    def load_settings(self):
        if os.path.exists(self.settings_file):
            try:
                with open(self.settings_file, "r") as f:
                    data = json.load(f)
                    if "steps_per_mm" in data:
                        self.steps_per_mm = data["steps_per_mm"]
                    if "axis_limits" in data:
                        self.axis_limits = data["axis_limits"]
            except Exception as e:
                print(f"Could not load settings: {e}")

    def save_settings(self):
        """Saves current settings to disk. Returns (success_bool, error_message)."""
        try:
            data = {
                "steps_per_mm": self.steps_per_mm,
                "axis_limits": self.axis_limits
            }
            with open(self.settings_file, "w") as f:
                json.dump(data, f)
            return True, ""
        except Exception as e:
            err_msg = str(e)
            print(f"Could not save settings: {err_msg}")
            return False, err_msg

    def connect(self, port_name, baudrate=19200):
        try:
            self.serial_port = serial.Serial(port_name, baudrate, timeout=0.1)
            self.is_running = True
            self.worker_thread = threading.Thread(target=self._serial_worker, daemon=True)
            self.worker_thread.start()
            self.status_msg = f"Connected to {port_name}"
            return True
        except Exception as e:
            self.status_msg = f"Connection error: {str(e)}"
            return False

    def disconnect(self):
        self.is_running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.status_msg = "Disconnected"

    def enqueue_command(self, cmd_string, expect_data_length=0, callback=None):
        """Adds a command to the queue to be processed by the worker thread."""
        if not self.is_running:
            self.log_event("ERROR", "Command ignored: Not connected to controller")
            if callback:
                try:
                    callback(False, None, "Not connected to controller")
                except Exception as e:
                    print(f"Callback error: {e}")
            return
        self.cmd_queue.put((cmd_string, expect_data_length, callback))

    def log_event(self, event_type, message):
        """Logs an event with a precise timestamp and current position, triggering the GUI callback."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        x = self.current_pos.get('x', 0.0)
        y = self.current_pos.get('y', 0.0)
        z = self.current_pos.get('z', 0.0)
        
        if self.log_callback:
            self.log_callback(timestamp, event_type, message, x, y, z)

    def check_limits(self, points):
        """
        Checks a list of points (dicts with x, y, z) against soft limits.
        Returns (is_safe: bool, warning_msg: str)
        """
        warnings = []
        min_vals = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
        max_vals = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
        
        for pt in points:
            for axis in ['x', 'y', 'z']:
                if axis in pt:
                    min_vals[axis] = min(min_vals[axis], pt[axis])
                    max_vals[axis] = max(max_vals[axis], pt[axis])
                    
        for axis in ['x', 'y', 'z']:
            if min_vals[axis] != float('inf') and max_vals[axis] != float('-inf'):
                if min_vals[axis] < 0.0 or max_vals[axis] > self.axis_limits[axis]:
                    warnings.append(f"{axis.upper()}-Axis target [{min_vals[axis]:.2f} to {max_vals[axis]:.2f}] is outside limits [0.00 to {self.axis_limits[axis]:.2f}]")
                    
        if warnings:
            return False, "Movement exceeds soft axis limits:\n" + "\n".join(warnings)
        return True, ""

    def _serial_worker(self):
        """Background thread that talks to the serial port."""
        while self.is_running:
            try:
                # Try to get a command from the queue
                cmd, expect_data_length, callback = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if not self.serial_port or not self.serial_port.is_open:
                # Need to clear task to prevent queue lockup if disconnected mid-task
                self.cmd_queue.task_done()
                continue

            # Clear input buffer before sending new command
            self.serial_port.reset_input_buffer()
            
            # Send command
            encoded_cmd = cmd.encode('ascii')
            self.serial_port.write(encoded_cmd)

            # Mark state as moving if it's a movement command
            if "A" in cmd or "M" in cmd or "R" in cmd:
                self.is_moving = True

            # Wait for handshake response
            response_char = None
            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    response_char = self.serial_port.read(1).decode('ascii')
                    break
                time.sleep(0.01)

            if not response_char:
                self.cmd_queue.task_done()
                continue # Thread stopping

            success = False
            data = None
            err_msg = ""

            if response_char == '0':
                # Success
                success = True
                if expect_data_length > 0:
                    # e.g., position command '@0P' returns '0' followed by 18 chars
                    while self.is_running and self.serial_port.in_waiting < expect_data_length:
                        time.sleep(0.01)
                    if self.is_running:
                        data = self.serial_port.read(expect_data_length).decode('ascii')
            else:
                # Error code received
                err_msg = ERROR_CODES.get(response_char, f"Unknown error code: {response_char}")
                self.status_msg = f"Controller Error: {err_msg}"
                self.log_event("ERROR", f"Hardware Error: {err_msg} (Cmd: {cmd.strip()})")
                print(f"HW ERROR: {err_msg} (Cmd: {cmd.strip()})")

            self.is_moving = False

            if callback:
                # Execute callback in a try-except to prevent crashing the worker thread
                try:
                    callback(success, data, err_msg)
                except Exception as e:
                    print(f"Callback error: {e}")
            
            self.cmd_queue.task_done()

    # --- High Level Commands ---

    def init_axes(self, axes_num=7):
        """Initializes axes and immediately forces 3D positioning mode."""
        self.log_event("CMD", "Initializing axes")
        self.status_msg = "Initializing axes..."
        def callback(success, data, err):
            if success:
                # Set 3D mode automatically
                self.set_3d_mode(True)
                self.status_msg = "Axes initialized (3D mode active)."
        self.enqueue_command(f"@0{axes_num}\r", callback=callback)

    def home_axes(self, axes_num=7):
        """Runs reference search (Homing)."""
        self.log_event("MOVE", "Homing axes")
        self.status_msg = "Homing axes (waiting for machine to finish)..."
        def callback(success, data, err):
            if success:
                self.status_msg = "Homing complete."
                # Force an immediate position fetch so zeros appear instantly
                self.get_position()
        self.enqueue_command(f"@0R{axes_num}\r", callback=callback)

    def set_homing_speed(self, speed_x, speed_y, speed_z):
        """Sets the reference/homing speed for the axes (Input in mm/s)."""
        self.log_event("CMD", f"Setting homing speed to {speed_x} mm/s")
        self.status_msg = f"Setting homing speed to {speed_x} mm/s..."
        sx = int(speed_x * self.steps_per_mm['x'])
        sy = int(speed_y * self.steps_per_mm['y'])
        sz = int(speed_z * self.steps_per_mm['z'])
        cmd = f"@0d{sx},{sy},{sz}\r"
        self.enqueue_command(cmd)

    def set_acceleration(self, accel):
        """Sets the acceleration (Input in mm/s^2). Converts to Hz/ms."""
        self.log_event("CMD", f"Setting acceleration to {accel} mm/s²")
        self.status_msg = f"Setting acceleration to {accel} mm/s²..."
        # 1 Hz/ms = 1000 steps/s^2.
        accel_hz_ms = int((accel * self.steps_per_mm['x']) / 1000.0)
        # Enforce controller limits
        accel_hz_ms = max(1, min(4000, accel_hz_ms))
        self.enqueue_command(f"@0J{accel_hz_ms}\r")

    def set_3d_mode(self, enable=True):
        """Enables 3D interpolation (Simultaneous X,Y,Z)."""
        self.log_event("CMD", f"Setting 3D mode: {'ON' if enable else 'OFF'}")
        mode = 1 if enable else 0
        self.enqueue_command(f"@0z{mode}\r")

    def get_position(self, log_arrival=False):
        """Requests position. Handled asynchronously to avoid blocking."""
        if self.is_moving:
            return # Don't request position while moving to avoid protocol mess
        
        def handle_pos(success, data, err):
            if success and data and len(data) >= 18:
                try:
                    # 6 hex chars per axis, 24-bit 2's complement
                    x_hex = data[0:6]
                    y_hex = data[6:12]
                    z_hex = data[12:18]
                    # Convert raw steps to mm and round to 3 decimal places
                    self.current_pos["x"] = round(self._parse_hex_24bit(x_hex) / self.steps_per_mm['x'], 3)
                    self.current_pos["y"] = round(self._parse_hex_24bit(y_hex) / self.steps_per_mm['y'], 3)
                    self.current_pos["z"] = round(self._parse_hex_24bit(z_hex) / self.steps_per_mm['z'], 3)
                    
                    if log_arrival:
                        self.log_event("POS", "Reached final resting position")
                except ValueError:
                    pass

        self.enqueue_command("@0P\r", expect_data_length=18, callback=handle_pos)

    def move_relative(self, x, y, z, speed, accel=None):
        """Relative movement. Inputs in mm and mm/s. Optional accel in mm/s^2."""
        self.log_event("MOVE", f"Relative X:{x} Y:{y} Z:{z} at {speed}mm/s")
        if accel is not None:
            self.set_acceleration(accel)
            
        self.status_msg = f"Moving relative: X{x} Y{y} Z{z} at {speed} mm/s..."
        def callback(success, data, err):
            if success:
                self.status_msg = "Movement complete."
        
        sx = int(x * self.steps_per_mm['x'])
        sy = int(y * self.steps_per_mm['y'])
        sz = int(z * self.steps_per_mm['z'])
        s_speed = int(speed * self.steps_per_mm['x']) # X dictates 3D vector speed
        
        cmd = f"@0A{sx},{s_speed},{sy},{s_speed},{sz},{s_speed},0,{s_speed}\r"
        self.enqueue_command(cmd, callback=callback)

    def move_relative_steps(self, steps_x, steps_y, steps_z, speed_hz):
        """Raw relative movement using steps and Hz directly (used for calibration)."""
        self.log_event("MOVE", f"Relative (Steps) X:{steps_x} Y:{steps_y} Z:{steps_z} at {speed_hz}Hz")
        self.status_msg = f"Moving relative: X{steps_x} Y{steps_y} Z{steps_z} at {speed_hz} Hz..."
        def callback(success, data, err):
            if success:
                self.status_msg = "Movement complete."
        
        cmd = f"@0A{steps_x},{speed_hz},{steps_y},{speed_hz},{steps_z},{speed_hz},0,{speed_hz}\r"
        self.enqueue_command(cmd, callback=callback)

    def move_absolute(self, x, y, z, speed, accel=None):
        """Absolute 3D movement. Inputs in mm and mm/s. Optional accel in mm/s^2."""
        self.log_event("MOVE", f"Absolute to X:{x} Y:{y} Z:{z} at {speed}mm/s")
        if accel is not None:
            self.set_acceleration(accel)
            
        self.status_msg = f"Moving absolute to: X{x} Y{y} Z{z} at {speed} mm/s..."
        def callback(success, data, err):
            if success:
                self.status_msg = "Movement complete."
                
        sx = int(x * self.steps_per_mm['x'])
        sy = int(y * self.steps_per_mm['y'])
        sz = int(z * self.steps_per_mm['z'])
        s_speed = int(speed * self.steps_per_mm['x'])
        
        cmd = f"@0M{sx},{s_speed},{sy},{s_speed},{sz},{s_speed},0,{s_speed}\r"
        self.enqueue_command(cmd, callback=callback)

    # --- Grid Automation Logic ---
    def generate_grid_points(self, pts, fast, mid, slow, pattern):
        """Mathematical generation of points based on axis priority and pattern."""
        zigzag = (pattern == "zig-zag")
        points = []
        for o in range(pts[slow]['n']):
            vo = pts[slow]['start'] + o * pts[slow]['space']

            m_range = range(pts[mid]['n'])
            if zigzag and (o % 2 != 0):
                m_range = reversed(m_range)

            for m_idx, m in enumerate(m_range):
                vm = pts[mid]['start'] + m * pts[mid]['space']

                i_range = range(pts[fast]['n'])
                if zigzag and ((o * pts[mid]['n'] + m_idx) % 2 != 0):
                    i_range = reversed(list(i_range))

                for i in i_range:
                    vi = pts[fast]['start'] + i * pts[fast]['space']

                    pt_dict = {'x': 0.0, 'y': 0.0, 'z': 0.0}
                    pt_dict[slow] = vo
                    pt_dict[mid] = vm
                    pt_dict[fast] = vi
                    points.append(pt_dict)

        return points, None

    def start_grid(self, points, speed, accel, wait_time, progress_callback=None):
        """Starts the background grid loop."""
        if self.grid_running:
            return False, "Grid is already running."
            
        self.log_event("CMD", f"Starting Grid sequence with {len(points)} points")
        self.set_acceleration(accel)
        self.grid_points = points
        self.grid_speed = speed
        self.grid_wait_time = wait_time
        self.grid_point_index = 0
        self.grid_running = True
        self.grid_waiting_for_move_to_finish = False
        self.grid_next_move_time = 0.0
        self.grid_progress_callback = progress_callback
        
        threading.Thread(target=self._grid_loop, daemon=True).start()
        return True, ""

    def _grid_loop(self):
        """Background thread executing the grid movement and waiting."""
        while self.grid_running and self.grid_point_index < len(self.grid_points):
            if self.grid_waiting_for_move_to_finish:
                # Check if physical movement is finished
                if not self.is_moving and self.cmd_queue.unfinished_tasks == 0:
                    self.grid_waiting_for_move_to_finish = False
                    self.grid_next_move_time = time.time() + self.grid_wait_time
                    if self.grid_progress_callback:
                        self.grid_progress_callback(self.grid_point_index, len(self.grid_points), None) # None = Waiting state
                time.sleep(0.05)
                continue
            
            # Handle wait time (dwell)
            if time.time() < self.grid_next_move_time:
                time.sleep(0.05)
                continue
                
            # Send next point to controller
            pt = self.grid_points[self.grid_point_index]
            self.grid_point_index += 1
            if self.grid_progress_callback:
                self.grid_progress_callback(self.grid_point_index, len(self.grid_points), pt)
                
            self.move_absolute(pt['x'], pt['y'], pt['z'], self.grid_speed)
            self.grid_waiting_for_move_to_finish = True
            
        self.grid_running = False
        if self.grid_progress_callback:
            self.grid_progress_callback(-1, len(self.grid_points), None) # -1 = Finished

    def emergency_stop(self):
        """Sends the immediate stop character (char 253). Clears queues and aborts grids."""
        self.log_event("ERROR", "Emergency Stop Triggered")
        self.grid_running = False
        with self.cmd_queue.mutex:
            self.cmd_queue.queue.clear()
            
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(bytes([253]))
        self.is_moving = False

    def _parse_hex_24bit(self, hex_str):
        val = int(hex_str, 16)
        if val & 0x800000: # Check sign bit
            val -= 0x1000000
        return val


class ApiServer:
    """
    TCP Socket Server to allow MATLAB or other programs to control the axes.
    """
    def __init__(self, controller, host='127.0.0.1', port=5000):
        self.controller = controller
        self.host = host
        self.port = port
        self.server_socket = None
        self.is_running = False
        self.thread = None

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)
        self.is_running = True
        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()
        print(f"API Server listening on {self.host}:{self.port}")

    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.server_socket:
            self.server_socket.close()

    def _server_loop(self):
        while self.is_running:
            try:
                client, addr = self.server_socket.accept()
                client_thread = threading.Thread(target=self._handle_client, args=(client,), daemon=True)
                client_thread.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    print(f"API Server error: {e}")

    def _handle_client(self, client_socket):
        client_socket.settimeout(1.0) # Allow periodic checks of self.is_running
        buffer = ""
        try:
            while self.is_running:
                try:
                    data = client_socket.recv(1024).decode('utf-8')
                    if not data:
                        break # Client disconnected gracefully
                        
                    buffer += data
                    
                    # Process all complete JSON lines in the buffer
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                            
                        try:
                            req = json.loads(line)
                        except json.JSONDecodeError:
                            err_resp = {"status": "error", "msg": "Invalid JSON format"}
                            client_socket.sendall((json.dumps(err_resp) + "\n").encode('utf-8'))
                            continue
                            
                        cmd = req.get("cmd")
                        self.controller.log_event("API", f"Received command: {cmd}")
                        response = {"status": "ok"}
                        ignore_limits = req.get("ignore_limits", False)

                        # Prevent execution if hardware is disconnected
                        if not self.controller.is_running and cmd not in ["get_status", "get_commands"]:
                            response = {"status": "error", "msg": "Command rejected: Not connected to controller."}
                            client_socket.sendall((json.dumps(response) + "\n").encode('utf-8'))
                            continue

                        if cmd == "init":
                            self.controller.init_axes()
                        elif cmd == "set_home_speed":
                            speed = float(req.get("speed", 12.5))
                            self.controller.set_homing_speed(speed, speed, speed)
                        elif cmd == "set_acceleration":
                            accel = float(req.get("accel", 1000.0))
                            self.controller.set_acceleration(accel)
                        elif cmd == "home":
                            # API explicitly configures homing speed to prevent hardware lockup/defaults 
                            speed = float(req.get("speed", 12.5))
                            self.controller.set_homing_speed(speed, speed, speed)
                            self.controller.home_axes()
                        elif cmd == "grid":
                            x_cfg = req.get("x", {})
                            y_cfg = req.get("y", {})
                            z_cfg = req.get("z", {})

                            pts = {
                                'x': {'start': float(x_cfg.get('start', 0.0)), 'space': float(x_cfg.get('space', 10.0)), 'n': int(x_cfg.get('n', 1))},
                                'y': {'start': float(y_cfg.get('start', 0.0)), 'space': float(y_cfg.get('space', 10.0)), 'n': int(y_cfg.get('n', 1))},
                                'z': {'start': float(z_cfg.get('start', 0.0)), 'space': float(z_cfg.get('space', 10.0)), 'n': int(z_cfg.get('n', 1))}
                            }
                            
                            fast = str(req.get("fast", "x")).lower()
                            mid = str(req.get("mid", "y")).lower()
                            slow = str(req.get("slow", "z")).lower()
                            pattern = str(req.get("pattern", "zig-zag")).lower()
                            speed = float(req.get("speed", 10.0))
                            accel = float(req.get("accel", 1000.0))
                            wait_t = float(req.get("wait", 0.5))
                            repetitions = int(req.get("repetitions", 1))

                            if set([fast, mid, slow]) != {'x', 'y', 'z'}:
                                response = {"status": "error", "msg": "Axes fast, mid, slow must be uniquely x, y, z."}
                            elif repetitions < 1:
                                response = {"status": "error", "msg": "Repetitions must be at least 1."}
                            else:
                                points, err = self.controller.generate_grid_points(pts, fast, mid, slow, pattern)
                                if err:
                                    response = {"status": "error", "msg": err}
                                else:
                                    # Multiply the point list by the requested repetitions to create a continuous loop
                                    points = points * repetitions
                                    safe, w_msg = self.controller.check_limits(points)
                                    if not safe and not ignore_limits:
                                        response = {"status": "error", "msg": w_msg + " (Add 'ignore_limits': true to override)"}
                                    else:
                                        success, msg = self.controller.start_grid(points, speed, accel, wait_t)
                                        if success:
                                            response = {"status": "ok", "msg": "Grid started.", "total_points": len(points)}
                                            if not safe: response["msg"] += " (Warning: Limits ignored)"
                                        else:
                                            response = {"status": "error", "msg": msg}
                        elif cmd == "move_rel":
                            x = float(req.get("x", 0.0))
                            y = float(req.get("y", 0.0))
                            z = float(req.get("z", 0.0))
                            speed = float(req.get("speed", 5.0))
                            accel = req.get("accel", None)
                            if accel is not None: accel = float(accel)
                            
                            target = {
                                'x': self.controller.current_pos['x'] + x,
                                'y': self.controller.current_pos['y'] + y,
                                'z': self.controller.current_pos['z'] + z,
                            }
                            safe, w_msg = self.controller.check_limits([target])
                            
                            if not safe and not ignore_limits:
                                response = {"status": "error", "msg": w_msg + " (Add 'ignore_limits': true to override)"}
                            else:
                                self.controller.move_relative(x, y, z, speed, accel)
                                if not safe: response["msg"] = "Move started. (Warning: Limits ignored)"
                                
                        elif cmd == "move_abs":
                            x = float(req.get("x", 0.0))
                            y = float(req.get("y", 0.0))
                            z = float(req.get("z", 0.0))
                            speed = float(req.get("speed", 5.0))
                            accel = req.get("accel", None)
                            if accel is not None: accel = float(accel)
                            
                            safe, w_msg = self.controller.check_limits([{'x': x, 'y': y, 'z': z}])
                            
                            if not safe and not ignore_limits:
                                response = {"status": "error", "msg": w_msg + " (Add 'ignore_limits': true to override)"}
                            else:
                                self.controller.move_absolute(x, y, z, speed, accel)
                                if not safe: response["msg"] = "Move started. (Warning: Limits ignored)"
                                
                        elif cmd == "get_pos":
                            # get_pos is explicitly handled below by the universal position attach
                            pass
                        elif cmd == "get_status":
                            is_busy = self.controller.grid_running or self.controller.cmd_queue.unfinished_tasks > 0
                            response["state"] = {
                                "is_connected": self.controller.is_running,
                                "is_busy": is_busy,
                                "is_moving": self.controller.is_moving,
                                "is_grid_running": self.controller.grid_running,
                                "queue_size": self.controller.cmd_queue.qsize(),
                                "unfinished_tasks": self.controller.cmd_queue.unfinished_tasks,
                                "status_msg": self.controller.status_msg,
                                "pos": self.controller.current_pos
                            }
                        elif cmd == "wait_ready":
                            pass # Logic is handled by the universal wait block below
                        elif cmd == "stop":
                            self.controller.emergency_stop()
                        elif cmd == "get_commands":
                            response["docs"] = self._get_api_docs()
                        else:
                            response = {"status": "error", "msg": "Unknown command"}
                        
                        # --- Universal wait for readiness ---
                        # Check unfinished_tasks instead of is_moving to correctly block for sequential 
                        # commands like init -> set_3d_mode which happen back to back.
                        if (req.get("wait_ready", False) or cmd == "wait_ready") and response.get("status") == "ok":
                            while self.is_running and self.controller.is_running and \
                                  (self.controller.cmd_queue.unfinished_tasks > 0 or self.controller.grid_running):
                                time.sleep(0.05)
                                
                                # Break out and report if a hardware error occurred during the movement
                                if "Error" in self.controller.status_msg:
                                    response["status"] = "error"
                                    response["msg"] = f"Hardware stopped with error: {self.controller.status_msg}"
                                    break
                            
                            # If it finished successfully without errors
                            if response.get("status") == "ok":
                                response["msg"] = response.get("msg", "Action completed.") + " Controller is ready."

                        # --- Ensure 'pos' is ALWAYS attached to successful responses ---
                        if response.get("status") == "ok" and "pos" not in response:
                            response["pos"] = self.controller.current_pos

                        client_socket.sendall((json.dumps(response) + "\n").encode('utf-8'))
                        
                except socket.timeout:
                    continue # Check self.is_running and loop again
        except Exception as e:
            if self.is_running:
                print(f"Client connection closed/error: {e}")
        finally:
            try:
                client_socket.close()
            except:
                pass

    def _get_api_docs(self):
        """Returns a dictionary detailing all supported API commands and limits."""
        wait_param = {"type": "boolean", "default": False, "note": "If true, the API does not reply until the action is physically completed."}
        ign_param = {"type": "boolean", "default": False, "note": "If true, bypasses software soft-limit warnings."}
        
        return {
            "init": {
                "description": "Initialize axes (required after power on or e-stop). Enables 3D mode automatically.",
                "parameters": {"wait_ready": wait_param}
            },
            "set_home_speed": {
                "description": "Set the speed for the homing operation.",
                "parameters": {
                    "speed": {"type": "float", "default": 12.5, "note": "mm/s"}
                }
            },
            "set_acceleration": {
                "description": "Set the acceleration for movements.",
                "parameters": {
                    "accel": {"type": "float", "default": 1000.0, "note": "mm/s^2"}
                }
            },
            "home": {
                "description": "Move all axes to physical reference switches.",
                "parameters": {
                    "speed": {"type": "float", "default": 12.5, "note": "Homing speed in mm/s"},
                    "wait_ready": wait_param
                }
            },
            "move_rel": {
                "description": "Move axes relative to current position.",
                "parameters": {
                    "x": {"type": "float", "default": 0.0, "note": "mm"},
                    "y": {"type": "float", "default": 0.0, "note": "mm"},
                    "z": {"type": "float", "default": 0.0, "note": "mm"},
                    "speed": {"type": "float", "default": 5.0, "note": "mm/s"},
                    "accel": {"type": "float", "default": "current", "note": "Optional: Acceleration in mm/s^2 for this move"},
                    "ignore_limits": ign_param,
                    "wait_ready": wait_param
                }
            },
            "move_abs": {
                "description": "Move axes to absolute coordinates in 3D space.",
                "parameters": {
                    "x": {"type": "float", "default": 0.0, "note": "mm"},
                    "y": {"type": "float", "default": 0.0, "note": "mm"},
                    "z": {"type": "float", "default": 0.0, "note": "mm"},
                    "speed": {"type": "float", "default": 5.0, "note": "mm/s"},
                    "accel": {"type": "float", "default": "current", "note": "Optional: Acceleration in mm/s^2 for this move"},
                    "ignore_limits": ign_param,
                    "wait_ready": wait_param
                }
            },
            "get_pos": {
                "description": "Get current absolute position of all axes (in mm).",
                "parameters": {}
            },
            "get_status": {
                "description": "Check if the controller is busy, moving, or has an error.",
                "parameters": {}
            },
            "wait_ready": {
                "description": "Blocks and waits until the controller finishes all current queues, grids and movements.",
                "parameters": {}
            },
            "stop": {
                "description": "Emergency stop all movements instantly.",
                "parameters": {}
            },
            "grid": {
                "description": "Start an automated 1D/2D/3D grid measurement sequence.",
                "parameters": {
                    "x": {"type": "object", "default": {"start": 0.0, "space": 10.0, "n": 1}, "note": "Dict with start [mm], space [mm], n [points]"},
                    "y": {"type": "object", "default": {"start": 0.0, "space": 10.0, "n": 1}},
                    "z": {"type": "object", "default": {"start": 0.0, "space": 10.0, "n": 1}},
                    "fast": {"type": "string", "default": "x", "note": "Fastest/inner loop axis (x, y, or z)"},
                    "mid": {"type": "string", "default": "y"},
                    "slow": {"type": "string", "default": "z"},
                    "pattern": {"type": "string", "default": "zig-zag", "note": "Routing pattern: 'zig-zag' or 'typewriter'"},
                    "speed": {"type": "float", "default": 10.0, "note": "Speed in mm/s"},
                    "accel": {"type": "float", "default": 1000.0, "note": "Acceleration in mm/s^2"},
                    "wait": {"type": "float", "default": 0.5, "note": "Dwell/wait time at each point in seconds"},
                    "repetitions": {"type": "int", "default": 1, "note": "Number of times to loop the entire grid pattern"},
                    "ignore_limits": ign_param,
                    "wait_ready": wait_param
                }
            },
            "get_commands": {
                "description": "Get documentation for all API commands.",
                "parameters": {}
            }
        }


class AppGUI(tk.Tk):
    """
    Main application window using Tkinter.
    """
    def __init__(self, controller, api_server):
        super().__init__()
        self.controller = controller
        self.api_server = api_server
        
        self.log_data = []
        self.controller.log_callback = self._on_log_event
        
        self.title("isel iMC-S8 Controller")
        self.geometry("900x850")
        self.minsize(850, 650)  # Prevents shrinking below a functional layout size
        self.configure(padx=10, pady=10)

        self.preview_timer = None # Timer for debouncing grid preview updates

        self._build_ui()
        self._update_loop()

    def _build_ui(self):
        # --- Status Bar (Outside Notebook, always visible) ---
        # Packed FIRST so it never disappears when the window is shrunk
        self.lbl_status = ttk.Label(self, text="Disconnected", relief="sunken", anchor="w")
        self.lbl_status.pack(side="bottom", fill="x")
        self.default_bg = self.lbl_status.cget("background")

        # Create Notebook for Tabs
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill="both")

        self.tab_control = ttk.Frame(self.notebook)
        self.tab_grid = ttk.Frame(self.notebook)
        self.tab_settings = ttk.Frame(self.notebook)
        self.tab_help = ttk.Frame(self.notebook)
        self.tab_log = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_control, text="Control Panel")
        self.notebook.add(self.tab_grid, text="Grid Automation")
        self.notebook.add(self.tab_settings, text="Settings")
        self.notebook.add(self.tab_log, text="Event Log")
        self.notebook.add(self.tab_help, text="Help & API")

        # --- TAB 1: CONTROL PANEL ---

        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(self.tab_control, text="Connection")
        conn_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").pack(side="left", padx=5, pady=5)
        self.port_cb = ttk.Combobox(conn_frame, width=15)
        self.port_cb.pack(side="left", padx=5, pady=5)
        
        self.btn_refresh_ports = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.btn_refresh_ports.pack(side="left", padx=5, pady=5)
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5, pady=5)

        self.refresh_ports() # Populate list on startup

        # --- Initialization Frame ---
        init_frame = ttk.LabelFrame(self.tab_control, text="Initialization")
        init_frame.pack(fill="x", padx=5, pady=5)

        ttk.Button(init_frame, text="1. Init Axes (XYZ)", command=self.do_init).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(init_frame, text="2. Home (Reference)", command=self.do_home).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(init_frame, text="Home Speed [mm/s]:").grid(row=0, column=2, padx=(10, 2), pady=5)
        self.ent_home_speed = ttk.Entry(init_frame, width=7)
        self.ent_home_speed.insert(0, "12.5")
        self.ent_home_speed.grid(row=0, column=3, padx=2, pady=5)

        ttk.Label(init_frame, text="Accel [mm/s²]:").grid(row=0, column=4, padx=(10, 2), pady=5)
        self.ent_accel = ttk.Entry(init_frame, width=6)
        self.ent_accel.insert(0, "1000")
        self.ent_accel.grid(row=0, column=5, padx=2, pady=5)
        ttk.Button(init_frame, text="Set", command=self.do_set_accel).grid(row=0, column=6, padx=2, pady=5)
        
        ttk.Button(init_frame, text="STOP", command=lambda: self.controller.emergency_stop()).grid(row=0, column=7, padx=15, pady=5)

        # --- Position Frame ---
        pos_frame = ttk.LabelFrame(self.tab_control, text="Current Position")
        pos_frame.pack(fill="x", padx=5, pady=5)
        
        self.lbl_x = ttk.Label(pos_frame, text="X: 0.000 mm", font=("Arial", 16, "bold"))
        self.lbl_x.pack(side="left", expand=True, pady=10)
        self.lbl_y = ttk.Label(pos_frame, text="Y: 0.000 mm", font=("Arial", 16, "bold"))
        self.lbl_y.pack(side="left", expand=True, pady=10)
        self.lbl_z = ttk.Label(pos_frame, text="Z: 0.000 mm", font=("Arial", 16, "bold"))
        self.lbl_z.pack(side="left", expand=True, pady=10)

        # --- Jogging Frame ---
        jog_frame = ttk.LabelFrame(self.tab_control, text="Manual Jog (Relative)")
        jog_frame.pack(fill="x", padx=5, pady=5)

        ctrl_panel = ttk.Frame(jog_frame)
        ctrl_panel.pack(pady=5)

        ttk.Label(ctrl_panel, text="Step Size [mm]:").grid(row=0, column=0, padx=5, pady=5)
        self.ent_step = ttk.Entry(ctrl_panel, width=10)
        self.ent_step.insert(0, "10.0")
        self.ent_step.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(ctrl_panel, text="Speed [mm/s]:").grid(row=0, column=2, padx=5, pady=5)
        self.ent_speed = ttk.Entry(ctrl_panel, width=10)
        self.ent_speed.insert(0, "5.0")
        self.ent_speed.grid(row=0, column=3, padx=5, pady=5)

        btn_panel = ttk.Frame(jog_frame)
        btn_panel.pack(pady=5)

        ttk.Button(btn_panel, text="Y+", command=lambda: self.jog("y", 1)).grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(btn_panel, text="X-", command=lambda: self.jog("x", -1)).grid(row=1, column=0, padx=5, pady=2)
        ttk.Button(btn_panel, text="X+", command=lambda: self.jog("x", 1)).grid(row=1, column=2, padx=5, pady=2)
        ttk.Button(btn_panel, text="Y-", command=lambda: self.jog("y", -1)).grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Frame(btn_panel, width=20).grid(row=1, column=3) # Spacer

        ttk.Button(btn_panel, text="Z+", command=lambda: self.jog("z", 1)).grid(row=0, column=4, padx=5, pady=2)
        ttk.Button(btn_panel, text="Z-", command=lambda: self.jog("z", -1)).grid(row=2, column=4, padx=5, pady=2)

        # --- Move Absolute Frame ---
        abs_frame = ttk.LabelFrame(self.tab_control, text="Move to Position (Absolute)")
        abs_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(abs_frame, text="X [mm]:").grid(row=0, column=0, padx=2, pady=5)
        self.ent_abs_x = ttk.Entry(abs_frame, width=10)
        self.ent_abs_x.insert(0, "0.0")
        self.ent_abs_x.grid(row=0, column=1, padx=2, pady=5)

        ttk.Label(abs_frame, text="Y [mm]:").grid(row=0, column=2, padx=2, pady=5)
        self.ent_abs_y = ttk.Entry(abs_frame, width=10)
        self.ent_abs_y.insert(0, "0.0")
        self.ent_abs_y.grid(row=0, column=3, padx=2, pady=5)

        ttk.Label(abs_frame, text="Z [mm]:").grid(row=0, column=4, padx=2, pady=5)
        self.ent_abs_z = ttk.Entry(abs_frame, width=10)
        self.ent_abs_z.insert(0, "0.0")
        self.ent_abs_z.grid(row=0, column=5, padx=2, pady=5)

        ttk.Button(abs_frame, text="Go To Position", command=self.move_absolute).grid(row=0, column=6, padx=10, pady=5)

        # --- TAB 2: GRID AUTOMATION ---
        
        # Axis Settings
        ax_frame = ttk.LabelFrame(self.tab_grid, text="Axis Grid Definition")
        ax_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(ax_frame, text="Axis").grid(row=0, column=0, padx=5, pady=2)
        ttk.Label(ax_frame, text="Start Point [mm]").grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(ax_frame, text="Spacing [mm]").grid(row=0, column=2, padx=5, pady=2)
        ttk.Label(ax_frame, text="Number of Points").grid(row=0, column=3, padx=5, pady=2)
        
        # X
        ttk.Label(ax_frame, text="X:").grid(row=1, column=0, padx=5)
        self.ent_g_x_start = ttk.Entry(ax_frame, width=15); self.ent_g_x_start.insert(0, "0.0"); self.ent_g_x_start.grid(row=1, column=1, padx=5, pady=5)
        self.ent_g_x_space = ttk.Entry(ax_frame, width=15); self.ent_g_x_space.insert(0, "10.0"); self.ent_g_x_space.grid(row=1, column=2, padx=5, pady=5)
        self.ent_g_x_pts = ttk.Entry(ax_frame, width=15); self.ent_g_x_pts.insert(0, "4"); self.ent_g_x_pts.grid(row=1, column=3, padx=5, pady=5)
        
        # Y
        ttk.Label(ax_frame, text="Y:").grid(row=2, column=0, padx=5)
        self.ent_g_y_start = ttk.Entry(ax_frame, width=15); self.ent_g_y_start.insert(0, "0.0"); self.ent_g_y_start.grid(row=2, column=1, padx=5, pady=5)
        self.ent_g_y_space = ttk.Entry(ax_frame, width=15); self.ent_g_y_space.insert(0, "10.0"); self.ent_g_y_space.grid(row=2, column=2, padx=5, pady=5)
        self.ent_g_y_pts = ttk.Entry(ax_frame, width=15); self.ent_g_y_pts.insert(0, "4"); self.ent_g_y_pts.grid(row=2, column=3, padx=5, pady=5)
        
        # Z
        ttk.Label(ax_frame, text="Z:").grid(row=3, column=0, padx=5)
        self.ent_g_z_start = ttk.Entry(ax_frame, width=15); self.ent_g_z_start.insert(0, "0.0"); self.ent_g_z_start.grid(row=3, column=1, padx=5, pady=5)
        self.ent_g_z_space = ttk.Entry(ax_frame, width=15); self.ent_g_z_space.insert(0, "10.0"); self.ent_g_z_space.grid(row=3, column=2, padx=5, pady=5)
        self.ent_g_z_pts = ttk.Entry(ax_frame, width=15); self.ent_g_z_pts.insert(0, "1"); self.ent_g_z_pts.grid(row=3, column=3, padx=5, pady=5)
        
        # Routing & Dynamics
        dyn_frame = ttk.LabelFrame(self.tab_grid, text="Routing & Dynamics")
        dyn_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(dyn_frame, text="Fastest Axis (Inner):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.cb_g_fast = ttk.Combobox(dyn_frame, values=["X", "Y", "Z"], width=5, state="readonly"); self.cb_g_fast.set("X"); self.cb_g_fast.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(dyn_frame, text="Middle Axis:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.cb_g_mid = ttk.Combobox(dyn_frame, values=["X", "Y", "Z"], width=5, state="readonly"); self.cb_g_mid.set("Y"); self.cb_g_mid.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Label(dyn_frame, text="Slowest Axis (Outer):").grid(row=0, column=4, padx=5, pady=5, sticky="e")
        self.cb_g_slow = ttk.Combobox(dyn_frame, values=["X", "Y", "Z"], width=5, state="readonly"); self.cb_g_slow.set("Z"); self.cb_g_slow.grid(row=0, column=5, padx=5, pady=5)
        
        ttk.Label(dyn_frame, text="Pattern:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.cb_g_pattern = ttk.Combobox(dyn_frame, values=["Zig-Zag", "Typewriter"], width=12, state="readonly"); self.cb_g_pattern.set("Zig-Zag"); self.cb_g_pattern.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky="w")
        
        ttk.Label(dyn_frame, text="Speed [mm/s]:").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.ent_g_speed = ttk.Entry(dyn_frame, width=8); self.ent_g_speed.insert(0, "10.0"); self.ent_g_speed.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(dyn_frame, text="Accel [mm/s²]:").grid(row=2, column=2, padx=5, pady=5, sticky="e")
        self.ent_g_accel = ttk.Entry(dyn_frame, width=8); self.ent_g_accel.insert(0, "1000"); self.ent_g_accel.grid(row=2, column=3, padx=5, pady=5, sticky="w")
        
        ttk.Label(dyn_frame, text="Wait at Point [s]:").grid(row=2, column=4, padx=5, pady=5, sticky="e")
        self.ent_g_wait = ttk.Entry(dyn_frame, width=8); self.ent_g_wait.insert(0, "0.5"); self.ent_g_wait.grid(row=2, column=5, padx=5, pady=5, sticky="w")

        ttk.Label(dyn_frame, text="Repetitions:").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self.ent_g_reps = ttk.Entry(dyn_frame, width=8); self.ent_g_reps.insert(0, "1"); self.ent_g_reps.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        # --- Path Preview ---
        preview_frame = ttk.LabelFrame(self.tab_grid, text="Path Preview (XY Top-Down)")
        preview_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.preview_canvas = tk.Canvas(preview_frame, height=180, bg="white")
        self.preview_canvas.pack(fill="both", expand=True, padx=5, pady=5)

        # Execution
        exec_frame = ttk.LabelFrame(self.tab_grid, text="Execution")
        exec_frame.pack(fill="x", padx=10, pady=5)
        
        self.lbl_g_status = ttk.Label(exec_frame, text="Ready.", font=("Arial", 10, "bold"))
        self.lbl_g_status.pack(pady=5)
        
        self.g_progress = ttk.Progressbar(exec_frame, orient="horizontal", mode="determinate")
        self.g_progress.pack(fill="x", padx=20, pady=5)
        
        btn_exec_frame = ttk.Frame(exec_frame)
        btn_exec_frame.pack(pady=10)
        
        self.btn_g_start = ttk.Button(btn_exec_frame, text="▶ Start Grid", command=self.start_grid)
        self.btn_g_start.pack(side="left", padx=10)
        
        self.btn_g_stop = ttk.Button(btn_exec_frame, text="⏹ Stop / E-Stop", command=self.stop_grid)
        self.btn_g_stop.pack(side="left", padx=10)

        # --- TAB 3: SETTINGS ---
        
        set_frame = ttk.LabelFrame(self.tab_settings, text="Axis Calibration (Steps per mm)")
        set_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(set_frame, text="These values define how many motor steps correspond to 1 mm of physical travel.\nConsult your hardware specifications (ball screw pitch, microstepping) to find these.").grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="w")

        ttk.Label(set_frame, text="X-Axis Steps/mm:").grid(row=1, column=0, padx=10, pady=5, sticky="e")
        self.ent_st_x = ttk.Entry(set_frame, width=15)
        self.ent_st_x.insert(0, str(self.controller.steps_per_mm['x']))
        self.ent_st_x.grid(row=1, column=1, padx=10, pady=5, sticky="w")
        
        ttk.Label(set_frame, text="Y-Axis Steps/mm:").grid(row=2, column=0, padx=10, pady=5, sticky="e")
        self.ent_st_y = ttk.Entry(set_frame, width=15)
        self.ent_st_y.insert(0, str(self.controller.steps_per_mm['y']))
        self.ent_st_y.grid(row=2, column=1, padx=10, pady=5, sticky="w")
        
        ttk.Label(set_frame, text="Z-Axis Steps/mm:").grid(row=3, column=0, padx=10, pady=5, sticky="e")
        self.ent_st_z = ttk.Entry(set_frame, width=15)
        self.ent_st_z.insert(0, str(self.controller.steps_per_mm['z']))
        self.ent_st_z.grid(row=3, column=1, padx=10, pady=5, sticky="w")

        # --- Soft Limits ---
        lim_frame = ttk.LabelFrame(self.tab_settings, text="Soft Axis Limits (Warnings)")
        lim_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(lim_frame, text="These values define the maximum physical travel limit. Warnings will appear\nif a move targets below 0.0 or above these limits.").grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="w")

        ttk.Label(lim_frame, text="X-Axis Max [mm]:").grid(row=1, column=0, padx=10, pady=5, sticky="e")
        self.ent_lim_x = ttk.Entry(lim_frame, width=15)
        self.ent_lim_x.insert(0, str(self.controller.axis_limits['x']))
        self.ent_lim_x.grid(row=1, column=1, padx=10, pady=5, sticky="w")
        
        ttk.Label(lim_frame, text="Y-Axis Max [mm]:").grid(row=2, column=0, padx=10, pady=5, sticky="e")
        self.ent_lim_y = ttk.Entry(lim_frame, width=15)
        self.ent_lim_y.insert(0, str(self.controller.axis_limits['y']))
        self.ent_lim_y.grid(row=2, column=1, padx=10, pady=5, sticky="w")
        
        ttk.Label(lim_frame, text="Z-Axis Max [mm]:").grid(row=3, column=0, padx=10, pady=5, sticky="e")
        self.ent_lim_z = ttk.Entry(lim_frame, width=15)
        self.ent_lim_z.insert(0, str(self.controller.axis_limits['z']))
        self.ent_lim_z.grid(row=3, column=1, padx=10, pady=5, sticky="w")

        btn_set_panel = ttk.Frame(self.tab_settings)
        btn_set_panel.pack(pady=20)

        ttk.Button(btn_set_panel, text="Apply & Save Settings", command=self.save_settings).pack(side="left", padx=10)
        ttk.Button(btn_set_panel, text="Calibration Tool...", command=self.open_calibration_tool).pack(side="left", padx=10)

        # --- TAB 5: EVENT LOG ---
        log_frame = ttk.Frame(self.tab_log)
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        log_scroll = ttk.Scrollbar(log_frame)
        log_scroll.pack(side="right", fill="y")
        
        cols_log = ("Timestamp", "Type", "X", "Y", "Z", "Event Message")
        self.log_tree = ttk.Treeview(log_frame, columns=cols_log, show="headings", yscrollcommand=log_scroll.set)
        self.log_tree.pack(side="left", fill="both", expand=True)
        log_scroll.config(command=self.log_tree.yview)
        
        self.log_tree.heading("Timestamp", text="Timestamp")
        self.log_tree.heading("Type", text="Type")
        self.log_tree.heading("X", text="X [mm]")
        self.log_tree.heading("Y", text="Y [mm]")
        self.log_tree.heading("Z", text="Z [mm]")
        self.log_tree.heading("Event Message", text="Event Message")
        
        self.log_tree.column("Timestamp", width=170, anchor="center")
        self.log_tree.column("Type", width=70, anchor="center")
        self.log_tree.column("X", width=70, anchor="center")
        self.log_tree.column("Y", width=70, anchor="center")
        self.log_tree.column("Z", width=70, anchor="center")
        self.log_tree.column("Event Message", width=450, anchor="w")
        
        self.log_tree.tag_configure("ERROR", foreground="red", font=("Arial", 9, "bold"))
        self.log_tree.tag_configure("MOVE", foreground="blue")
        self.log_tree.tag_configure("API", foreground="green")
        self.log_tree.tag_configure("CMD", foreground="black")
        self.log_tree.tag_configure("POS", foreground="purple", font=("Arial", 9, "italic"))
        
        btn_log_frame = ttk.Frame(self.tab_log)
        btn_log_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(btn_log_frame, text="Clear Log", command=self.clear_log).pack(side="left", padx=5)
        ttk.Button(btn_log_frame, text="Export Log to CSV...", command=self.export_log_csv).pack(side="left", padx=5)


        # --- TAB 4: HELP & API ---
        help_paned = ttk.PanedWindow(self.tab_help, orient=tk.VERTICAL)
        help_paned.pack(fill="both", expand=True, padx=5, pady=5)

        # 1. Protocol Structure
        proto_frame = ttk.LabelFrame(help_paned, text="1. Protocol & JSON Structure")
        help_paned.add(proto_frame, weight=1)
        
        proto_text = tk.Text(proto_frame, wrap="word", height=6, font=("Consolas", 10))
        proto_text.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        self.proto_content = """Connection: TCP Socket on 127.0.0.1, Port 5000.
Messages must be valid JSON objects terminated by a newline character (\\n).
Units: All physical units are strictly in millimeters (mm), seconds (s), and Hz.

[Request Format Example]            [Response Format Example]
{                                   {
  "cmd": "move_abs",                  "status": "ok",  (or "error")
  "x": 150.0,                         "msg": "Action completed.",
  "speed": 15.0,                      "pos": {"x": 150.0, "y": 0.0, "z": 0.0}
  "accel": 500.0,                   }
  "wait_ready": true                  
}                                   """
        proto_text.insert(tk.END, self.proto_content)
        proto_text.config(state="disabled", bg="#f0f0f0")

        # 2. Command Reference (Treeview)
        cmd_frame = ttk.LabelFrame(help_paned, text="2. Available API Commands & Parameters")
        help_paned.add(cmd_frame, weight=3)

        btn_export_frame = ttk.Frame(cmd_frame)
        btn_export_frame.pack(side="bottom", fill="x", pady=5)

        btn_export = ttk.Button(btn_export_frame, text="Export API Docs...", command=self.export_api_docs)
        btn_export.pack(side="left", expand=True, padx=5)
        
        btn_export_matlab = ttk.Button(btn_export_frame, text="Export MATLAB Wrapper (isel_cmd.m)...", command=self.export_matlab_wrapper)
        btn_export_matlab.pack(side="left", expand=True, padx=5)

        tree_scroll = ttk.Scrollbar(cmd_frame)
        tree_scroll.pack(side="right", fill="y")

        cols = ("Type", "Default", "Description")
        self.cmd_tree = ttk.Treeview(cmd_frame, columns=cols, selectmode="browse", yscrollcommand=tree_scroll.set)
        self.cmd_tree.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        tree_scroll.config(command=self.cmd_tree.yview)

        self.cmd_tree.heading("#0", text="Command / Parameter")
        self.cmd_tree.heading("Type", text="Data Type")
        self.cmd_tree.heading("Default", text="Default Value")
        self.cmd_tree.heading("Description", text="Description / Note")
        
        self.cmd_tree.column("#0", width=180, anchor="w")
        self.cmd_tree.column("Type", width=80, anchor="center")
        self.cmd_tree.column("Default", width=100, anchor="center")
        self.cmd_tree.column("Description", width=400, anchor="w")
        
        # Add visual tags for command rows
        self.cmd_tree.tag_configure("command", font=("Arial", 10, "bold"), background="#e6f2ff")
        self.cmd_tree.tag_configure("param", font=("Arial", 10))

        # Populate Treeview dynamically from the API documentation
        docs = self.api_server._get_api_docs()
        for cmd_name, cmd_info in docs.items():
            # Insert parent command
            cmd_node = self.cmd_tree.insert("", tk.END, text=cmd_name, values=("(Command)", "", cmd_info.get("description", "")), open=True, tags=("command",))
            
            # Insert children parameters
            params = cmd_info.get("parameters", {})
            if not params:
                self.cmd_tree.insert(cmd_node, tk.END, text="  (No parameters)", values=("", "", ""), tags=("param",))
            else:
                for p_name, p_info in params.items():
                    self.cmd_tree.insert(cmd_node, tk.END, text=f"  └ {p_name}", values=(
                        p_info.get("type", ""),
                        str(p_info.get("default", "")),
                        p_info.get("note", "")
                    ), tags=("param",))

        # 3. MATLAB Example Code
        ex_frame = ttk.LabelFrame(help_paned, text="3. MATLAB Sample Code")
        help_paned.add(ex_frame, weight=2)
        
        ex_scroll = ttk.Scrollbar(ex_frame)
        ex_scroll.pack(side="right", fill="y")
        
        ex_text = tk.Text(ex_frame, wrap="word", height=10, font=("Consolas", 10), yscrollcommand=ex_scroll.set)
        ex_text.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        ex_scroll.config(command=ex_text.yview)
        
        self.matlab_content = """% --- Using the isel_cmd wrapper function ---
% First, export the MATLAB function using the button above and save it 
% as isel_cmd.m in your MATLAB working directory.

% 1. Initialize axes
[is_ok, msg] = isel_cmd('init', 'wait_ready', true);

% 2. Home axes
disp('Homing... Please wait.');
[is_ok, msg] = isel_cmd('home', 'wait_ready', true);

% 3. Move to absolute position (with custom acceleration)
disp('Moving to target position...');
[is_ok, msg, data] = isel_cmd('move_abs', 'x', 50, 'y', 20, 'z', 0, 'speed', 10, 'accel', 800, 'wait_ready', true);

% 4. Check results
if ~is_ok
    disp(['Error: ', msg]);
else
    disp('Arrived at destination!');
    disp(['Current X Position: ', num2str(data.pos.x), ' mm']);
end

% 5. Start a grid measurement easily
x_cfg = struct('start', 0, 'space', 10, 'n', 4);
y_cfg = struct('start', 0, 'space', 10, 'n', 4);
[is_ok, msg] = isel_cmd('grid', 'x', x_cfg, 'y', y_cfg, 'wait', 0.5, 'wait_ready', true);
"""
        ex_text.insert(tk.END, self.matlab_content)
        ex_text.config(state="disabled", bg="#f8f9fa") # Read-only with slight gray background

        # --- Bind Auto-Zero on Focus Out ---
        entries = [
            self.ent_step, self.ent_speed, self.ent_home_speed, self.ent_accel,
            self.ent_abs_x, self.ent_abs_y, self.ent_abs_z,
            self.ent_g_x_start, self.ent_g_x_space, self.ent_g_x_pts,
            self.ent_g_y_start, self.ent_g_y_space, self.ent_g_y_pts,
            self.ent_g_z_start, self.ent_g_z_space, self.ent_g_z_pts,
            self.ent_g_speed, self.ent_g_accel, self.ent_g_wait, self.ent_g_reps,
            self.ent_lim_x, self.ent_lim_y, self.ent_lim_z
        ]
        for entry in entries:
            entry.bind("<FocusOut>", self.ensure_zero_on_blur)

        # --- Bindings for Grid Preview Update ---
        grid_inputs = [
            self.ent_g_x_start, self.ent_g_x_space, self.ent_g_x_pts,
            self.ent_g_y_start, self.ent_g_y_space, self.ent_g_y_pts,
            self.ent_g_z_start, self.ent_g_z_space, self.ent_g_z_pts,
            self.ent_g_reps
        ]
        for ent in grid_inputs:
            ent.bind("<KeyRelease>", self.trigger_preview_update)
            
        self.cb_g_fast.bind("<<ComboboxSelected>>", self.trigger_preview_update)
        self.cb_g_mid.bind("<<ComboboxSelected>>", self.trigger_preview_update)
        self.cb_g_slow.bind("<<ComboboxSelected>>", self.trigger_preview_update)
        self.cb_g_pattern.bind("<<ComboboxSelected>>", self.trigger_preview_update)
        self.preview_canvas.bind("<Configure>", self.trigger_preview_update)

        # Initial trigger
        self.after(500, self._draw_preview)

    def save_settings(self):
        try:
            self.controller.steps_per_mm['x'] = float(self.ent_st_x.get())
            self.controller.steps_per_mm['y'] = float(self.ent_st_y.get())
            self.controller.steps_per_mm['z'] = float(self.ent_st_z.get())
            
            self.controller.axis_limits['x'] = float(self.ent_lim_x.get())
            self.controller.axis_limits['y'] = float(self.ent_lim_y.get())
            self.controller.axis_limits['z'] = float(self.ent_lim_z.get())
            
            success, error_msg = self.controller.save_settings()
            if success:
                messagebox.showinfo("Success", "Settings saved successfully!")
            else:
                messagebox.showerror("Error", f"Failed to save settings:\n{error_msg}")
        except ValueError:
            messagebox.showerror("Error", "Inputs must be valid numbers (e.g. 100 or 100.5)")

    def open_calibration_tool(self):
        """Opens a popup window to assist with axis calibration."""
        if not self.controller.is_running:
            messagebox.showwarning("Warning", "Please connect to the controller first.")
            return

        calib_win = tk.Toplevel(self)
        calib_win.title("Axis Calibration Tool")
        calib_win.geometry("400x350")
        calib_win.grab_set() # Make it modal to block main window interactions

        ttk.Label(calib_win, text="1. Select Axis to Calibrate:").pack(pady=(15, 2))
        axis_var = tk.StringVar(value="x")
        axis_frame = ttk.Frame(calib_win)
        axis_frame.pack()
        ttk.Radiobutton(axis_frame, text="X-Axis", variable=axis_var, value="x").pack(side="left", padx=10)
        ttk.Radiobutton(axis_frame, text="Y-Axis", variable=axis_var, value="y").pack(side="left", padx=10)
        ttk.Radiobutton(axis_frame, text="Z-Axis", variable=axis_var, value="z").pack(side="left", padx=10)

        ttk.Label(calib_win, text="2. Steps to Move (Speed: 5000 Hz):").pack(pady=(15, 2))
        ent_steps = ttk.Entry(calib_win, width=15)
        ent_steps.insert(0, "10000")
        ent_steps.pack()

        def do_move():
            try:
                steps = int(ent_steps.get())
                axis = axis_var.get()
                sx, sy, sz = 0, 0, 0
                if axis == "x": sx = steps
                if axis == "y": sy = steps
                if axis == "z": sz = steps
                self.controller.move_relative_steps(sx, sy, sz, 5000)
            except ValueError:
                messagebox.showerror("Error", "Steps must be an integer.", parent=calib_win)

        ttk.Button(calib_win, text="Move Axis", command=do_move).pack(pady=5)

        ttk.Label(calib_win, text="3. Enter physically measured distance [mm]:").pack(pady=(20, 2))
        ent_measured = ttk.Entry(calib_win, width=15)
        ent_measured.pack()

        def do_calculate():
            try:
                steps = int(ent_steps.get())
                measured_mm = float(ent_measured.get())
                if measured_mm == 0:
                    messagebox.showerror("Error", "Distance cannot be zero.", parent=calib_win)
                    return
                
                new_val = steps / measured_mm
                axis = axis_var.get()
                
                # Apply calculated value to the main UI setting entries
                if axis == "x":
                    self.ent_st_x.delete(0, tk.END)
                    self.ent_st_x.insert(0, f"{new_val:.4f}")
                elif axis == "y":
                    self.ent_st_y.delete(0, tk.END)
                    self.ent_st_y.insert(0, f"{new_val:.4f}")
                elif axis == "z":
                    self.ent_st_z.delete(0, tk.END)
                    self.ent_st_z.insert(0, f"{new_val:.4f}")

                messagebox.showinfo("Success", f"Calculated {new_val:.4f} Steps/mm for {axis.upper()}-Axis.\n\nDon't forget to click 'Apply & Save Settings' in the main window to store this.", parent=calib_win)
                calib_win.destroy()
            except ValueError:
                messagebox.showerror("Error", "Please enter valid numbers.", parent=calib_win)

        ttk.Button(calib_win, text="Calculate & Apply", command=do_calculate).pack(pady=(5, 10))

    def export_api_docs(self):
        """Exports the API documentation to a text or markdown file."""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt", 
            filetypes=[("Text Files", "*.txt"), ("Markdown Files", "*.md"), ("All Files", "*.*")],
            title="Export API Documentation",
            initialfile="isel_api_docs.txt"
        )
        
        if not file_path:
            return
            
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write("=== ISEL iMC-S8 Controller API Documentation ===\n\n")
                
                f.write("--- 1. Protocol & JSON Structure ---\n")
                f.write(self.proto_content + "\n\n")
                
                f.write("--- 2. Available API Commands & Parameters ---\n")
                docs = self.api_server._get_api_docs()
                for cmd_name, cmd_info in docs.items():
                    f.write(f"\nCommand: {cmd_name}\n")
                    f.write(f"  Description: {cmd_info.get('description', '')}\n")
                    params = cmd_info.get("parameters", {})
                    if not params:
                        f.write("  Parameters: None\n")
                    else:
                        f.write("  Parameters:\n")
                        for p_name, p_info in params.items():
                            f.write(f"    - {p_name} ({p_info.get('type', '')}): ")
                            f.write(f"Default = {p_info.get('default', '')}. ")
                            f.write(f"{p_info.get('note', '')}\n")
                            
                f.write("\n\n--- 3. MATLAB Sample Code ---\n")
                f.write(self.matlab_content + "\n")
                
            messagebox.showinfo("Success", f"API documentation successfully exported to:\n{file_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not save file:\n{str(e)}")

    def export_matlab_wrapper(self):
        """Exports the reusable MATLAB wrapper function."""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".m", 
            filetypes=[("MATLAB Code", "*.m"), ("All Files", "*.*")],
            title="Export MATLAB Wrapper",
            initialfile="isel_cmd.m"
        )
        
        if not file_path:
            return
            
        matlab_code = """function [is_ok, msg, data] = isel_cmd(command, varargin)
% ISEL_CMD Sends commands to the Isel Controller Python API.
%
% Usage:
%   [is_ok, msg] = isel_cmd('init');
%   [is_ok, msg] = isel_cmd('home', 'wait_ready', true);
%   [is_ok, msg, data] = isel_cmd('move_abs', 'x', 50, 'y', 20, 'speed', 15, 'accel', 800, 'wait_ready', true);
%   [is_ok, msg, data] = isel_cmd('get_pos');
%
% Note: The function keeps a persistent TCP connection open to minimize overhead.

persistent t;

% Initialize connection if it does not exist or was closed
if isempty(t) || ~isvalid(t)
    try
        t = tcpclient("127.0.0.1", 5000, "Timeout", 300);
    catch ME
        is_ok = false;
        msg = ['Connection failed: ', ME.message];
        data = [];
        return;
    end
end

% Flush leftover data in the buffer to prevent reading old responses
if t.NumBytesAvailable > 0
    read(t, t.NumBytesAvailable);
end

% Build the request structure
req = struct();
req.cmd = command;

% Parse name-value pairs
for i = 1:2:length(varargin)
    if i+1 <= length(varargin)
        req.(varargin{i}) = varargin{i+1};
    end
end

% Convert to JSON and send
try
    json_str = jsonencode(req);
    write(t, uint8([json_str, newline]));
    
    % Wait for the response (blocks until newline is received)
    response_str = readline(t);
    
    if isempty(response_str)
        is_ok = false;
        msg = 'No response from server.';
        data = [];
        return;
    end
    
    % Decode JSON
    data = jsondecode(response_str);
    
    is_ok = strcmp(data.status, 'ok');
    if isfield(data, 'msg')
        msg = data.msg;
    else
        msg = '';
    end
    
catch ME
    is_ok = false;
    msg = ['Communication error: ', ME.message];
    data = [];
end
end
"""
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(matlab_code)
            messagebox.showinfo("Success", f"MATLAB wrapper successfully exported to:\n{file_path}\n\nMake sure this file is in your MATLAB working directory!")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not save file:\n{str(e)}")

    def ensure_zero_on_blur(self, event):
        """Automatically fills the entry with '0' if the user leaves it empty."""
        widget = event.widget
        if not widget.get().strip():
            widget.insert(0, "0")

    def toggle_connection(self):
        if self.controller.is_running:
            self.controller.disconnect()
            self.btn_connect.config(text="Connect")
            self.btn_refresh_ports.config(state="normal")
        else:
            port = self.port_cb.get()
            if not port:
                messagebox.showerror("Error", "Please select a COM port.")
                return
            if self.controller.connect(port):
                self.btn_connect.config(text="Disconnect")
                self.btn_refresh_ports.config(state="disabled")
                
                # Automatically initialize axes and 3D mode upon successful connection
                # Added a slight 500ms delay to ensure the serial port is fully settled
                self.after(500, self.do_init)
            else:
                messagebox.showerror("Error", self.controller.status_msg)
        
        self.lbl_status.config(text=self.controller.status_msg)

    def refresh_ports(self):
        """Refreshes the list of available COM ports in the combobox."""
        available_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_cb['values'] = available_ports
        current_selection = self.port_cb.get()
        
        if available_ports:
            # If the current selection is no longer valid, switch to the first available port
            if current_selection not in available_ports:
                self.port_cb.set(available_ports[0])
        else:
            self.port_cb.set('')

    def do_init(self):
        self.controller.init_axes()
        self.do_set_accel() # Automatically apply acceleration on init

    def do_set_accel(self):
        """Sets the requested acceleration."""
        try:
            accel = float(self.ent_accel.get())
            self.controller.set_acceleration(accel)
        except ValueError:
            messagebox.showerror("Input Error", "Acceleration must be a number.")

    def do_home(self):
        """Sets the requested homing speed and then triggers homing."""
        try:
            h_speed_mm = float(self.ent_home_speed.get())
            
            # Calculate what this translates to in Hz to verify safety limits
            h_speed_hz = int(h_speed_mm * self.controller.steps_per_mm['x'])
            
            # --- Safety Checks for Homing Speed ---
            if h_speed_hz < 300:
                messagebox.showwarning("Warning", "Homing speed is too low (< 300 Hz Start-Stop Frequency).\nIt has been automatically adjusted.")
                # Force minimum 300 Hz
                h_speed_mm = 300.0 / self.controller.steps_per_mm['x']
                self.ent_home_speed.delete(0, tk.END)
                self.ent_home_speed.insert(0, str(round(h_speed_mm, 2)))
            elif h_speed_hz > 4000:
                messagebox.showwarning("Safety Warning", "Homing speed is too high and might damage the physical reference switches due to inertia.\n\nSpeed has been capped at equivalent of 4000 Hz.")
                # Force maximum 4000 Hz
                h_speed_mm = 4000.0 / self.controller.steps_per_mm['x']
                self.ent_home_speed.delete(0, tk.END)
                self.ent_home_speed.insert(0, str(round(h_speed_mm, 2)))
                
            # Queue the speed setting command, immediately followed by the homing command
            self.controller.set_homing_speed(h_speed_mm, h_speed_mm, h_speed_mm)
            self.controller.home_axes()
        except ValueError:
            messagebox.showerror("Input Error", "Homing speed must be a number.")

    def jog(self, axis, direction):
        try:
            step = float(self.ent_step.get()) * direction
            speed = float(self.ent_speed.get())
            
            x, y, z = 0.0, 0.0, 0.0
            if axis == "x": x = step
            if axis == "y": y = step
            if axis == "z": z = step
            
            target = {
                'x': self.controller.current_pos['x'] + x,
                'y': self.controller.current_pos['y'] + y,
                'z': self.controller.current_pos['z'] + z
            }
            
            safe, w_msg = self.controller.check_limits([target])
            if not safe:
                if not messagebox.askyesno("Limit Warning", w_msg + "\n\nDo you want to continue anyway?"):
                    return

            self.controller.move_relative(x, y, z, speed)
        except ValueError:
            messagebox.showerror("Input Error", "Step and Speed must be numbers.")

    def move_absolute(self):
        try:
            x = float(self.ent_abs_x.get())
            y = float(self.ent_abs_y.get())
            z = float(self.ent_abs_z.get())
            speed = float(self.ent_speed.get())

            safe, w_msg = self.controller.check_limits([{'x': x, 'y': y, 'z': z}])
            if not safe:
                if not messagebox.askyesno("Limit Warning", w_msg + "\n\nDo you want to continue anyway?"):
                    return

            self.controller.move_absolute(x, y, z, speed)
        except ValueError:
            messagebox.showerror("Input Error", "Coordinates and Speed must be numbers.")

    # --- Grid Automation Logic ---

    def _on_log_event(self, timestamp, event_type, message, x, y, z):
        """Thread-safe trigger for GUI log update."""
        self.after(0, self._insert_log_gui, timestamp, event_type, message, x, y, z)
        
    def _insert_log_gui(self, timestamp, event_type, message, x, y, z):
        """Inserts a new log entry into the Treeview and auto-scrolls."""
        x_str = f"{x:.3f}"
        y_str = f"{y:.3f}"
        z_str = f"{z:.3f}"
        row_data = [timestamp, event_type, x_str, y_str, z_str, message]
        
        self.log_data.append(row_data)
        item = self.log_tree.insert("", tk.END, values=row_data, tags=(event_type,))
        self.log_tree.see(item)
        
    def clear_log(self):
        """Clears all entries from the log memory and GUI."""
        if messagebox.askyesno("Clear Log", "Are you sure you want to clear the entire event log?"):
            self.log_data.clear()
            for item in self.log_tree.get_children():
                self.log_tree.delete(item)
                
    def export_log_csv(self):
        """Exports the current log data to a CSV file."""
        if not self.log_data:
            messagebox.showinfo("Export", "The log is empty.")
            return
            
        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv", 
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            title="Export Log to CSV",
            initialfile=f"isel_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        if not file_path:
            return
            
        try:
            with open(file_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp", "Type", "X [mm]", "Y [mm]", "Z [mm]", "Event Message"])
                writer.writerows(self.log_data)
            messagebox.showinfo("Success", f"Log successfully exported to:\n{file_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not save file:\n{str(e)}")

    def trigger_preview_update(self, event=None):
        """Debounces and schedules a canvas redraw."""
        if hasattr(self, 'preview_timer') and self.preview_timer:
            self.after_cancel(self.preview_timer)
        self.preview_timer = self.after(300, self._draw_preview)

    def _calculate_points(self):
        """Parses GUI inputs and requests mathematical point generation from the controller."""
        try:
            pts = {
                'x': {'start': float(self.ent_g_x_start.get()), 'space': float(self.ent_g_x_space.get()), 'n': int(self.ent_g_x_pts.get())},
                'y': {'start': float(self.ent_g_y_start.get()), 'space': float(self.ent_g_y_space.get()), 'n': int(self.ent_g_y_pts.get())},
                'z': {'start': float(self.ent_g_z_start.get()), 'space': float(self.ent_g_z_space.get()), 'n': int(self.ent_g_z_pts.get())}
            }
            reps = int(self.ent_g_reps.get())
        except ValueError:
            return None, "Invalid numeric input in grid settings."

        fast = self.cb_g_fast.get().lower()
        mid = self.cb_g_mid.get().lower()
        slow = self.cb_g_slow.get().lower()
        pattern = self.cb_g_pattern.get().lower()

        if set([fast, mid, slow]) != {'x', 'y', 'z'}:
            return None, "Fastest, Middle, and Slowest axes must be uniquely X, Y, and Z."

        for k, v in pts.items():
            if v['n'] < 1:
                return None, f"Number of points for {k.upper()} must be at least 1."
                
        if reps < 1:
            return None, "Repetitions must be at least 1."

        points, err = self.controller.generate_grid_points(pts, fast, mid, slow, pattern)
        if err:
            return None, err
            
        # Multiply the point list by the requested repetitions
        points = points * reps
        return points, None

    def _draw_preview(self):
        """Renders the grid path into the canvas."""
        if not hasattr(self, 'preview_canvas'): return
        self.preview_canvas.delete("all")
        
        points, err = self._calculate_points()
        if err:
            self.preview_canvas.create_text(10, 10, text=f"Error: {err}", fill="red", anchor="nw")
            return
            
        if not points:
            return

        w = self.preview_canvas.winfo_width()
        h = self.preview_canvas.winfo_height()
        if w < 10 or h < 10: 
            return # Canvas not ready yet, wait for another event

        margin = 30
        
        # Add a slight 2D offset for Z so 3D layers don't stack invisibly over each other
        proj_pts = []
        for p in points:
            px = p['x'] + p['z'] * 0.1
            py = p['y'] - p['z'] * 0.1
            proj_pts.append((px, py))

        min_x = min(p[0] for p in proj_pts)
        max_x = max(p[0] for p in proj_pts)
        min_y = min(p[1] for p in proj_pts)
        max_y = max(p[1] for p in proj_pts)

        dx = max_x - min_x
        dy = max_y - min_y
        if dx == 0: dx = 1
        if dy == 0: dy = 1

        scale_x = (w - 2 * margin) / dx
        scale_y = (h - 2 * margin) / dy
        scale = min(scale_x, scale_y)

        cx = w / 2 - (min_x + max_x) * scale / 2
        cy = h / 2 - (min_y + max_y) * -scale / 2 # Invert Y so positive is up

        canvas_coords = []
        for px, py in proj_pts:
            cx_pt = cx + px * scale
            cy_pt = cy - py * scale
            canvas_coords.append((cx_pt, cy_pt))

        # Draw connecting lines
        if len(canvas_coords) > 1:
            self.preview_canvas.create_line(canvas_coords, fill="royalblue", width=2)

        # Draw point nodes
        for i, (cx_pt, cy_pt) in enumerate(canvas_coords):
            r = 3
            color = "gray"
            if i == 0:
                color = "green" # Start
                r = 6
            elif i == len(canvas_coords) - 1:
                color = "red" # End
                r = 6
            self.preview_canvas.create_oval(cx_pt-r, cy_pt-r, cx_pt+r, cy_pt+r, fill=color, outline="black")

        self.preview_canvas.create_text(10, 10, text=f"Total Points: {len(points)} (Start = Green, End = Red)", anchor="nw", font=("Arial", 9, "bold"))

    def _grid_progress_cb(self, idx, total, pt):
        """Callback from controller grid thread. Schedules GUI update in main thread."""
        self.after(0, lambda: self._update_grid_ui(idx, total, pt))

    def _update_grid_ui(self, idx, total, pt):
        """Safely updates UI elements during grid execution."""
        if idx == -1: # Finished or stopped
            self.lbl_g_status.config(text="Grid automation complete/stopped.")
            self.btn_g_start.config(state="normal")
        else:
            self.g_progress["maximum"] = total
            self.g_progress["value"] = idx
            if pt:
                self.lbl_g_status.config(text=f"Moving to point {idx} of {total}: X={pt['x']:.2f} Y={pt['y']:.2f} Z={pt['z']:.2f}")
            else:
                self.lbl_g_status.config(text=f"Waiting at point {idx} of {total}...")

    def start_grid(self):
        if not self.controller.is_running:
            messagebox.showerror("Error", "Connect to the controller first.")
            return
            
        points, err = self._calculate_points()
        if err:
            messagebox.showerror("Configuration Error", err)
            return
            
        safe, w_msg = self.controller.check_limits(points)
        if not safe:
            if not messagebox.askyesno("Limit Warning", w_msg + "\n\nDo you want to continue anyway?"):
                return
                
        try:
            speed = float(self.ent_g_speed.get())
            accel = float(self.ent_g_accel.get())
            wait_time = float(self.ent_g_wait.get())
        except ValueError:
            messagebox.showerror("Input Error", "Please ensure speed, accel and wait time are valid numbers.")
            return
            
        self.btn_g_start.config(state="disabled")
        
        success, msg = self.controller.start_grid(points, speed, accel, wait_time, progress_callback=self._grid_progress_cb)
        if not success:
            messagebox.showerror("Error", msg)
            self.btn_g_start.config(state="normal")
            
    def stop_grid(self):
        self.controller.emergency_stop()
        self.lbl_g_status.config(text="Grid stopped by user/API.")
        self.btn_g_start.config(state="normal")

    def _update_loop(self):
        """Periodically polls position and updates GUI labels."""
        if self.controller.is_running:
            is_busy = self.controller.cmd_queue.unfinished_tasks > 0 or self.controller.grid_running or self.controller.is_moving
            if is_busy:
                self.was_busy = True
                # Visual feedback that the machine is currently busy/moving
                self.lbl_status.config(text=f"⚙️ BUSY: {self.controller.status_msg}", background="gold")
            else:
                # If we were previously busy, fetch position one last time and flag it as a resting point
                log_arrival = getattr(self, 'was_busy', False)
                self.was_busy = False
                
                # Only ask for position if controller isn't currently processing ANY movement/queue
                self.controller.get_position(log_arrival=log_arrival)
                
                # Update status bar color based on error state
                if "Error" in self.controller.status_msg:
                    self.lbl_status.config(text=self.controller.status_msg, background="lightcoral")
                else:
                    self.lbl_status.config(text=self.controller.status_msg, background=self.default_bg)
            
            # Update GUI Labels
            pos = self.controller.current_pos
            self.lbl_x.config(text=f"X: {pos['x']:.3f} mm")
            self.lbl_y.config(text=f"Y: {pos['y']:.3f} mm")
            self.lbl_z.config(text=f"Z: {pos['z']:.3f} mm")
        else:
            self.lbl_status.config(text=self.controller.status_msg, background=self.default_bg)

        # Call this function again in 200 ms
        self.after(200, self._update_loop)

if __name__ == "__main__":
    controller = IselController()
    
    # Start TCP Server on port 5000 for MATLAB communication
    api = ApiServer(controller, host='0.0.0.0', port=5000)
    api.start()

    # Start Main GUI
    app = AppGUI(controller, api)
    app.mainloop()

    # Cleanup on exit
    api.stop()
    controller.disconnect()