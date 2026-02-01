#!/usr/bin/env python3
"""
Flight Computer Data Monitor
Live plotting and logging tool for Teensy-based flight computer
Includes HITL (Hardware-In-The-Loop) functionality for testing
"""

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import threading
import queue
from collections import deque
import time
from datetime import datetime
import csv
import os

class FlightComputerMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("Flight Computer Monitor")
        self.root.geometry("1400x900")
        
        # Data storage
        self.max_points = 500  # Maximum points to display
        self.data_buffer = queue.Queue()
        self.error_buffer = queue.Queue()
        
        # Column names from the data format
        self.columns = [
            "Time", "Baro Altitude", "Accel X", "Accel Y", "Accel Z",
            "Gyro X", "Gyro Y", "Gyro Z", "GPS Lat", "GPS Lon",
            "GPS Vel X", "GPS Vel Y", "GPS Vel Z", "State X", "State Y", "State Z",
            "State Vel X", "State Vel Y", "State Vel Z", "State Rot X", "State Rot Y",
            "State Rot Z", "Rocket State", "Battery Voltage"
        ]
        
        # Initialize data storage for each column
        self.data = {col: deque(maxlen=self.max_points) for col in self.columns}
        self.time_data = deque(maxlen=self.max_points)
        
        # Visibility toggles for each data series
        self.visibility = {col: tk.BooleanVar(value=False) for col in self.columns if col != "Time"}
        
        # Serial connection
        self.serial_conn = None
        self.is_running = False
        self.read_thread = None
        
        # HITL mode
        self.hitl_active = False
        self.hitl_thread = None
        self.hitl_data = []
        self.hitl_headers = []
        self.hitl_mode = tk.StringVar(value="ORK")
        self.hitl_file_path = tk.StringVar(value="No file loaded")
        self.hitl_playback_speed = tk.DoubleVar(value=1.0)
        self.hitl_loop = tk.BooleanVar(value=False)
        
        # Setup GUI
        self.setup_gui()
        
        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        
    def setup_gui(self):
        """Setup the GUI layout"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel for controls
        left_panel = ttk.Frame(main_frame, width=250)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        left_panel.pack_propagate(False)
        
        # Connection controls
        conn_frame = ttk.LabelFrame(left_panel, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(conn_frame, text="Port:").pack(anchor=tk.W)
        self.port_combo = ttk.Combobox(conn_frame, width=20)
        self.port_combo.pack(fill=tk.X, pady=(0, 5))
        self.refresh_ports()
        
        ttk.Label(conn_frame, text="Baud Rate:").pack(anchor=tk.W)
        self.baud_combo = ttk.Combobox(conn_frame, values=[9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600], width=20)
        self.baud_combo.set(115200)
        self.baud_combo.pack(fill=tk.X, pady=(0, 5))
        
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X)
        
        self.refresh_btn = ttk.Button(btn_frame, text="Refresh", command=self.refresh_ports, width=10)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 2))
        
        self.connect_btn = ttk.Button(btn_frame, text="Connect", command=self.toggle_connection, width=10)
        self.connect_btn.pack(side=tk.LEFT)
        
        # HITL Controls
        hitl_frame = ttk.LabelFrame(left_panel, text="HITL Mode", padding=5)
        hitl_frame.pack(fill=tk.X, pady=(0, 5))
        
        # Mode selection
        ttk.Label(hitl_frame, text="Mode:").pack(anchor=tk.W)
        mode_frame = ttk.Frame(hitl_frame)
        mode_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Radiobutton(mode_frame, text="ORK", variable=self.hitl_mode, value="ORK").pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="FLD", variable=self.hitl_mode, value="FLD").pack(side=tk.LEFT, padx=(10, 0))
        
        # File selection
        ttk.Label(hitl_frame, text="CSV File:").pack(anchor=tk.W)
        file_display = ttk.Label(hitl_frame, textvariable=self.hitl_file_path, relief=tk.SUNKEN, anchor=tk.W)
        file_display.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(hitl_frame, text="Load CSV File", command=self.load_hitl_file).pack(fill=tk.X, pady=(0, 5))
        
        # Playback speed
        ttk.Label(hitl_frame, text="Playback Speed:").pack(anchor=tk.W)
        speed_frame = ttk.Frame(hitl_frame)
        speed_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Scale(speed_frame, from_=0.1, to=10.0, variable=self.hitl_playback_speed, orient=tk.HORIZONTAL).pack(side=tk.LEFT, fill=tk.X, expand=True)
        speed_label = ttk.Label(speed_frame, text="1.0x", width=5)
        speed_label.pack(side=tk.LEFT, padx=(5, 0))
        
        def update_speed_label(*args):
            speed_label.config(text=f"{self.hitl_playback_speed.get():.1f}x")
        self.hitl_playback_speed.trace_add('write', update_speed_label)
        
        # Loop option
        ttk.Checkbutton(hitl_frame, text="Loop playback", variable=self.hitl_loop).pack(anchor=tk.W, pady=(0, 5))
        
        # Start/Stop button
        self.hitl_btn = ttk.Button(hitl_frame, text="Start HITL", command=self.toggle_hitl, state=tk.DISABLED)
        self.hitl_btn.pack(fill=tk.X)
        
        # Data series selection
        series_frame = ttk.LabelFrame(left_panel, text="Data Series", padding=5)
        series_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        # Add scrollable frame for checkboxes
        canvas = tk.Canvas(series_frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(series_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Enable mouse wheel scrolling
        def on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        def bind_mousewheel(event):
            canvas.bind_all("<MouseWheel>", on_mousewheel)
        
        def unbind_mousewheel(event):
            canvas.unbind_all("<MouseWheel>")
        
        canvas.bind("<Enter>", bind_mousewheel)
        canvas.bind("<Leave>", unbind_mousewheel)
        
        # Add checkboxes for each data series
        for col in self.columns:
            if col != "Time":
                cb = ttk.Checkbutton(scrollable_frame, text=col, variable=self.visibility[col])
                cb.pack(anchor=tk.W, pady=1)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Buttons for quick selection
        quick_frame = ttk.Frame(left_panel)
        quick_frame.pack(fill=tk.X)
        
        ttk.Button(quick_frame, text="Select All", command=self.select_all, width=12).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Button(quick_frame, text="Clear All", command=self.clear_all, width=12).pack(side=tk.LEFT)
        
        # Right panel for plot and errors
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Plot area
        plot_frame = ttk.LabelFrame(right_panel, text="Live Data Plot", padding=5)
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.ax.set_xlabel('Time (ms)')
        self.ax.set_ylabel('Value')
        self.ax.set_title('Flight Computer Data')
        self.ax.grid(True, alpha=0.3)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Error/Warning log
        log_frame = ttk.LabelFrame(right_panel, text="Errors & Warnings", padding=5)
        log_frame.pack(fill=tk.BOTH, pady=(0, 0))
        
        self.error_text = scrolledtext.ScrolledText(log_frame, height=8, wrap=tk.WORD, bg='#1e1e1e', fg='#d4d4d4', font=('Consolas', 9))
        self.error_text.pack(fill=tk.BOTH, expand=True)
        
        # Clear log button
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).pack(pady=(5, 0))
        
    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if not self.is_running:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """Connect to the selected serial port"""
        port = self.port_combo.get()
        baud = int(self.baud_combo.get())
        
        if not port:
            self.log_error("Error: No port selected")
            return
        
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
            self.is_running = True
            self.connect_btn.config(text="Disconnect")
            self.log_error(f"Connected to {port} at {baud} baud")
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
        except Exception as e:
            self.log_error(f"Error connecting: {str(e)}")
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.is_running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
        
        self.connect_btn.config(text="Connect")
        self.log_error("Disconnected")
    
    def read_serial(self):
        """Read data from serial port in a separate thread"""
        in_data_block = False
        
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # Check for delimiter
                    if line == "=============":
                        in_data_block = not in_data_block
                        continue
                    
                    # Skip header lines
                    if line.startswith("Time,Baro Altitude"):
                        continue
                    
                    # Check if this looks like a data line (starts with number, has commas)
                    if self.is_data_line(line):
                        self.data_buffer.put(line)
                    else:
                        # This is error/warning/other info
                        self.error_buffer.put(line)
                            
            except Exception as e:
                self.error_buffer.put(f"Read error: {str(e)}")
                time.sleep(0.1)
    
    def is_data_line(self, line):
        """Check if a line looks like valid data"""
        if not line:
            return False
        
        # Data lines should start with a number (timestamp)
        parts = line.split(',')
        
        # Should have at least several comma-separated values
        if len(parts) < 10:
            return False
        
        # Check if first element is a pure number (timestamp)
        # and doesn't contain a colon (which would indicate an error message)
        if ':' in parts[0]:
            return False
            
        try:
            float(parts[0])
            # Also verify second value is numeric (baro altitude)
            float(parts[1])
            return True
        except (ValueError, IndexError):
            return False
    
    def parse_data_line(self, line):
        """Parse a CSV data line"""
        try:
            values = line.split(',')
            # Accept lines with correct number of columns or pad/trim as needed
            data_dict = {}
            for i, col in enumerate(self.columns):
                try:
                    if i < len(values):
                        data_dict[col] = float(values[i])
                    else:
                        data_dict[col] = 0.0
                except (ValueError, IndexError):
                    data_dict[col] = 0.0
            return data_dict
        except Exception as e:
            self.error_buffer.put(f"Parse error: {str(e)} - Line: {line}")
        return None
    
    def update_plot(self, frame):
        """Update the plot with new data"""
        # Process new data from buffer
        data_count = 0
        while not self.data_buffer.empty() and data_count < 50:  # Process up to 50 points per update
            try:
                line = self.data_buffer.get_nowait()
                parsed = self.parse_data_line(line)
                if parsed:
                    for col in self.columns:
                        self.data[col].append(parsed[col])
                    data_count += 1
            except queue.Empty:
                break
        
        # Process error messages
        while not self.error_buffer.empty():
            try:
                msg = self.error_buffer.get_nowait()
                self.log_error(msg)
            except queue.Empty:
                break
        
        # Update plot
        if len(self.data["Time"]) > 0:
            self.ax.clear()
            self.ax.set_xlabel('Time (ms)')
            self.ax.set_ylabel('Value')
            self.ax.set_title('Flight Computer Data')
            self.ax.grid(True, alpha=0.3)
            
            time_values = list(self.data["Time"])
            
            # Plot each visible series
            for col in self.columns:
                if col != "Time" and self.visibility[col].get():
                    values = list(self.data[col])
                    if len(values) == len(time_values):
                        self.ax.plot(time_values, values, label=col, linewidth=1.5, alpha=0.8)
            
            self.ax.legend(loc='upper left', fontsize=8, ncol=2)
            self.canvas.draw()
    
    def log_error(self, message):
        """Add message to error log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.error_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.error_text.see(tk.END)
    
    def clear_log(self):
        """Clear the error log"""
        self.error_text.delete(1.0, tk.END)
    
    def select_all(self):
        """Select all data series"""
        for var in self.visibility.values():
            var.set(True)
    
    def clear_all(self):
        """Deselect all data series"""
        for var in self.visibility.values():
            var.set(False)
    
    def load_hitl_file(self):
        """Load CSV file for HITL testing"""
        filename = filedialog.askopenfilename(
            title="Select HITL CSV File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                
                # Read headers from first line
                self.hitl_headers = next(reader)
                
                # Read all data rows
                self.hitl_data = []
                for row in reader:
                    if row:  # Skip empty rows
                        self.hitl_data.append(row)
                
                # Update UI
                base_name = os.path.basename(filename)
                self.hitl_file_path.set(base_name)
                self.hitl_btn.config(state=tk.NORMAL)
                self.log_error(f"Loaded {len(self.hitl_data)} rows from {base_name}")
                self.log_error(f"Headers: {', '.join(self.hitl_headers[:5])}...")  # Show first 5 headers
                
        except Exception as e:
            self.log_error(f"Error loading HITL file: {str(e)}")
            self.hitl_file_path.set("Error loading file")
            self.hitl_btn.config(state=tk.DISABLED)
    
    def toggle_hitl(self):
        """Start or stop HITL playback"""
        if not self.hitl_active:
            self.start_hitl()
        else:
            self.stop_hitl()
    
    def start_hitl(self):
        """Start HITL data playback"""
        if not self.serial_conn:
            self.log_error("Error: Not connected to serial port")
            return
        
        if not self.hitl_data:
            self.log_error("Error: No HITL data loaded")
            return
        
        self.hitl_active = True
        self.hitl_btn.config(text="Stop HITL")
        self.connect_btn.config(state=tk.DISABLED)
        
        mode = self.hitl_mode.get()
        self.log_error(f"Starting HITL in {mode} mode at {self.hitl_playback_speed.get():.1f}x speed")
        
        # Send mode flag to FC
        try:
            mode_msg = f"HITL_MODE:{mode}\n"
            self.serial_conn.write(mode_msg.encode('utf-8'))
            time.sleep(0.1)  # Give FC time to process
        except Exception as e:
            self.log_error(f"Error sending HITL mode: {str(e)}")
        
        # Start playback thread
        self.hitl_thread = threading.Thread(target=self.hitl_playback_worker, daemon=True)
        self.hitl_thread.start()
    
    def stop_hitl(self):
        """Stop HITL data playback"""
        self.hitl_active = False
        if self.hitl_thread:
            self.hitl_thread.join(timeout=1)
        
        self.hitl_btn.config(text="Start HITL")
        self.connect_btn.config(state=tk.NORMAL)
        self.log_error("HITL playback stopped")
        
        # Send stop signal to FC
        try:
            if self.serial_conn:
                stop_msg = "HITL_MODE:STOP\n"
                self.serial_conn.write(stop_msg.encode('utf-8'))
        except Exception as e:
            self.log_error(f"Error sending HITL stop: {str(e)}")
    
    def hitl_playback_worker(self):
        """Worker thread for HITL data playback"""
        while self.hitl_active:
            try:
                # Send header first
                header_line = ','.join(self.hitl_headers) + '\n'
                self.serial_conn.write(header_line.encode('utf-8'))
                
                # Send each data row
                for i, row in enumerate(self.hitl_data):
                    if not self.hitl_active:
                        break
                    
                    # Send data line
                    data_line = ','.join(row) + '\n'
                    self.serial_conn.write(data_line.encode('utf-8'))
                    
                    # Calculate delay between rows based on playback speed
                    # If there's a time column, use actual timing, otherwise use fixed rate
                    if i < len(self.hitl_data) - 1:
                        try:
                            # Try to use timestamp for realistic timing
                            current_time = float(row[0])
                            next_time = float(self.hitl_data[i + 1][0])
                            delay = (next_time - current_time) / 1000000.0  # Convert microseconds to seconds
                            delay = max(delay, 0.001)  # Minimum 1ms delay
                            delay = delay / self.hitl_playback_speed.get()  # Apply speed multiplier
                        except (ValueError, IndexError):
                            # Fallback to fixed rate if timing isn't available
                            delay = 0.01 / self.hitl_playback_speed.get()  # 100Hz base rate
                        
                        time.sleep(delay)
                
                # Check if we should loop
                if not self.hitl_loop.get():
                    self.hitl_active = False
                    self.root.after(0, lambda: self.hitl_btn.config(text="Start HITL"))
                    self.root.after(0, lambda: self.connect_btn.config(state=tk.NORMAL))
                    self.log_error("HITL playback completed")
                else:
                    self.log_error("HITL playback looping...")
                    time.sleep(0.5)  # Brief pause between loops
                    
            except Exception as e:
                self.log_error(f"HITL playback error: {str(e)}")
                self.hitl_active = False
                self.root.after(0, lambda: self.hitl_btn.config(text="Start HITL"))
                self.root.after(0, lambda: self.connect_btn.config(state=tk.NORMAL))
                break
    
    def on_closing(self):
        """Handle window closing"""
        if self.hitl_active:
            self.stop_hitl()
        self.disconnect()
        self.root.quit()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = FlightComputerMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
