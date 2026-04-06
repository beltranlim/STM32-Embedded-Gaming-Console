import sys
import serial
import serial.tools.list_ports
import threading
import queue
import re
from collections import deque
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QTextEdit, QGroupBox, QGridLayout, QScrollArea, QStackedWidget)
from PyQt5.QtCore import QTimer, Qt, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import numpy as np


class SerialReader(threading.Thread):
    """Thread for reading serial data without blocking GUI"""
    def __init__(self, port, baudrate, data_queue):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.running = False
        self.serial_port = None
        self.daemon = True
    
    def run(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            print(f"Connected to {self.port} at {self.baudrate} baud")
            
            while self.running:
                if self.serial_port.in_waiting:
                    try:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.data_queue.put(line)
                    except Exception as e:
                        print(f"Read error: {e}")
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")
            self.running = False
    
    def stop(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


class DataParser:
    """Parse incoming serial messages"""
    def __init__(self):
        self.patterns = {
            'temp_pres_humi': re.compile(r'Temperature:\s*([\d.]+),\s*Pressure:\s*([\d.]+),\s*Humidity:\s*([\d.]+)'),
            'accel': re.compile(r'Accel X\s*:\s*([-\d.]+);\s*Accel Y\s*:\s*([-\d.]+);\s*Accel Z\s*:\s*([-\d.]+)'),
            'gyro': re.compile(r'Gyro X\s*:\s*([-\d.]+);\s*Gyro Y\s*:\s*([-\d.]+);\s*Gyro Z\s*:\s*([-\d.]+)'),
            'magneto': re.compile(r'Magneto X\s*:\s*([-\d.]+);\s*Magneto Y\s*:\s*([-\d.]+);\s*Magneto Z\s*:\s*([-\d.]+)'),
            'magneto_baseline': re.compile(r'Baseline captured: M\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
            'temp_pres_humi_cnr': re.compile(r'Temp:\s*(\d+)C,\s*Pres:\s*(\d+)\s*hPa,\s*Humi:\s*(\d+)%'),
            'proximity': re.compile(r'Proximity Level:\s*(\d)'),
            'escaped': re.compile(r'You escaped'),
            'mode': re.compile(r'Entering (.*?) as Player'),
            'red_light': re.compile(r'Red Light!'),
            'green_light': re.compile(r'Green Light!'),
            'temp_spike': re.compile(r'Temperature (spike|drop) detected! T:\s*(\d+)C'),
            'pres_spike': re.compile(r'Pressure (spike|drop) detected! P:\s*(\d+)'),
            'humi_spike': re.compile(r'Humidity (spike|drop) detected! [HL]:\s*(\d+)'),
            'game_over': re.compile(r'GAME OVER'),
            'tilt': re.compile(r'Tilt: (LEFT|RIGHT)'),
            'debug_interrupt': re.compile(r'Debug: Tilt Interrupt Triggered!'),
            'debug_baseline': re.compile(r'Debug: Baseline Acc X: ([-\d.]+)'),
            'debug_current': re.compile(r'Debug: Current Acc X: ([-\d.]+)'),
        }
    
    def parse(self, line):
        """Parse a line and return data type and values"""
        for key, pattern in self.patterns.items():
            match = pattern.search(line)
            if match:
                return key, match.groups()
        return None, None


class STM32Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 IoT Dashboard - EE2028")
        self.setGeometry(100, 100, 1400, 900)
        
        # Data structures
        self.data_queue = queue.Queue()
        self.serial_thread = None
        self.parser = DataParser()
        
        # Data buffers (store last 50 points for stability)
        self.max_points = 50  # Reduced from 100 to prevent crashes
        self.time_data = deque(maxlen=self.max_points)
        self.temp_data = deque(maxlen=self.max_points)
        self.pres_data = deque(maxlen=self.max_points)
        self.humi_data = deque(maxlen=self.max_points)
        self.accel_x = deque(maxlen=self.max_points)
        self.accel_y = deque(maxlen=self.max_points)
        self.accel_z = deque(maxlen=self.max_points)
        self.gyro_x = deque(maxlen=self.max_points)
        self.gyro_y = deque(maxlen=self.max_points)
        self.gyro_z = deque(maxlen=self.max_points)
        self.mag_x = deque(maxlen=self.max_points)
        self.mag_y = deque(maxlen=self.max_points)
        self.mag_z = deque(maxlen=self.max_points)
        
        self.start_time = datetime.now()
        
        # Current state
        self.current_mode = "Unknown"
        self.current_proximity = 0
        
        # Setup UI
        self.init_ui()
        
        # Create game over overlay (initially hidden)
        self.create_game_over_overlay()
        
        # Timer for updating GUI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start(100)  # Update every 100ms
    
    def init_ui(self):
        """Initialize the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Connection panel
        conn_group = self.create_connection_panel()
        main_layout.addWidget(conn_group)
        
        # Status panel
        status_group = self.create_status_panel()
        main_layout.addWidget(status_group)
        
        # Create horizontal layout for plots
        plots_layout = QHBoxLayout()
        
        # Left column - Environmental sensors
        left_plots = QVBoxLayout()
        self.temp_plot = self.create_plot("Temperature (°C)", "red")
        self.pres_plot = self.create_plot("Pressure (mbar)", "blue")
        self.humi_plot = self.create_plot("Humidity (%)", "green")
        left_plots.addWidget(self.temp_plot)
        left_plots.addWidget(self.pres_plot)
        left_plots.addWidget(self.humi_plot)
        
        # Right column - Motion sensors
        right_plots = QVBoxLayout()
        self.accel_plot = self.create_multi_plot("Accelerometer (mg)", ["X", "Y", "Z"])
        self.gyro_plot = self.create_multi_plot("Gyroscope (dps)", ["X", "Y", "Z"])
        self.mag_plot = self.create_multi_plot("Magnetometer (gauss)", ["X", "Y", "Z"])
        right_plots.addWidget(self.accel_plot)
        right_plots.addWidget(self.gyro_plot)
        right_plots.addWidget(self.mag_plot)
        
        plots_layout.addLayout(left_plots)
        plots_layout.addLayout(right_plots)

        # Create a container for the plots
        self.plot_widget_container = QWidget()
        self.plot_widget_container.setLayout(plots_layout)

        # Create the spaceship game view
        self.create_spaceship_game_view()

        # Create a stacked layout to switch between plots and game view
        self.view_stack = QStackedWidget()
        self.view_stack.addWidget(self.plot_widget_container)
        self.view_stack.addWidget(self.spaceship_game_view)

        main_layout.addWidget(self.view_stack)
        
        # Message log
        log_group = QGroupBox("Message Log")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setFont(QFont("Courier", 9))
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
    
    def create_spaceship_game_view(self):
        """Creates the view for the spaceship game"""
        self.spaceship_game_view = QWidget()
        layout = QVBoxLayout(self.spaceship_game_view)
        layout.setAlignment(Qt.AlignCenter)

        title = QLabel("Spaceship Tilt Game")
        title.setFont(QFont("Arial", 24, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # This label will act as our spaceship
        self.spaceship_label = QLabel("🚀")
        self.spaceship_label.setFont(QFont("Arial", 40))
        
        # A container widget to allow absolute positioning of the spaceship
        self.game_area = QWidget()
        self.game_area.setMinimumSize(600, 200)
        self.game_area.setStyleSheet("background-color: #1a1a2e; border-radius: 10px;")
        self.spaceship_label.setParent(self.game_area)
        self.spaceship_label.move(270, 80) # Start in the middle

        layout.addWidget(self.game_area)

    def create_game_over_overlay(self):
        """Create a game over overlay that appears on top of everything"""
        # Create overlay widget
        self.game_over_overlay = QWidget(self)
        self.game_over_overlay.setStyleSheet("""
            QWidget {
                background-color: rgba(0, 0, 0, 200);
            }
        """)
        self.game_over_overlay.setGeometry(0, 0, self.width(), self.height())
        
        # Create layout for overlay content
        overlay_layout = QVBoxLayout(self.game_over_overlay)
        overlay_layout.setAlignment(Qt.AlignCenter)
        
        # Game Over text
        game_over_label = QLabel("💀 GAME OVER 💀")
        game_over_label.setStyleSheet("""
            QLabel {
                color: #ff0000;
                font-size: 72px;
                font-weight: bold;
                background-color: transparent;
                border: 4px solid #ff0000;
                border-radius: 15px;
                padding: 20px;
            }
        """)
        game_over_label.setAlignment(Qt.AlignCenter)
        
        # You Lose text
        you_lose_label = QLabel("YOU LOSE!")
        you_lose_label.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 48px;
                font-weight: bold;
                background-color: rgba(255, 0, 0, 100);
                border: 3px solid #ffffff;
                border-radius: 10px;
                padding: 15px;
                margin-top: 20px;
            }
        """)
        you_lose_label.setAlignment(Qt.AlignCenter)
        
        # Dismiss button
        dismiss_btn = QPushButton("Click to Continue")
        dismiss_btn.setStyleSheet("""
            QPushButton {
                background-color: #667eea;
                color: white;
                font-size: 24px;
                font-weight: bold;
                padding: 20px 40px;
                border-radius: 10px;
                border: 3px solid #ffffff;
                margin-top: 30px;
            }
            QPushButton:hover {
                background-color: #5568d3;
                border: 3px solid #ffff00;
            }
            QPushButton:pressed {
                background-color: #4556b8;
            }
        """)
        dismiss_btn.clicked.connect(self.hide_game_over)
        dismiss_btn.setMaximumWidth(300)
        
        # Add widgets to layout
        overlay_layout.addWidget(game_over_label)
        overlay_layout.addSpacing(30)
        overlay_layout.addWidget(you_lose_label)
        overlay_layout.addSpacing(20)
        overlay_layout.addWidget(dismiss_btn, alignment=Qt.AlignCenter)
        
        # Initially hide the overlay
        self.game_over_overlay.hide()
    
    def show_game_over(self):
        """Show the game over overlay with animation"""
        # Resize overlay to match window size
        self.game_over_overlay.setGeometry(0, 0, self.width(), self.height())
        
        # Show the overlay
        self.game_over_overlay.show()
        self.game_over_overlay.raise_()  # Bring to front
        
        # Optional: Add fade-in animation
        self.game_over_overlay.setWindowOpacity(0)
        self.fade_animation = QPropertyAnimation(self.game_over_overlay, b"windowOpacity")
        self.fade_animation.setDuration(500)
        self.fade_animation.setStartValue(0)
        self.fade_animation.setEndValue(1)
        self.fade_animation.setEasingCurve(QEasingCurve.InOutQuad)
        self.fade_animation.start()
    
    def hide_game_over(self):
        """Hide the game over overlay"""
        self.game_over_overlay.hide()
    
    def resizeEvent(self, event):
        """Handle window resize to adjust overlay size"""
        super().resizeEvent(event)
        if hasattr(self, 'game_over_overlay'):
            self.game_over_overlay.setGeometry(0, 0, self.width(), self.height())

    def create_connection_panel(self):
        """Create the connection control panel"""
        group = QGroupBox("Connection")
        layout = QHBoxLayout()
        
        # COM port selection
        layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)
        
        # Refresh button
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)
        
        # Baud rate
        layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "230400", "460800"])
        self.baud_combo.setCurrentText("115200")
        layout.addWidget(self.baud_combo)
        
        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        # Status label
        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.conn_status)
        
        # Add separator
        layout.addSpacing(20)
        separator = QLabel("|")
        separator.setStyleSheet("color: gray; font-size: 16px;")
        layout.addWidget(separator)
        layout.addSpacing(20)
        
        # Clear All button
        clear_btn = QPushButton("🗑️ Clear All")
        clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
            QPushButton:pressed {
                background-color: #a93226;
            }
        """)
        clear_btn.clicked.connect(self.clear_all)
        layout.addWidget(clear_btn)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
    
    def create_status_panel(self):
        """Create the game status panel"""
        group = QGroupBox("Game Status")
        layout = QHBoxLayout()  # Changed from QGridLayout to QHBoxLayout
        
        # Mode section
        mode_section = QVBoxLayout()
        mode_label_header = QLabel("Current Mode:")
        mode_label_header.setFont(QFont("Arial", 10))
        mode_section.addWidget(mode_label_header)
        self.mode_label = QLabel("Unknown")
        self.mode_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.mode_label.setStyleSheet("color: #667eea; padding: 10px; background-color: #f0f0f0; border-radius: 5px;")
        self.mode_label.setAlignment(Qt.AlignCenter)
        mode_section.addWidget(self.mode_label)
        
        # Proximity section
        prox_section = QVBoxLayout()
        prox_label_header = QLabel("Proximity Level:")
        prox_label_header.setFont(QFont("Arial", 10))
        prox_section.addWidget(prox_label_header)
        
        prox_layout = QHBoxLayout()
        self.prox_indicators = []
        colors = ["#FFA500", "#FF6347", "#FF0000"]  # Orange, Tomato, Red
        for i in range(3):
            label = QLabel(f"L{i+1}")
            label.setAlignment(Qt.AlignCenter)
            label.setFixedSize(60, 60)
            label.setStyleSheet(f"background-color: gray; border: 2px solid black; border-radius: 5px; font-weight: bold; font-size: 14px;")
            self.prox_indicators.append(label)
            prox_layout.addWidget(label)
        
        prox_section.addLayout(prox_layout)
        
        # Add both sections to main layout with spacing
        layout.addLayout(mode_section)
        layout.addSpacing(30)
        layout.addLayout(prox_section)
        layout.addSpacing(30)

        # Debug section for tilt game
        debug_section = QVBoxLayout()
        debug_label_header = QLabel("Tilt Debug Info:")
        debug_label_header.setFont(QFont("Arial", 10))
        debug_section.addWidget(debug_label_header)

        self.baseline_accel_label = QLabel("Baseline X: N/A")
        self.current_accel_label = QLabel("Current X: N/A")
        self.interrupt_status_label = QLabel("Interrupt: Idle")
        
        debug_font = QFont("Courier", 10)
        self.baseline_accel_label.setFont(debug_font)
        self.current_accel_label.setFont(debug_font)
        self.interrupt_status_label.setFont(debug_font)
        self.interrupt_status_label.setStyleSheet("background-color: #DDDDDD; padding: 2px; border-radius: 3px;")

        debug_section.addWidget(self.baseline_accel_label)
        debug_section.addWidget(self.current_accel_label)
        debug_section.addWidget(self.interrupt_status_label)
        
        layout.addLayout(debug_section)
        layout.addStretch()
        
        self.msg_count = 0
        
        group.setLayout(layout)
        return group
    
    def create_plot(self, title, color):
        """Create a single line plot"""
        plot_widget = pg.PlotWidget()
        plot_widget.setBackground('w')
        plot_widget.setTitle(title, color='k', size='12pt')
        plot_widget.setLabel('left', title)
        plot_widget.setLabel('bottom', 'Time (s)')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        plot_widget.addLegend()
        
        # Add plot line
        plot_widget.plot_line = plot_widget.plot([], [], pen=pg.mkPen(color, width=2), name=title)
        
        return plot_widget
    
    def create_multi_plot(self, title, labels):
        """Create a multi-line plot"""
        plot_widget = pg.PlotWidget()
        plot_widget.setBackground('w')
        plot_widget.setTitle(title, color='k', size='12pt')
        plot_widget.setLabel('left', title)
        plot_widget.setLabel('bottom', 'Time (s)')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        plot_widget.addLegend()
        
        # Add plot lines
        colors = ['r', 'g', 'b']
        plot_widget.plot_lines = []
        for i, label in enumerate(labels):
            line = plot_widget.plot([], [], pen=pg.mkPen(colors[i], width=2), name=label)
            plot_widget.plot_lines.append(line)
        
        return plot_widget
    
    def refresh_ports(self):
        """Refresh available COM ports"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
    
    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_thread and self.serial_thread.running:
            # Disconnect
            self.serial_thread.stop()
            self.serial_thread.join(timeout=1)
            self.serial_thread = None
            self.connect_btn.setText("Connect")
            self.conn_status.setText("Disconnected")
            self.conn_status.setStyleSheet("color: red; font-weight: bold;")
        else:
            # Connect
            port_text = self.port_combo.currentText()
            if not port_text:
                self.log_message("ERROR: No COM port selected", "red")
                return
            
            port = port_text.split(" - ")[0]
            baudrate = int(self.baud_combo.currentText())
            
            self.serial_thread = SerialReader(port, baudrate, self.data_queue)
            self.serial_thread.start()
            
            self.connect_btn.setText("Disconnect")
            self.conn_status.setText("Connected")
            self.conn_status.setStyleSheet("color: green; font-weight: bold;")
            self.log_message(f"Connected to {port} at {baudrate} baud", "green")
            
            # Reset data and time
            self.start_time = datetime.now()
            self.clear_data()
    
    def clear_data(self):
        """Clear all data buffers"""
        self.time_data.clear()
        self.temp_data.clear()
        self.pres_data.clear()
        self.humi_data.clear()
        self.accel_x.clear()
        self.accel_y.clear()
        self.accel_z.clear()
        self.gyro_x.clear()
        self.gyro_y.clear()
        self.gyro_z.clear()
        self.mag_x.clear()
        self.mag_y.clear()
        self.mag_z.clear()
    
    def clear_all(self):
        """Clear all data buffers, plots, and message log"""
        # Clear data buffers
        self.clear_data()
        
        # Clear message log
        self.log_text.clear()
        
        # Reset time reference
        self.start_time = datetime.now()
        
        # Update all plots to show empty data
        self.update_plots()
        
        # Log the clear action
        self.log_message("All data and logs cleared", "blue")
    
    def update_gui(self):
        """Update GUI with new data from queue"""
        # Process all messages in queue
        while not self.data_queue.empty():
            try:
                line = self.data_queue.get_nowait()
                self.process_message(line)
            except queue.Empty:
                break
        
        # Update plots
        self.update_plots()
    
    def process_message(self, line):
        """Process a single message line"""
        self.msg_count += 1
        
        # Parse the message
        msg_type, values = self.parser.parse(line)
        
        current_time = (datetime.now() - self.start_time).total_seconds()
        
        if msg_type == 'temp_pres_humi':
            temp, pres, humi = map(float, values)
            self.time_data.append(current_time)
            self.temp_data.append(temp)
            self.pres_data.append(pres)
            self.humi_data.append(humi)
            self.log_message(line, "blue")
        
        elif msg_type == 'temp_pres_humi_cnr':
            # This is the Catch & Run format: "Temp: XC, Pres: Y hPa, Humi: Z%"
            temp, pres, humi = map(float, values)
            if len(self.time_data) == 0 or current_time != self.time_data[-1]:
                self.time_data.append(current_time)
            self.temp_data.append(temp)
            self.pres_data.append(pres)
            self.humi_data.append(humi)
            # Don't log to message log - just plot
        
        elif msg_type == 'accel':
            x, y, z = map(float, values)
            if len(self.accel_x) == 0 or current_time != self.time_data[-1]:
                self.time_data.append(current_time)
            self.accel_x.append(x)
            self.accel_y.append(y)
            self.accel_z.append(z)
            # Don't log accelerometer data to reduce clutter
        
        elif msg_type == 'gyro':
            x, y, z = map(float, values)
            if len(self.gyro_x) == 0 or current_time != self.time_data[-1]:
                self.time_data.append(current_time)
            self.gyro_x.append(x)
            self.gyro_y.append(y)
            self.gyro_z.append(z)
            # Don't log gyroscope data to reduce clutter
        
        elif msg_type == 'magneto':
            # This is the new format: "Magneto X : X; Magneto Y : Y; Magneto Z : Z"
            x, y, z = map(float, values)
            if len(self.mag_x) == 0 or current_time != self.time_data[-1]:
                self.time_data.append(current_time)
            self.mag_x.append(x)
            self.mag_y.append(y)
            self.mag_z.append(z)
            # Don't log magnetometer data - it's too frequent
        
        elif msg_type == 'magneto_baseline':
            # Baseline message for magnetometer
            x, y, z = map(float, values)
            # Don't log baseline messages
        
        elif msg_type == 'proximity':
            level = int(values[0])
            self.current_proximity = level
            self.update_proximity_indicators()
            self.log_message(line, "orange")
        
        elif msg_type == 'escaped':
            self.current_proximity = 0
            self.update_proximity_indicators()
            self.log_message(line, "green")
        
        elif msg_type == 'mode':
            self.current_mode = values[0].strip()
            self.mode_label.setText(self.current_mode)
            self.log_message(line, "darkblue")

            # Switch view based on mode
            if "Spaceship" in self.current_mode:
                self.view_stack.setCurrentWidget(self.spaceship_game_view)
            else:
                self.view_stack.setCurrentWidget(self.plot_widget_container)
        
        elif msg_type == 'red_light':
            # Update mode to RLGL if not already set
            if self.current_mode != "Red Light, Green Light":
                self.current_mode = "Red Light, Green Light"
                self.mode_label.setText(self.current_mode)
            self.log_message(line, "red")
        
        elif msg_type == 'green_light':
            # Update mode to RLGL if not already set
            if self.current_mode != "Red Light, Green Light":
                self.current_mode = "Red Light, Green Light"
                self.mode_label.setText(self.current_mode)
            self.log_message(line, "green")
        
        elif msg_type == 'game_over':
            self.log_message(line, "red")
            # Show the game over overlay
            self.show_game_over()
        
        elif msg_type == 'tilt':
            direction = values[0]
            current_pos = self.spaceship_label.pos()
            new_x = current_pos.x()
            
            if direction == 'LEFT':
                new_x -= 25
            else: # RIGHT
                new_x += 25
            
            # Boundary check
            if new_x < 0: new_x = 0
            if new_x > self.game_area.width() - self.spaceship_label.width():
                new_x = self.game_area.width() - self.spaceship_label.width()

            self.spaceship_label.move(new_x, current_pos.y())
            self.log_message(line, "purple")

        elif msg_type == 'debug_interrupt':
            self.interrupt_status_label.setText("Interrupt: TRIGGERED!")
            self.interrupt_status_label.setStyleSheet("background-color: #FFD700; color: black; font-weight: bold; padding: 2px; border-radius: 3px;")
            self.log_message(line, "#FF8C00") # DarkOrange
            # Reset status after a short time
            QTimer.singleShot(500, lambda: self.interrupt_status_label.setStyleSheet("background-color: #DDDDDD; padding: 2px; border-radius: 3px;"))
            QTimer.singleShot(500, lambda: self.interrupt_status_label.setText("Interrupt: Idle"))

        elif msg_type == 'debug_baseline':
            self.baseline_accel_label.setText(f"Baseline X: {values[0]}")
            self.log_message(line, "#006400") # DarkGreen

        elif msg_type == 'debug_current':
            self.current_accel_label.setText(f"Current X: {values[0]}")
            # Don't log this to the main log, it's too frequent. It's visible in the status panel.

        elif msg_type in ['temp_spike', 'pres_spike', 'humi_spike']:
            self.log_message(line, "red")
        
        else:
            # Log unrecognized messages (but skip sensor data and baseline messages)
            if not any(keyword in line.lower() for keyword in ['accel', 'gyro', 'magneto', 'baseline', 'skipping', 'temp:', 'pres:', 'humi:']):
                self.log_message(line, "gray")
    
    def update_proximity_indicators(self):
        """Update proximity level visual indicators"""
        colors = ["#FFA500", "#FF6347", "#FF0000"]  # Orange, Tomato, Red
        for i in range(3):
            if i < self.current_proximity:
                self.prox_indicators[i].setStyleSheet(
                    f"background-color: {colors[i]}; border: 2px solid black; border-radius: 5px; font-weight: bold;"
                )
            else:
                self.prox_indicators[i].setStyleSheet(
                    "background-color: gray; border: 2px solid black; border-radius: 5px;"
                )
    
    def update_plots(self):
        """Update all plot widgets"""
        if len(self.time_data) > 0:
            time_array = np.array(self.time_data)
            
            # Update environmental sensors
            if len(self.temp_data) > 0:
                self.temp_plot.plot_line.setData(time_array[-len(self.temp_data):], np.array(self.temp_data))
            if len(self.pres_data) > 0:
                self.pres_plot.plot_line.setData(time_array[-len(self.pres_data):], np.array(self.pres_data))
            if len(self.humi_data) > 0:
                self.humi_plot.plot_line.setData(time_array[-len(self.humi_data):], np.array(self.humi_data))
            
            # Update accelerometer
            if len(self.accel_x) > 0:
                time_accel = time_array[-len(self.accel_x):]
                self.accel_plot.plot_lines[0].setData(time_accel, np.array(self.accel_x))
                self.accel_plot.plot_lines[1].setData(time_accel, np.array(self.accel_y))
                self.accel_plot.plot_lines[2].setData(time_accel, np.array(self.accel_z))
            
            # Update gyroscope
            if len(self.gyro_x) > 0:
                time_gyro = time_array[-len(self.gyro_x):]
                self.gyro_plot.plot_lines[0].setData(time_gyro, np.array(self.gyro_x))
                self.gyro_plot.plot_lines[1].setData(time_gyro, np.array(self.gyro_y))
                self.gyro_plot.plot_lines[2].setData(time_gyro, np.array(self.gyro_z))
            
            # Update magnetometer
            if len(self.mag_x) > 0:
                time_mag = time_array[-len(self.mag_x):]
                self.mag_plot.plot_lines[0].setData(time_mag, np.array(self.mag_x))
                self.mag_plot.plot_lines[1].setData(time_mag, np.array(self.mag_y))
                self.mag_plot.plot_lines[2].setData(time_mag, np.array(self.mag_z))
    
    def log_message(self, message, color="black"):
        """Add message to log with color"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_msg = f'<span style="color:{color};">[{timestamp}] {message}</span>'
        self.log_text.append(formatted_msg)
        
        # Auto-scroll to bottom
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def closeEvent(self, event):
        """Handle window close event"""
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.stop()
            self.serial_thread.join(timeout=1)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    dashboard = STM32Dashboard()
    dashboard.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()