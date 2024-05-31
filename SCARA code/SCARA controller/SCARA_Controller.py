import sys
import math
import time
import threading
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtChart import QChart, QChartView, QLineSeries
from waypoints import Waypoint, parse_file

try:
    import serial
    SERIAL_PORT = '/dev/cu.usbserial-0001'
    BAUDRATE = 115200

    try:
        arduino = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        arduino_connected = True
    except serial.SerialException:
        print("Arduino not connected. Running in simulation mode.")
        arduino_connected = False
except ImportError:
    print("PySerial library not found. Running in simulation mode.")
    arduino_connected = False

WIDTH = 800
HEIGHT = 800
BLOCK_WIDTH = 80
ORIGIN = (WIDTH // 2, HEIGHT // 2)
ARM_SEGMENT_ONE = 160
ARM_SEGMENT_TWO = 202
ARM_SEGMENT_THREE = 40
MAX_Z = 758
FPS = 60
WHEEL_EVENT_INTERVAL = 100

class Dot:
    def __init__(self, pos):
        self.pos = list(pos)
        self.radius = 10
        self.color = QtGui.QColor(255, 0, 0)

    def draw(self, painter):
        painter.setBrush(self.color)
        painter.drawEllipse(QtCore.QPointF(*self.pos), self.radius, self.radius)

    def is_moused_over(self, mouse_pos):
        return math.dist(mouse_pos, self.pos) < self.radius

class PrintLogger:
    def __init__(self, widget):
        self.widget = widget

    def write(self, message):
        self.widget.append(message)

    def flush(self):
        pass


class ArmCanvas(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.pointer = Dot(ORIGIN)
        self.angles = [0, 0, 0]
        self.z_height = 0
        self.dragging = False
        self.gripped = False
        self.stored_path = []
        self.saving_path = False
        self.current_waypoint = 0
        self.current_state = 1
        self.arduino_ready = True
        self.update_arduino_count = 0
        self.last_wheel_event_time = 0
        self.use_wasd_controls = True
        self.end_effector_mode = 'free'
        self.relative_theta_3_adjustment = 0
        self.baseline_theta_3 = 0
        self.data_packet = ""
        self.lock = threading.Lock()
        self.data_history = []
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.initUI()
        self.start_sending_thread()
        self.start_serial_reading_thread()

    def start_serial_reading_thread(self):
        thread = threading.Thread(target=self.read_serial_data)
        thread.daemon = True
        thread.start()

    def read_serial_data(self):
        while True:
            if arduino_connected and self.main_window.arduino.in_waiting:
                s = self.main_window.arduino.readline().decode().strip()
                print(s)  # This prints to the console for debugging
                if s in ("Ready", "Close", "Open"): 
                    self.arduino_ready = True
                    self.update()
                    # Invoke the update method on the main thread
                    QtCore.QMetaObject.invokeMethod(self, "update", QtCore.Qt.QueuedConnection)
                self.main_window.serial_monitor.append(s)  # Append to Serial Monitor for visibility
                self.main_window.print_monitor.append(s)  # Append to Print Monitor for visibility
            time.sleep(0.01)  # Small sleep to prevent excessive CPU usage

    def update(self):
        super().update()
        if self.current_state in (2, 3) and self.arduino_ready:
            self.current_state = 2
            self.process_waypoint()
        # Additional UI updates if necessary

    def initUI(self):
        self.building_path_label = QtWidgets.QLabel(self)
        self.building_path_label.setStyleSheet("color: red; font-size: 16px;")
        self.building_path_label.setGeometry(30, 30, 200, 30)
        self.update_building_path_label()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        self.draw_grid(painter)
        self.draw_arm(painter)
        self.pointer.draw(painter)

    def draw_grid(self, painter):
        pen = QtGui.QPen(QtGui.QColor(200, 200, 200), 1, QtCore.Qt.DotLine)
        painter.setPen(pen)
        for x in range(WIDTH // BLOCK_WIDTH):
            painter.drawLine((x + 1) * BLOCK_WIDTH, 0, (x + 1) * BLOCK_WIDTH, HEIGHT)
        for y in range(HEIGHT // BLOCK_WIDTH):
            painter.drawLine(0, (y + 1) * BLOCK_WIDTH, WIDTH, (y + 1) * BLOCK_WIDTH)

    def draw_arm(self, painter):
        pen = QtGui.QPen(QtGui.QColor(0, 0, 0), 10)
        painter.setPen(pen)
        end = self.draw_joint(painter, ORIGIN, self.angles[0], ARM_SEGMENT_ONE, 10)
        pen.setWidth(5)
        painter.setPen(pen)
        end = self.draw_joint(painter, end, self.angles[0] + self.angles[1], ARM_SEGMENT_TWO, 5)
        pen.setWidth(2)
        painter.setPen(pen)
        self.draw_joint(painter, end, self.angles[0] + self.angles[1] + self.angles[2], ARM_SEGMENT_THREE, 2)

    def draw_joint(self, painter, start_pos, angle, length, width):
        x = start_pos[0] + math.cos(math.radians(angle - 90)) * length
        y = start_pos[1] + math.sin(math.radians(angle - 90)) * length
        end_pos = (x, y)
        painter.drawLine(QtCore.QPointF(*start_pos), QtCore.QPointF(*end_pos))
        return end_pos

    def mousePressEvent(self, event):
        if self.pointer.is_moused_over((event.x(), event.y())):
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            self.pointer.pos = [event.x(), event.y()]
            self.update_angles()
            self.update()

    def mouseReleaseEvent(self, event):
        self.dragging = False

    def wheelEvent(self, event):
        current_time = time.time()
        if current_time - self.last_wheel_event_time > WHEEL_EVENT_INTERVAL / 1000.0:
            z_height_change = event.angleDelta().y() / 120
            new_z_height = self.z_height - z_height_change
            if 0 <= new_z_height <= MAX_Z:
                self.z_height = int(new_z_height)
                self.write_data()
            self.last_wheel_event_time = current_time
            self.update_status()

    def process_waypoint(self):
        if self.current_state in (2, 3) and self.arduino_ready:
            if self.current_waypoint < len(self.stored_path):
                waypoint = self.stored_path[self.current_waypoint]
                if waypoint and waypoint.angles:
                    self.angles = list(waypoint.angles)  # Ensure angles are a list
                    self.z_height = waypoint.height
                    self.gripped = waypoint.grip
                    print(f"Processing waypoint {self.current_waypoint}: angles={self.angles}, z_height={self.z_height}, gripped={self.gripped}")
                    self.write_data()
                    self.arduino_ready = False
                    self.current_waypoint = (self.current_waypoint + 1) % len(self.stored_path)
                    if self.current_waypoint == 40 and self.current_state == 2:
                        self.current_state = 1
                else:
                    print(f"Invalid waypoint at index {self.current_waypoint}: {waypoint}")
            else:
                print(f"Waypoint index out of range: {self.current_waypoint}")

    def keyPressEvent(self, event):
        moved = False
        if event.key() == QtCore.Qt.Key_B:
            self.saving_path = True
            self.stored_path = []
            self.update_building_path_label()
        elif event.key() == QtCore.Qt.Key_Z and not event.isAutoRepeat():
            self.main_window.save_path()
        elif event.key() == QtCore.Qt.Key_Space and self.saving_path:
            self.stored_path.append(Waypoint(angles=self.angles[:], height=self.z_height, grip=self.gripped))
            self.update_building_path_label()
        elif event.key() == QtCore.Qt.Key_P and self.current_state == 1:
            self.main_window.load_path(2)
        elif event.key() == QtCore.Qt.Key_L and self.current_state == 1:
            self.main_window.load_path(3)
        elif event.key() == QtCore.Qt.Key_Escape:
            self.current_state = 1
            self.current_waypoint = 0
            self.update_building_path_label()
        elif event.key() == QtCore.Qt.Key_N:
            if self.current_state == 1:
                self.current_state = 2
            else:
                #self.current_waypoint += 1
                self.current_state = 2
                self.arduino_ready = True
                self.process_waypoint()  # Ensure the waypoint is processed immediately

        elif event.key() == QtCore.Qt.Key_Shift:
            self.gripped = not self.gripped
            self.write_data()
        elif event.key() == QtCore.Qt.Key_Left:
            if self.end_effector_mode == 'end_lock':
                self.relative_theta_3_adjustment -= 1
                moved = True
            else:
                self.angles[2] -= 2
            self.write_data()
        elif event.key() == QtCore.Qt.Key_Right:
            if self.end_effector_mode == 'end_lock':
                self.relative_theta_3_adjustment += 1
                moved = True
            else:
                self.angles[2] += 2
            self.write_data()

        if self.use_wasd_controls:
            if event.key() == QtCore.Qt.Key_W:
                self.pointer.pos[1] -= 5
                moved = True
            elif event.key() == QtCore.Qt.Key_S:
                self.pointer.pos[1] += 5
                moved = True
            elif event.key() == QtCore.Qt.Key_A:
                self.pointer.pos[0] -= 5
                moved = True
            elif event.key() == QtCore.Qt.Key_D:
                self.pointer.pos[0] += 5
                moved = True
        if moved:
            self.update_angles()
        self.angles[2] = self.normalize_angle(self.angles[2])
        self.update_status()
        self.update()

    def write_data(self):
        angles = self.angles[:]
        height = self.z_height
        gripped = int(self.gripped)
        data_packet = f"{int(angles[0])};{int(angles[1])};{int(angles[2])};{int(height)};{gripped};"
        with self.lock:
            self.data_packet = data_packet
        print(data_packet)

    def send_data(self):
        while True:
            with self.lock:
                if self.data_packet:
                    if len(self.data_history) >= 7 and all(packet == self.data_packet for packet in self.data_history[-7:]):
                        continue  # Skip sending if the last 7 packets are unchanged

                    self.data_history.append(self.data_packet)
                    if len(self.data_history) > 7:
                        self.data_history.pop(0)  # Maintain the history size to the last 7 packets

                    if arduino_connected:
                        self.main_window.arduino.write(self.data_packet.encode())
                        self.main_window.serial_monitor.append(f"Sent to Arduino: {self.data_packet}")
                    else:
                        self.main_window.serial_monitor.append(f"Simulated command: {self.data_packet}")
            time.sleep(0.25)  # Adjust the interval as needed

    def start_sending_thread(self):
        thread = threading.Thread(target=self.send_data)
        thread.daemon = True
        thread.start()

    def normalize_angle(self, angle):
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        return angle

    def toggle_end_effector_mode(self, mode):
        self.end_effector_mode = mode
        self.calculate_baseline_orientation()
        self.relative_theta_3_adjustment = 0
        self.update_angles()
        self.update()

    def calculate_baseline_orientation(self):
        if self.end_effector_mode == 'horizontal':
            self.baseline_theta_3 = 90 - (self.angles[0] + self.angles[1])
        elif self.end_effector_mode == 'vertical':
            self.baseline_theta_3 = -(self.angles[0] + self.angles[1])
        elif self.end_effector_mode == 'end_lock':
            self.baseline_theta_3 = -(self.angles[0] + self.angles[1])
        else:
            self.baseline_theta_3 = self.angles[2]  # For free mode, take current angle as baseline

    def update_angles(self):
        target_pos = (self.pointer.pos[0] - ORIGIN[0], ORIGIN[1] - self.pointer.pos[1])
        angles = self.inverse_kinematics(target_pos)
        if angles:
            self.angles[0], self.angles[1] = angles[0], angles[1]
            if self.end_effector_mode != 'free':
                self.angles[2] = angles[2]
        self.update_status()
        self.update_chart()
        self.update()
        self.write_data()

    def inverse_kinematics(self, target_pos):
        invert = False

        L1 = ARM_SEGMENT_ONE
        L2 = ARM_SEGMENT_TWO
        x = target_pos[0]
        y = target_pos[1]

        if x >= 0:
            invert = True
            x = -x

        r1 = math.sqrt(x ** 2 + y ** 2)
        try:
            phi_1 = math.acos((L2 ** 2 - L1 ** 2 - r1 ** 2) / (-2 * L1 * r1))
            phi_2 = math.atan2(y, x)
            theta_1 = math.degrees(phi_2 - phi_1) - 90
            phi_3 = math.acos((r1 ** 2 - L1 ** 2 - L2 ** 2) / (-2 * L1 * L2))
            theta_2 = 180 - math.degrees(phi_3)

            if invert:
                theta_1 = -theta_1
                theta_2 = -theta_2

            theta_1 = self.normalize_angle(theta_1)
            theta_2 = self.normalize_angle(theta_2)

            if self.end_effector_mode == 'horizontal':
                theta_3 = 90 - (theta_1 + theta_2)
                self.baseline_theta_3 = theta_3
            elif self.end_effector_mode == 'vertical':
                theta_3 = -(theta_1 + theta_2)
                self.baseline_theta_3 = theta_3
            elif self.end_effector_mode == 'end_lock':
                theta_3 = -(theta_1 + theta_2) + self.relative_theta_3_adjustment
            else:  # Free mode
                theta_3 = self.angles[2]  # Maintain the current angle in free mode

            theta_3 = self.normalize_angle(theta_3)

            return [-theta_1, -theta_2, -theta_3]  # Return as list to ensure mutability
        except ValueError:
            return None

    def update_building_path_label(self):
        if self.saving_path:
            self.building_path_label.setText(f"Building path: {len(self.stored_path)} waypoints")
            self.building_path_label.show()
        else:
            self.building_path_label.hide()

    def update_chart(self):
        self.main_window.update_chart(self.angles)

    def update_status(self):
        self.main_window.update_status()

    def toggle_controls(self):
        self.use_wasd_controls = not self.use_wasd_controls
        mode = "WASD" if self.use_wasd_controls else "Mouse"
        print(f"Control mode set to: {mode}")

    def export_waypoints(self, path, filename):
        try:
            with open(filename, "w") as f:
                for point in path:
                    point.write(f)
        except IOError as e:
            print(f"Error saving waypoints: {e}")

    def load_waypoints(self, filename):
        try:
            with open(filename, "r") as f:
                return parse_file(f.readlines())
        except IOError as e:
            print(f"Error loading waypoints: {e}")
            return []

    def save_path(self):
        filename = self.main_window.filename_input.text()
        if filename:
            self.export_waypoints(self.stored_path, f"{filename}.txt")

    def load_path(self, state):
        filename = self.main_window.filename_input.text()
        if filename:
            try:
                with open(f"{filename}.txt", "r") as f:
                    self.stored_path = parse_file(f.readlines())
                    self.current_state = state
                    self.current_waypoint = 0
                    self.update_building_path_label()
                    self.process_waypoint()
                    print(f'Loaded path from {filename}.txt')
            except IOError as e:
                print(f"Error loading waypoints: {e}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Robotic Arm Control')
        self.setGeometry(100, 100, WIDTH, HEIGHT)

        self.initSerialMonitor()  # Initialize the serial monitor first
        self.initPrintMonitor()   # Initialize the print monitor

        self.arduino = None
        self.connect_to_arduino()

        self.canvas = ArmCanvas(self)
        self.setCentralWidget(self.canvas)
        self.initUI()
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2E3440;
                color: #D8DEE9;
            }
            QPushButton {
                background-color: #4C566A;
                color: #D8DEE9;
                border: none;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #5E81AC;
            }
            QLineEdit {
                background-color: #4C566A;
                color: #D8DEE9;
                border: 1px solid #D8DEE9;
                border-radius: 5px;
                padding: 5px;
            }
            QLabel {
                color: #D8DEE9;
            }
        """)

    def initSerialMonitor(self):
        self.serial_monitor = QtWidgets.QTextEdit(self)
        self.serial_monitor.setReadOnly(True)
        self.serial_monitor.setFixedWidth(300)
        dock_widget = QtWidgets.QDockWidget("Serial Monitor", self)
        dock_widget.setWidget(self.serial_monitor)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock_widget)

    def initPrintMonitor(self):
        self.print_monitor = QtWidgets.QTextEdit(self)
        self.print_monitor.setReadOnly(True)
        self.print_monitor.setFixedWidth(300)
        dock_widget = QtWidgets.QDockWidget("Print Monitor", self)
        dock_widget.setWidget(self.print_monitor)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock_widget)
        sys.stdout = PrintLogger(self.print_monitor)

    def connect_to_arduino(self):
        global arduino_connected
        try:
            self.arduino = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
            arduino_connected = True
            self.serial_monitor.append("Arduino connected.")
        except serial.SerialException:
            arduino_connected = False
            self.serial_monitor.append("Arduino not connected. Running in simulation mode.")

    def reset_connection(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        self.connect_to_arduino()

    def initUI(self):
        control_panel = QtWidgets.QWidget(self)
        control_panel_layout = QtWidgets.QVBoxLayout()

        self.filename_input = QtWidgets.QLineEdit(self)
        self.filename_input.setPlaceholderText('Enter filename')

        save_button = QtWidgets.QPushButton('Save Path', self)
        save_button.clicked.connect(self.save_path)

        load_button_p = QtWidgets.QPushButton('Load Path P', self)
        load_button_p.clicked.connect(lambda: self.canvas.load_path(2))

        load_button_l = QtWidgets.QPushButton('Load Path L', self)
        load_button_l.clicked.connect(lambda: self.canvas.load_path(3))

        reset_button = QtWidgets.QPushButton('Reset Connection', self)
        reset_button.clicked.connect(self.reset_connection)

        toggle_mode_button = QtWidgets.QPushButton('Toggle Mode', self)
        toggle_mode_button.clicked.connect(self.toggle_mode)

        horizontal_button = QtWidgets.QPushButton('Set Horizontal', self)
        horizontal_button.clicked.connect(lambda: self.canvas.toggle_end_effector_mode('horizontal'))

        vertical_button = QtWidgets.QPushButton('Set Vertical', self)
        vertical_button.clicked.connect(lambda: self.canvas.toggle_end_effector_mode('vertical'))

        free_button = QtWidgets.QPushButton('Set Free', self)
        free_button.clicked.connect(lambda: self.canvas.toggle_end_effector_mode('free'))

        end_lock_button = QtWidgets.QPushButton('Set End Lock', self)
        end_lock_button.clicked.connect(lambda: self.canvas.toggle_end_effector_mode('end_lock'))

        show_controls_button = QtWidgets.QPushButton('Show Controls', self)
        show_controls_button.clicked.connect(self.show_controls)

        z_height_spinbox = QtWidgets.QSpinBox(self)
        z_height_spinbox.setRange(0, MAX_Z)
        z_height_spinbox.setFixedWidth(100)  # Shorten the width of the Z height input box
        z_height_spinbox.valueChanged.connect(lambda value: self.canvas.update_z_height(value))

        control_panel_layout.addWidget(self.filename_input)
        control_panel_layout.addWidget(save_button)
        control_panel_layout.addWidget(load_button_p)
        control_panel_layout.addWidget(load_button_l)
        control_panel_layout.addWidget(reset_button)
        control_panel_layout.addWidget(toggle_mode_button)
        control_panel_layout.addWidget(horizontal_button)
        control_panel_layout.addWidget(vertical_button)
        control_panel_layout.addWidget(free_button)
        control_panel_layout.addWidget(end_lock_button)
        control_panel_layout.addWidget(QtWidgets.QLabel("Z Height"))
        control_panel_layout.addWidget(z_height_spinbox)
        control_panel_layout.addWidget(show_controls_button)

        control_panel.setLayout(control_panel_layout)

        dock_widget = QtWidgets.QDockWidget("Controls", self)
        dock_widget.setWidget(control_panel)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, dock_widget)

        self.status_label = QtWidgets.QLabel(self)
        self.status_label.setGeometry(10, HEIGHT - 30, 780, 30)
        self.update_status()

        self.initCharts()

    def initCharts(self):
        self.chart = QChart()
        self.series = QLineSeries()
        self.chart.addSeries(self.series)
        self.chart.createDefaultAxes()

        self.chart.axisX(self.series).setRange(-180, 180)
        self.chart.axisY(self.series).setRange(-180, 180)

        self.chart_view = QChartView(self.chart)
        self.chart_view.setRenderHint(QtGui.QPainter.Antialiasing)

        dock_widget = QtWidgets.QDockWidget("Chart", self)
        dock_widget.setWidget(self.chart_view)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, dock_widget)

    def toggle_mode(self):
        self.canvas.toggle_controls()

    def update_z_height(self, value):
        self.canvas.z_height = value
        self.canvas.write_data()  # Ensure data is written after updating Z height
        self.update_status()
        self.canvas.update()

    def update_status(self):
        arduino_status = 'Ready' if self.canvas.arduino_ready else 'Not Ready'
        status_text = (f"state: {self.canvas.current_state} | Current Waypoint: {self.canvas.current_waypoint} | Z-Height: {self.canvas.z_height} | "
                    f"Angles: {self.canvas.angles[0]:.2f}, {self.canvas.angles[1]:.2f}, {self.canvas.angles[2]:.2f} | "
                    f"{'Arduino Connected' if arduino_connected else 'Arduino Not Connected'} | Arduino State: {arduino_status}")
        self.status_label.setText(status_text)

    def update_chart(self, angles):
        self.series.append(angles[0], angles[1])

    def show_controls(self):
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Controls")
        msg.setText(
            "Controls:\n"
            "B: Begin path\n"
            "S: Save path\n"
            "SPACE: Add waypoint\n"
            "P: Load waypoints (state 2)\n"
            "L: Load waypoints (state 3)\n"
            "ESC: Return to manual control\n"
            "LSHIFT: Toggle gripper\n"
            "Arrow Keys: Adjust angle\n"
            "WASD: Move pointer\n"
            "Mouse: Drag pointer\n"
            "Mouse Wheel: Adjust Z-height\n"
        )
        msg.exec_()

    def export_waypoints(self, path, filename):
        try:
            with open(filename, "w") as f:
                for point in path:
                    point.write(f)
        except IOError as e:
            print(f"Error saving waypoints: {e}")

    def load_waypoints(self, filename):
        try:
            with open(filename, "r") as f:
                return parse_file(f.readlines())
        except IOError as e:
            print(f"Error loading waypoints: {e}")
            return []

    def save_path(self):
        filename = self.filename_input.text()
        if filename:
            self.export_waypoints(self.canvas.stored_path, f"{filename}.txt")

    def load_path(self, state):
        filename = self.filename_input.text()
        if filename:
            self.canvas.load_path(state)
    

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

