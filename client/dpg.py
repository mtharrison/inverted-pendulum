import math
import json
import threading
import time
from time import perf_counter

import serial
import dearpygui.dearpygui as dpg

from client import SerialCommunicator  # assuming your serial client is the same

class PendulumVisualizerDPG:
    def __init__(self):
        # Window and update parameters
        self.width = 2000
        self.height = 1200
        self.update_interval = 0.016  # ~60 Hz
        self.velocity_scale = 40      # for scaling the velocity chart (if needed)

        # Pendulum and state variables
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.limitL = False
        self.limitR = False
        self.running = True

        # History for charts
        self.angle_history = []      # will store continuous angles
        self.velocity_history = []   # store angular velocity history

        # Chart sizes (we use these to trim the history lists)
        self.angle_chart_width = round(self.width/2) - 30
        self.angle_chart_height = round(self.height/2) - 50
        self.velocity_chart_width = round(self.width/2) - 30
        self.velocity_chart_height = round(self.height/2) - 50

        self.pendulum_width = self.width/2

        # Lock for serial communication
        self.serial_lock = threading.Lock()

        # Initialize your serial communicator (adjust the port as needed)
        self.serial = SerialCommunicator(port='/dev/cu.usbmodem2101')

        # ---------------------
        # Initialize Dear PyGui
        # ---------------------
        dpg.create_context()
        dpg.create_viewport(title="Pendulum Visualizer", width=self.width, height=self.height)

        # Register a key-press handler
        with dpg.handler_registry():
            dpg.add_key_press_handler(callback=self.handle_key_press)

        # Create a window for the pendulum drawing.
        # (We use a drawlist as our canvas.)
        with dpg.window(label="Pendulum Visualization", pos=(0, 0), width=self.width*.5, height=self.height):
            # The drawlist acts as our drawing canvas.
            self.pendulum_drawlist = dpg.add_drawlist(width=self.width, height=self.height)

        # Create a window for the Angle Chart.
        with dpg.window(label="Angle Chart", pos=(self.width*.5, 0), width=self.width * .5, height=self.height/2):
            # A text widget to display the current angle.
            dpg.add_text(default_value="Angle: 0.00 rad", tag="angle_text")
            with dpg.plot(label="Angle Chart", height=self.angle_chart_height, width=self.angle_chart_width):
                dpg.add_plot_legend()
                self.angle_x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time", auto_fit=True)
                self.angle_y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Sine(angle)")
                # Our line series; we start with empty data.
                self.angle_series = dpg.add_line_series([], [], label="Angle", parent=self.angle_y_axis)

        # Create a window for the Angular Velocity Chart.
        with dpg.window(label="Angular Velocity Chart", pos=(self.width/2, self.height/2),
                          width=self.width/2, height=self.height/2):
            dpg.add_text(default_value="Angular Velocity: 0.00 rad/s", tag="velocity_text")
            with dpg.plot(label="Angular Velocity Chart", height=self.velocity_chart_height, width=self.velocity_chart_width):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="Time", auto_fit=True)
                self.velocity_y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Angular Velocity")
                self.velocity_series = dpg.add_line_series([], [], label="Angular Velocity", parent=self.velocity_y_axis)

        dpg.setup_dearpygui()
        dpg.show_viewport()

    def continuous_angle(self, angle):
        """Return an angle in the range [0, 2π)."""
        return angle % (2 * math.pi)

    def fetch_serial_data(self):
        """
        Read from the serial port (using your SerialCommunicator)
        and update history lists.
        """
        try:
            data = self.serial.observe()
            if data.get('status') == 'OK':
                angle = data.get('theta', 0)
                velocity = data.get('angular_velocity', 0)

                # Update history lists (store the continuous angle)
                self.angle_history.append(self.continuous_angle(angle))
                self.velocity_history.append(velocity)

                # Keep only the most recent data (based on chart width)
                self.angle_history = self.angle_history[-self.angle_chart_width:]
                self.velocity_history = self.velocity_history[-self.velocity_chart_width:]

                return {
                    'angle': angle,
                    'velocity': velocity,
                    'limitL': data.get('limitL', False),
                    'limitR': data.get('limitR', False)
                }
        except (json.JSONDecodeError, serial.SerialException) as e:
            print(f"Serial error: {e}")
        return None

    def handle_key_press(self, sender, app_data):
        """
        Called when a key is pressed.
        In Dear PyGui the key codes for left/right arrows are assumed here
        to be 263 (left) and 262 (right).
        """
        step = 0
        if app_data == 263:  # left arrow key
            step = -1
        elif app_data == 262:  # right arrow key
            step = 1

        if step != 0:
            try:
                with self.serial_lock:
                    # Send a JSON message similar to the pygame version.
                    msg = json.dumps([0, 1, step]).encode() + b'\n'
                    self.serial.write(msg)
            except serial.SerialException as e:
                print(f"Write error: {e}")

    def draw_pendulum(self):
        """
        Clear and redraw the pendulum on the drawlist.
        Also draws the limit switch indicators.
        """
        # Remove any previous drawings
        dpg.delete_item(self.pendulum_drawlist, children_only=True)

        # Define the pivot and rod length (same as the pygame version)
        pivot = (self.pendulum_width // 2, 200)
        rod_length = 400
        bob_x = pivot[0] + rod_length * math.sin(-self.angle)
        bob_y = pivot[1] + rod_length * math.cos(self.angle)

        # Draw the rod (a line from the pivot to the bob)
        dpg.draw_line(p1=pivot, p2=(bob_x, bob_y),
                      color=[255, 255, 255, 255],
                      thickness=5,
                      parent=self.pendulum_drawlist)

        # Draw the bob (a circle)
        dpg.draw_circle(center=(bob_x, bob_y),
                        radius=60,
                        color=[255, 255, 255, 255],
                        fill=[255, 255, 255, 255],
                        parent=self.pendulum_drawlist)

        # Draw limit switch indicators at fixed positions
        self.draw_limit((100, 100), self.limitL, "LEFT", parent=self.pendulum_drawlist)
        self.draw_limit((self.pendulum_width - 100, 100), self.limitR, "RIGHT", parent=self.pendulum_drawlist)

    def draw_limit(self, position, triggered, label, parent):
        """Draw a limit switch indicator (a circle with text)."""
        color = [255, 0, 0, 255] if triggered else [255, 255, 255, 255]
        dpg.draw_circle(center=position,
                        radius=20,
                        color=color,
                        fill=color,
                        parent=parent)
        state = "TRIGGERED" if triggered else ""
        text = f"{label} {state}"
        # Offset the text so that it appears below the circle.
        text_position = (position[0] - 30, position[1] + 40)
        dpg.draw_text(pos=text_position,
                      text=text,
                      color=[255, 255, 255, 255],
                      size=20,
                      parent=parent)

    def update_charts(self):
        # Update angle chart data
        x_data = list(range(len(self.angle_history)))
        y_data = [math.sin(a) for a in self.angle_history]  # or use self.angle_history directly if preferred
        dpg.set_value(self.angle_series, [x_data, y_data])
        dpg.set_value("angle_text", f"Angle: {self.continuous_angle(self.angle):.2f} rad")
        
        # Set fixed y-axis limits for angle chart
        dpg.set_axis_limits(self.angle_y_axis, -1.2, 1.2)

        # Update velocity chart data
        x_data_vel = list(range(len(self.velocity_history)))
        y_data_vel = self.velocity_history
        dpg.set_value(self.velocity_series, [x_data_vel, y_data_vel])
        dpg.set_value("velocity_text", f"Angular Velocity: {self.angular_velocity:.2f} rad/s")
        # Set fixed y-axis limits for velocity chart (adjust as necessary)
        dpg.set_axis_limits(self.velocity_y_axis, -40, 40)


    def update(self):
        """Fetch new serial data, update state and redraw everything."""
        data = self.fetch_serial_data()
        if data:
            self.angle = data['angle']
            self.angular_velocity = data['velocity']
            self.limitL = data['limitL']
            self.limitR = data['limitR']

        self.draw_pendulum()
        self.update_charts()

    def run(self):
        """Main update loop (runs until the viewport is closed)."""
        last_update = perf_counter()
        while dpg.is_dearpygui_running():
            now = perf_counter()
            if now - last_update >= self.update_interval:
                self.update()
                last_update = now

            dpg.render_dearpygui_frame()

        # On exit, close the serial connection and clean up DPG.
        self.serial.close()
        dpg.destroy_context()


if __name__ == "__main__":
    visualizer = PendulumVisualizerDPG()
    visualizer.run()
