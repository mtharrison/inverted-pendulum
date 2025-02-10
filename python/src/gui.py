import math
from time import perf_counter
import threading
import random

import dearpygui.dearpygui as dpg

from serial_client import SerialCommunicator
from serial_mock import MockSerialEndpoint
from serial_virtual import VirtualSerialPair
from screeninfo import get_monitors

MARGIN_AROUND_VIEWPORT = 200
FRAMES_PER_SECOND = 60

class PendulumVisualizerDPG:
    def __init__(self, monitor, port="/dev/cu.usbmodem2101"):
        # Initialize serial communicator and lock
        self.serial_lock = threading.Lock()
        self.serial = SerialCommunicator(port)

        initial_state = self.serial.sense()
        self.state = initial_state

        # Sizes
        self.viewport_width = monitor.width - MARGIN_AROUND_VIEWPORT
        self.viewport_height = monitor.height - MARGIN_AROUND_VIEWPORT
        self.update_interval = 1/FRAMES_PER_SECOND
        self.pendulum_drawing_height = self.viewport_height //2.5
        self.pendulum_window_width = self.viewport_width // 2
        self.pendulum_chart_height = ((self.viewport_height - self.pendulum_drawing_height) // 2) - 27

        # Pendulum and state variables
        self.last_update = perf_counter()

        # History for charts
        self.angle_history = []
        self.angular_velocity_history = []
        self.position_history = []
        self.velocity_history = []

        dpg.create_context()
        dpg.create_viewport(
            title="Pendulum Visualizer", width=self.viewport_width, height=self.viewport_height
        )

        self.init_pendulum_window()
        self.init_training_window()

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_viewport_pos([0, 0])

    def init_pendulum_window(self):
        with dpg.window(
            label="pendulum_window",
            pos=(0, 0),
            height=self.viewport_height,
            width=self.viewport_width // 2,
        ):
            with dpg.table(
                tag="pendulum_table",
                header_row=False,
                borders_innerH=False,
                borders_outerH=False,
                borders_innerV=False,
                borders_outerV=False,
            ):
                dpg.add_table_column()

                with dpg.table_row():
                    self.pendulum_drawlist = dpg.add_drawlist(
                        width=self.pendulum_window_width,
                        height=self.pendulum_drawing_height,
                    )

                with dpg.table_row():
                    with dpg.table(
                        tag="pendulum_plots_first",
                        header_row=False,
                        borders_innerH=False,
                        borders_outerH=False,
                        borders_innerV=False,
                        borders_outerV=False,
                    ):
                        dpg.add_table_column()
                        dpg.add_table_column()

                        with dpg.table_row():
                            with dpg.child_window(
                                label="angle_chart", width=-1, height=self.pendulum_chart_height
                            ):
                                dpg.add_text(
                                    default_value="Angle: 0.00 rad", tag="angle_text"
                                )
                                with dpg.plot(
                                    label="Angle",
                                    height=-1,
                                    width=-1,
                                ):
                                    
                                    dpg.add_plot_legend()
                                    self.angle_x_axis = dpg.add_plot_axis(
                                        dpg.mvXAxis, label="Time",  tag="angle_x_axis", auto_fit=True
                                    )
                                    self.angle_y_axis = dpg.add_plot_axis(
                                        dpg.mvYAxis, label="Sin(angle)",  tag="angle_y_axis", auto_fit=True
                                    )
                                    self.angle_series = dpg.add_line_series(
                                        [], [], label="Angle", parent='angle_y_axis', tag="angle_series"
                                    )

                            with dpg.child_window(
                                label="angular_velocity_chart",
                                width=-1,
                                height=self.pendulum_chart_height,
                            ):
                                dpg.add_text(
                                    default_value="Angular Velocity: 0.00 rad/s",
                                    tag="angular_velocity_text",
                                )
                                with dpg.plot(
                                    label="Angular Velocity",
                                    height=-1,
                                    width=-1,
                                ):
                                    dpg.add_plot_legend()
                                    dpg.add_plot_axis(
                                        dpg.mvXAxis, label="Time", auto_fit=True
                                    )
                                    self.angular_velocity_y_axis = dpg.add_plot_axis(
                                        dpg.mvYAxis, label="Angular Velocity", auto_fit=True
                                    )
                                    self.angular_velocity_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="Angular Velocity",
                                        parent=self.angular_velocity_y_axis,
                                        tag="angular_velocity_series"
                                    )

                with dpg.table_row():
                    with dpg.table(
                        tag="pendulum_plots_second",
                        header_row=False,
                        borders_innerH=False,
                        borders_outerH=False,
                        borders_innerV=False,
                        borders_outerV=False,
                    ):
                        dpg.add_table_column()
                        dpg.add_table_column()

                        with dpg.table_row():
                            with dpg.child_window(
                                label="position_chart",
                                width=-1,
                                height=self.pendulum_chart_height,
                            ):
                                dpg.add_text(
                                    default_value="Position: 0", tag="position_text"
                                )
                                with dpg.plot(
                                    label="Position",
                                    height=-1,
                                    width=-1,
                                ):
                                    dpg.add_plot_legend()
                                    self.position_x_axis = dpg.add_plot_axis(
                                        dpg.mvXAxis, label="Time", auto_fit=True
                                    )
                                    self.position_y_axis = dpg.add_plot_axis(
                                        dpg.mvYAxis, label="Position", auto_fit=True
                                    )
                                    self.position_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="Position",
                                        parent=self.position_y_axis,
                                    )

                            with dpg.child_window(
                                label="velocity_chart",
                                width=-1,
                                height=self.pendulum_chart_height,
                            ):
                                dpg.add_text(
                                    default_value="Velocity: 0", tag="velocity_text"
                                )
                                with dpg.plot(
                                    label="Velocity",
                                    height=-1,
                                    width=-1,
                                ):
                                    dpg.add_plot_legend()
                                    self.velocity_x_axis = dpg.add_plot_axis(
                                        dpg.mvXAxis, label="Time", auto_fit=True
                                    )
                                    self.velocity_y_axis = dpg.add_plot_axis(
                                        dpg.mvYAxis, label="Velocity", auto_fit=True
                                    )
                                    self.velocity_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="Velocity",
                                        parent=self.velocity_y_axis,
                                    )

    def init_training_window(self):
        with dpg.window(
            label="training_window",
            pos=(self.viewport_width // 2, 0),
            height=self.viewport_height,
            width=self.viewport_width // 2,
        ):
            with dpg.table(
                tag="training_table",
                header_row=False,
                borders_innerH=False,
                borders_outerH=False,
                borders_innerV=False,
                borders_outerV=False,
            ):
                dpg.add_table_column()

                with dpg.table_row():
                    with dpg.group(
                        tag="training_button_group", horizontal=True, horizontal_spacing=100
                    ):
                        with dpg.group(tag="button group sub", horizontal=True):
                            dpg.add_button(tag="Btn1", label="START TRAINING")
                            dpg.add_button(tag="Btn2", label="RESET")

                            with dpg.item_handler_registry(
                                tag="reset handler"
                            ) as handler:
                                dpg.add_item_clicked_handler(callback=self.reset)

                            dpg.bind_item_handler_registry("Btn2", "reset handler")

                        # dpg.add_slider_double(
                        #     width=400,
                        #     label="manual cart position",
                        #     max_value=1000,
                        #     min_value=-1000,
                        #     format="position = %.14f",
                        #     callback=lambda sender, app_data: self.serial.step(
                        #         app_data, self.handle_step_response
                        #     ),
                        # )

                with dpg.table_row():
                    with dpg.group(tag="episode stats", horizontal=True):
                        dpg.add_text("Episode: 0/100", tag="episode_text")
                        dpg.add_text("Reward: 0", tag="reward_text")
                        dpg.add_text("Epsilon: 0", tag="epsilon_text")
                        dpg.add_text("Loss: 0", tag="loss_text")
                        dpg.add_text("Steps: 0", tag="steps_text")
                        dpg.add_text("Time: 0", tag="time_text")
                        dpg.add_text("Total time: 0", tag="total_time_text")

                with dpg.table_row():
                    # Add reward vs episode plot
                    with dpg.child_window(
                        label="training_chart", width=-1, height=-1
                    ):
                        with dpg.plot(
                            label="Reward vs Episode",
                            height=-1,
                            width=-1,
                        ):
                            pass

    def continuous_angle(self, angle):
        """Return an angle in the range [0, 2π)."""
        return angle % (2 * math.pi)

    def get_observation(self):
        with self.serial_lock:
            data = self.serial.sense()

            if data.get("status") == "OK":
                self.state = data
                
                self.angle_history.append(self.continuous_angle(self.state['theta']))
                self.angular_velocity_history.append(self.state['angular_velocity'])
                self.position_history.append(self.state['position'])
                self.velocity_history.append(self.state['velocity'])

    def reset(self, sender, app_data):
        with self.serial_lock:
            self.serial.reset()

    def update(self):
        """Update the pendulum and charts with the latest data."""
        now = perf_counter()
        if now - self.last_update >= self.update_interval:
            self.get_observation()
            self.draw_pendulum()
            self.update_charts()
            self.last_update = now

    def run(self):
        """Main update loop"""
        while dpg.is_dearpygui_running():
            self.update()
            dpg.render_dearpygui_frame()

        # Cleanup
        self.running = False
        self.serial.close()
        dpg.destroy_context()

    def draw_pendulum(self):
        """
        Clear and redraw the pendulum on the drawlist.
        Also draws the limit switch indicators.
        """

        # Remove any previous drawings
        dpg.delete_item(self.pendulum_drawlist, children_only=True)

        # Define the pivot and rod length (same as the pygame version)
        pivot = (self.pendulum_window_width // 2, self.pendulum_drawing_height // 2 - 10)
        rod_length = 150
        bob_x = pivot[0] + rod_length * math.sin(-self.state['theta'])
        bob_y = pivot[1] + rod_length * math.cos(self.state['theta'])

        # Draw the rod (a line from the pivot to the bob)
        dpg.draw_line(
            p1=pivot,
            p2=(bob_x, bob_y),
            color=[255, 255, 255, 255],
            thickness=5,
            parent=self.pendulum_drawlist,
        )

        # Draw the bob (a circle)
        dpg.draw_circle(
            center=(bob_x, bob_y),
            radius=30,
            color=[255, 255, 255, 255],
            fill=[255, 255, 255, 255],
            parent=self.pendulum_drawlist,
        )

        cart_position_origin = (
            self.pendulum_window_width // 2,
            self.pendulum_drawing_height // 2,
        )

        dpg.draw_rectangle(
            pmin=(cart_position_origin[0] - 30, cart_position_origin[1] - 40),
            pmax=(cart_position_origin[0] + 30, cart_position_origin[1]),
            color=[255, 255, 255, 255],
            fill=[255, 255, 255, 255],
            parent=self.pendulum_drawlist,
        )

        dpg.draw_line(
            p1=(100, self.pendulum_drawing_height // 2),
            p2=(self.pendulum_window_width - 100, self.pendulum_drawing_height // 2),
            color=[255, 255, 255, 255],
            thickness=1,
            parent=self.pendulum_drawlist,
        )

        # Draw limit switch indicators at fixed positions
        self.draw_limit(
            (100, self.pendulum_drawing_height // 2),
            self.state['limitL'],
            "LIMIT",
            parent=self.pendulum_drawlist,
        )
        self.draw_limit(
            (self.pendulum_window_width - 100, self.pendulum_drawing_height // 2),
            self.state['limitR'],
            "LIMIT",
            parent=self.pendulum_drawlist,
        )

    def draw_limit(self, position, triggered, label, parent):
        """Draw a limit switch indicator (a circle with text)."""
        color = [255, 0, 0, 255] if triggered else [255, 255, 255, 255]
        dpg.draw_circle(
            center=position, radius=20, color=color, fill=color, parent=parent
        )
        state = "TRIGGERED" if triggered else ""
        text = f"{label} {state}"
        # Offset the text so that it appears below the circle.
        text_position = (position[0] - 20, position[1] - 50)
        dpg.draw_text(
            pos=text_position,
            text=text,
            color=[255, 255, 255, 255],
            size=15,
            parent=parent,
        )

    def update_charts(self):

        number_of_points = 1000

        # Update angle chart data
        x_data = list(range(len(self.angle_history)))[-number_of_points:]
        y_data = [math.sin(a) for a in self.angle_history][-number_of_points:]
        dpg.set_value(self.angle_series, [x_data, y_data])
        dpg.set_value(
            "angle_text", f"Angle: {self.continuous_angle(self.state['theta']):.2f} rad"
        )

        # Update angular velocity chart data
        x_data_vel = list(range(len(self.angular_velocity_history)))[-number_of_points:]
        y_data_vel = self.angular_velocity_history[-number_of_points:]
        dpg.set_value(self.angular_velocity_series, [x_data_vel, y_data_vel])
        dpg.set_value(
            "angular_velocity_text", f"Angular Velocity: {self.state['angular_velocity']:.2f} rad/s"
        )

        # Update position chart data
        x_data_pos = list(range(len(self.position_history)))[-number_of_points:]
        y_data_pos = self.position_history[-number_of_points:]
        dpg.set_value(self.position_series, [x_data_pos, y_data_pos])
        dpg.set_value(
            "position_text", f"Position: {self.state['position']:.2f}"
        )

        # Update velocity chart data
        x_data_vel = list(range(len(self.velocity_history)))[-number_of_points:]
        y_data_vel = self.velocity_history[-number_of_points:]
        dpg.set_value(self.velocity_series, [x_data_vel, y_data_vel])
        dpg.set_value(
            "velocity_text", f"Velocity: {self.state['velocity']:.2f}"
        )


def launch():
    for m in get_monitors():
        monitor = m

    position = 0

    def on_sense(state, request):
        global position
        state["theta"] += 0.01
        state["angular_velocity"] = random.uniform(-1, 1)
        state["position"] = math.cos(state['theta']) * 1000
        state["velocity"] = random.uniform(-1, 1)
        state["limitL"] = True if random.uniform(-1, 1) < 0 else False
        state["limitR"] = True if random.uniform(-1, 1) < 0 else False
        state["enabled"] = True if random.uniform(-1, 1) < 0 else False
        state["resetting"] = False
        state["extents"] = 1000

        return {"status": "OK", **state, "id": request["id"]}

    def on_move(state, request):
        state["target"] += request["params"]["distance"]
        return {"status": "OK", "id": request["id"]}

    def on_reset(state, request):
        state["resetting"] = True
        return {"status": "OK", "id": request["id"], "resetting": True}

    with VirtualSerialPair() as (port1, port2):
        MockSerialEndpoint(
            port=port1, on_sense=on_sense, on_move=on_move, on_reset=on_reset
        )

        visualizer = PendulumVisualizerDPG(monitor, port2)
        visualizer.run()


if __name__ == "__main__":
    launch()
