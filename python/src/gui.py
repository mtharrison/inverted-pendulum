import math
from time import perf_counter
import random
import os
import dearpygui.dearpygui as dpg

from serial_mock import MockSerialEndpoint
from serial_virtual import VirtualSerialPair
from screeninfo import get_monitors

MARGIN_AROUND_VIEWPORT = 200
FRAMES_PER_SECOND = 60


class PendulumVisualizerDPG:
    def __init__(self, data_queue):
        for m in get_monitors():
            monitor = m
            
        self.data_queue = data_queue
            
        # Initialize serial communicator and lock
        self.state = {
            'theta': 0,
            'angular_velocity': 0,
            'current_position': 0,
            'target_position': 0,
            'velocity': 0,
            'limitL': False,
            'limitR': False,
            'enabled': False,
            'resetting': False,
            'extent': 1000
        }

        # Sizes
        self.viewport_width = monitor.width - MARGIN_AROUND_VIEWPORT
        self.viewport_height = monitor.height - MARGIN_AROUND_VIEWPORT
        self.update_interval = 1 / FRAMES_PER_SECOND
        self.pendulum_drawing_height = self.viewport_height // 2.5
        self.pendulum_window_width = self.viewport_width // 2
        self.pendulum_chart_height = (
            (self.viewport_height - self.pendulum_drawing_height) // 2
        ) - 27

        # Pendulum and state variables
        self.last_update = perf_counter()

        # History for charts
        self.angle_sin_history = []
        self.angle_cos_history = []
        self.angular_velocity_history = []
        self.current_position_history = []
        self.target_position_history = []
        self.velocity_history = []
        self.reward_history = []
        self.loss_history = []
        
        dpg.create_context()
        dpg.create_viewport(
            title="Pendulum Visualizer",
            width=self.viewport_width,
            height=self.viewport_height,
        )

        self.init_pendulum_window()
        self.init_training_window()

        dpg.setup_dearpygui()
        dpg.show_style_editor()
        dpg.show_viewport()
        dpg.set_viewport_pos([0, 0])

    def init_pendulum_window(self):
        with dpg.window(
            label="pendulum_window",
            pos=(0, 0),
            height=self.viewport_height,
            width=self.viewport_width // 2,
        ):
            dpg.add_slider_double(
                width=400,
                pos=(10, 30),
                tag="cart_position_slider",
                max_value=self.state["extent"],
                min_value=-self.state["extent"],
                format="position = %.14f",
                callback=lambda sender, app_data: self.move(
                    app_data - self.state["current_position"]
                ),
            )
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
                                label="angle_chart",
                                width=-1,
                                height=self.pendulum_chart_height,
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
                                        dpg.mvXAxis,
                                        label="Time",
                                        tag="angle_x_axis",
                                        auto_fit=True,
                                    )
                                    self.angle_y_axis = dpg.add_plot_axis(
                                        dpg.mvYAxis,
                                        label="Sin/Cos",
                                        tag="angle_y_axis",
                                        auto_fit=True,
                                    )
                                    dpg.set_axis_limits("angle_y_axis", -1, 1)
                                    self.angle_sin_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="sin(angle)",
                                        parent="angle_y_axis",
                                        tag="angle_sin_series",
                                    )
                                    
                                    self.angle_cos_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="cos(angle)",
                                        parent="angle_y_axis",
                                        tag="angle_cos_series",
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
                                        dpg.mvYAxis,
                                        label="Angular Velocity",
                                        auto_fit=True,
                                        tag="angular_velocity_y_axis",
                                    )
                                    dpg.set_axis_limits("angular_velocity_y_axis", -20, 20)
                                    self.angular_velocity_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="Angular Velocity",
                                        parent=self.angular_velocity_y_axis,
                                        tag="angular_velocity_series",
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
                                    default_value="Position: 0",
                                    tag="current_position_text",
                                )
                                with dpg.plot(
                                    label="Position",
                                    height=-1,
                                    width=-1,
                                ):
                                    dpg.add_plot_legend()
                                    self.current_position_x_axis = dpg.add_plot_axis(
                                        dpg.mvXAxis, label="Time", auto_fit=True
                                    )
                                    self.current_position_y_axis = dpg.add_plot_axis(
                                        dpg.mvYAxis, label="Position", auto_fit=True, tag="current_position_y_axis"
                                    )
                                    self.current_position_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="Position",
                                        parent=self.current_position_y_axis,
                                    )

                                    self.target_position_series = dpg.add_line_series(
                                        [],
                                        [],
                                        label="Target Position",
                                        parent=self.current_position_y_axis,
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
                                        dpg.mvYAxis, label="Velocity", tag="velocity_y_axis", auto_fit=True
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
                        tag="training_button_group",
                        horizontal=True,
                        horizontal_spacing=100,
                    ):
                        with dpg.group(tag="button group sub", horizontal=True):
                            dpg.add_button(tag="Btn1", label="START TRAINING")
                            dpg.add_button(tag="Btn2", label="RESET")

                            with dpg.item_handler_registry(
                                tag="reset handler"
                            ) as handler:
                                dpg.add_item_clicked_handler(callback=self.reset)

                            dpg.bind_item_handler_registry("Btn2", "reset handler")

                with dpg.table_row():
                    with dpg.group(tag="episode stats", horizontal=True):
                        dpg.add_text("Episode: 0/100", tag="episode_text")
                        dpg.add_text("Epsilon: 0", tag="epsilon_text")
                        dpg.add_text("Steps: 0", tag="steps_text")
                        dpg.add_text("Episode time: 0", tag="episode time_text")
                        dpg.add_text("Total time: 0", tag="total time_text")

                with dpg.table_row():
                    # Add reward vs episode plot
                    with dpg.child_window(label="reward_chart", width=-1, height=self.viewport_height // 2 - 45):
                        with dpg.plot(
                            label="Reward vs Episode",
                            height=-1,
                            width=-1,
                        ):
                            dpg.add_plot_legend()
                            self.reward_x_axis = dpg.add_plot_axis(
                                dpg.mvXAxis,
                                label="Episode",
                                tag="reward_x_axis",
                                auto_fit=True,
                            )
                            self.reward_y_axis = dpg.add_plot_axis(
                                dpg.mvYAxis,
                                label="Reward",
                                tag="reward_y_axis",
                                auto_fit=True,
                            )
                            self.reward_series = dpg.add_line_series(
                                [],
                                [],
                                label="Reward",
                                parent="reward_y_axis",
                                tag="reward_series",
                            )
                            
                            
                        
                with dpg.table_row():
                    # Add reward vs episode plot
                    with dpg.child_window(label="loss_chart", width=-1, height=self.viewport_height // 2 - 45):
                        with dpg.plot(
                            label="Loss vs Episode",
                            height=-1,
                            width=-1,
                        ):
                            dpg.add_plot_legend()
                            self.loss_x_axis = dpg.add_plot_axis(
                                dpg.mvXAxis,
                                label="Episode",
                                tag="loss_x_axis",
                                auto_fit=True,
                            )
                            self.loss_y_axis = dpg.add_plot_axis(
                                dpg.mvYAxis,
                                label="Loss",
                                tag="loss_y_axis",
                                auto_fit=True,
                            )
                            self.loss_series = dpg.add_line_series(
                                [],
                                [],
                                label="Loss",
                                parent="loss_y_axis",
                                tag="loss_series",
                            )

    def continuous_angle(self, angle):
        """Return an angle in the range [0, 2π)."""
        return angle % (2 * math.pi)

    def reset(self, sender, app_data):
        with self.serial_lock:
            self.serial.reset()

    def move(self, distance):
        with self.serial_lock:
            self.serial.move(distance)

    def pump_queue(self):
        qsize = self.data_queue.qsize()
        for _ in range(qsize):
            item = self.data_queue.get()
            if item["type"] == "observation":
                self.state = item["data"]
                self.angle_sin_history.append(math.sin(self.state["theta"]))
                self.angle_cos_history.append(math.cos(self.state["theta"]))
                self.angular_velocity_history.append(self.state["angular_velocity"])
                self.current_position_history.append(self.state["current_position"])
                self.target_position_history.append(self.state["target_position"])
                self.velocity_history.append(self.state["velocity"])
            if item["type"] == "training":
                for key, value in item['data'].items():
                    dpg.set_value(f'{key}_text', f'{key.title()}: {value}')
            if item["type"] == "episode":
                self.reward_history.append(item["data"]["reward"])
                self.loss_history.append(item["data"]["loss"])
        

    def update(self):
        """Update the pendulum and charts with the latest data."""
        now = perf_counter()
        if now - self.last_update >= self.update_interval:
            self.pump_queue()
            # self.get_observation()
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
        dpg.destroy_context()

    def draw_pendulum(self):
        """
        Clear and redraw the pendulum on the drawlist.
        Also draws the limit switch indicators.
        """

        min_x_pos = 100
        max_x_pos = self.pendulum_window_width - 100
        extent = self.state.get("extent", 1000)
        current_position = self.state["current_position"]
        if extent != 0:
            pendulum_offset_from_center = (current_position / extent) * (
                (max_x_pos - min_x_pos - 90) / 2
            )
        else:
            pendulum_offset_from_center = 0

        # Remove any previous drawings
        dpg.delete_item(self.pendulum_drawlist, children_only=True)

        # Define the pivot and rod length (same as the pygame version)
        pivot = (
            self.pendulum_window_width // 2 + pendulum_offset_from_center,
            self.pendulum_drawing_height // 2,
        )
        rod_length = 180
        bob_x = pivot[0] + rod_length * math.sin(-self.state["theta"])
        bob_y = pivot[1] + rod_length * math.cos(self.state["theta"])
        
        theta = self.state["theta"] % (2 * math.pi)
        
        dpg.draw_triangle(
            p1=pivot,
            p2=(pivot[0] - 20, pivot[1] - 150),
            p3=(pivot[0] + 20, pivot[1] - 150),
            color=[0, 255, 0, 30] if ((theta > math.pi - 0.15) and theta < math.pi + 0.15) else [255, 0, 0, 30],
            fill=[0, 255, 0, 30] if ((theta > math.pi - 0.15) and theta < math.pi + 0.15) else [255, 0, 0, 30],
            parent=self.pendulum_drawlist,
        )

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
            self.pendulum_window_width // 2 + pendulum_offset_from_center,
            self.pendulum_drawing_height // 2,
        )

        dpg.draw_circle(
            center=cart_position_origin,
            radius=10,
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

        dpg.draw_text(
            pos=(
                self.pendulum_window_width // 2 - 80,
                self.pendulum_drawing_height // 2 - 40,
            ),
            text="Resetting Pendulum" if self.state["resetting"] else "",
            color=[255, 255, 255, 255],
            size=15,
            parent=self.pendulum_drawlist,
        )

        # Draw limit switch indicators at fixed positions
        self.draw_limit(
            (100, self.pendulum_drawing_height // 2),
            self.state["limitL"],
            "LIMIT",
            parent=self.pendulum_drawlist,
        )
        self.draw_limit(
            (self.pendulum_window_width - 100, self.pendulum_drawing_height // 2),
            self.state["limitR"],
            "LIMIT",
            parent=self.pendulum_drawlist,
        )

    def draw_limit(self, position, triggered, label, parent):
        """Draw a limit switch indicator (a circle with text)."""
        color = [255, 0, 0, 255] if triggered else [255, 255, 255, 255]
        dpg.draw_circle(
            center=position, radius=20, color=color, fill=color, parent=parent
        )
        text = f"{label}"
        # Offset the text so that it appears below the circle.
        text_position = (position[0] - 20, position[1] + 40)
        dpg.draw_text(
            pos=text_position,
            text=text,
            color=[255, 255, 255, 255],
            size=15,
            parent=parent,
        )

    def update_charts(self):
        number_of_points = 1000
        
        dpg.set_axis_limits("current_position_y_axis", -self.state["extent"], self.state["extent"])
        dpg.set_axis_limits("velocity_y_axis", -self.state["extent"], self.state["extent"])

        # Update angle chart data
        x_data_sin = list(range(len(self.angle_sin_history)))[-number_of_points:]
        y_data_sin = self.angle_sin_history[-number_of_points:]
        dpg.set_value(self.angle_sin_series, [x_data_sin, y_data_sin])
        
        x_data_cos = list(range(len(self.angle_cos_history)))[-number_of_points:]
        y_data_cos = self.angle_cos_history[-number_of_points:]
        dpg.set_value(self.angle_cos_series, [x_data_cos, y_data_cos])
        
        dpg.set_value(
            "angle_text", f"Angle: {self.continuous_angle(self.state['theta']):.2f} rad"
        )

        # Update angular velocity chart data
        x_data_vel = list(range(len(self.angular_velocity_history)))[-number_of_points:]
        y_data_vel = self.angular_velocity_history[-number_of_points:]
        dpg.set_value(self.angular_velocity_series, [x_data_vel, y_data_vel])
        dpg.set_value(
            "angular_velocity_text",
            f"Angular Velocity: {self.state['angular_velocity']:.2f} rad/s",
        )

        # Update position chart data
        x_data_pos = list(range(len(self.current_position_history)))[-number_of_points:]
        y_data_pos = self.current_position_history[-number_of_points:]
        dpg.set_value(self.current_position_series, [x_data_pos, y_data_pos])
        dpg.set_value(
            "current_position_text", f"Position: {self.state['current_position']:.2f}"
        )

        x_data_target = list(range(len(self.target_position_history)))[
            -number_of_points:
        ]
        y_data_target = self.target_position_history[-number_of_points:]
        dpg.set_value(self.target_position_series, [x_data_target, y_data_target])

        # Update velocity chart data
        x_data_vel = list(range(len(self.velocity_history)))[-number_of_points:]
        y_data_vel = self.velocity_history[-number_of_points:]
        dpg.set_value(self.velocity_series, [x_data_vel, y_data_vel])
        dpg.set_value("velocity_text", f"Velocity: {self.state['velocity']:.2f}")

        dpg.set_value("cart_position_slider", self.state["current_position"])
        
        # Update reward vs episode chart data
        x_data_reward = list(range(len(self.reward_history)))
        y_data_reward = self.reward_history
        dpg.set_value(self.reward_series, [x_data_reward, y_data_reward])
        
        # Update loss vs episode chart data
        x_data_loss = list(range(len(self.loss_history)))
        y_data_loss = self.loss_history
        dpg.set_value(self.loss_series, [x_data_loss, y_data_loss])


def launch():
    for m in get_monitors():
        monitor = m
    print(os.getenv("ENVIRONMENT"))
    if os.getenv("ENVIRONMENT") == "dev":

        def on_sense(state, request):
            state["theta"] += 0.01
            state["angular_velocity"] = random.uniform(-1, 1)
            state["current_position"] = math.cos(state["theta"]) * 1000
            state["target_position"] = (math.cos(state["theta"]) * 1000) + 200
            state["velocity"] = random.uniform(-1, 1)
            state["limitL"] = False
            state["limitR"] = False
            state["enabled"] = True if random.uniform(-1, 1) < 0 else False
            state["resetting"] = state["resetting"]
            state["extent"] = 1000

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

    else:
        visualizer = PendulumVisualizerDPG(monitor)
        visualizer.run()


if __name__ == "__main__":
    launch()
