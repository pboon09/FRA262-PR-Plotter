import tkinter as tk
import platform
import time
from components.color import Color
from components.grid import Grid
from components.target import Target
from components.navigator import Navigator
from components.button import PressButton, RadioButton, ToggleButton, StatusButton
from components.shape import RoundRectangle, Line
from components.text import TextBox, MessageBox, Error
from components.photo import Photo
from components.entry import Entry, OrderEntry
from protocol import Protocol_RT
import math

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        # Title
        self.title('Base System')
        # Mode
        # self.mode = "Graphic"
        self.mode = "Protocol"
        # Define os
        self.os = platform.platform()[0].upper()
        # Window Dimension
        window_width = 900
        window_height = 800
        # Find Window Center
        window_center_x = int(self.winfo_screenwidth()/2 - window_width / 2)
        window_center_y = 0
        # Set Window Properties
        self.geometry(f'{window_width}x{window_height}+{window_center_x}+{window_center_y}')
        self.resizable(False, False)
        self.configure(bg=Color.darkgray)

        self.point_entries = []  
        self.jog_points = []  
        self.point_mode_points = []  

        # create component
        self.create_components()
        # Counting Time
        self.time_ms_rt = 0 
        self.connection = True
        self.new_connection = True
        self.homing = False
        self.running = False   

        # Prepare Protocol
        if self.mode == "Protocol":
            self.protocol_rt = Protocol_RT()
            self.connection = True
            self.new_connection = True
        else:
            self.protocol_rt = None  
            self.connection = False
            self.new_connection = False
        
    def polar_to_cartesian(self, r, theta):
        try:
            theta_rad = math.radians(float(theta))
            x = float(r) * math.cos(theta_rad)
            y = float(r) * math.sin(theta_rad)
            return x, y
        except (ValueError, TypeError):
            return 0, 0
            
    def task(self):
        self.handle_toggle_up_down()
        self.handle_radio_operation()
        self.handle_press_home()
        self.handle_press_run()

        if self.mode == "Protocol":
            self.start_time = time.time()
            self.handle_protocol_rt()

        # Loop every 10 ms
        self.after(10, self.task)
        self.time_ms_rt += 10  


    def create_components(self):
        """
        This function creates each UI components
        """
        # Define font size of each OS
        font_size_title = 12
        font_size_subtitle = 11
        font_size_detail = 9
        font_size_unit_grid = 6
        font_size_button_small = 9
        font_size_button_home = 12
        font_size_button_run = 17
        font_size_message_error = 7

        # Field of table (background)
        self.canvas_field = tk.Canvas(master=self, width=900, height=800, bg=Color.darkgray, bd=0, highlightthickness=0)
        self.canvas_field.pack(side="top")

        # Baackground
        self.background_field_table = RoundRectangle(canvas=self.canvas_field, x=25, y=100, w=510, h=600, r=20, color=Color.whitegray)
        self.background_field_detail = RoundRectangle(canvas=self.canvas_field, x=550, y=70, w=320, h=680, r=20, color=Color.whitegray)
        
        # Grid
        self.grid = Grid(canvas=self.canvas_field, offset_x=40, offset_y=160, row=48, column=48, color_grid=Color.lightgray, color_highlight=Color.gray)

        # Title
        self.text_title = TextBox(canvas=self.canvas_field, x=700, y=40, text="ROBOTICS STUDIO III : BASE SYSTEM", font_name="Inter-Bold", font_size=font_size_title, color=Color.whitegray, anchor="center")

        # Group Detail
        self.text_detail = TextBox(canvas=self.canvas_field, x=720, y=100, text="Detail", font_name="Inter-SemiBold", font_size=font_size_subtitle, color=Color.darkgray, anchor="center")

        self.text_r_pos = TextBox(canvas=self.canvas_field, x=580, y=125, text="r Position", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_theta_pos = TextBox(canvas=self.canvas_field, x=580, y=150, text="theta Position", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_r_pos_unit = TextBox(canvas=self.canvas_field, x=800, y=125, text="mm", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_theta_pos_unit = TextBox(canvas=self.canvas_field, x=800, y=150, text="degree", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_r_pos_num = TextBox(canvas=self.canvas_field, x=720, y=125, text="0.0", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")
        self.text_theta_pos_num = TextBox(canvas=self.canvas_field, x=720, y=150, text="0.0", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")

        self.text_r_spd = TextBox(canvas=self.canvas_field, x=580, y=175, text="r Speed", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_theta_spd = TextBox(canvas=self.canvas_field, x=580, y=200, text="theta Speed", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_r_spd_unit = TextBox(canvas=self.canvas_field, x=800, y=175, text="mm/s", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_theta_spd_unit = TextBox(canvas=self.canvas_field, x=800, y=200, text="degree/s", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_r_spd_num = TextBox(canvas=self.canvas_field, x=720, y=175, text="0.0", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")
        self.text_theta_spd_num = TextBox(canvas=self.canvas_field, x=720, y=200, text="0.0", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")

        self.text_r_acc = TextBox(canvas=self.canvas_field, x=580, y=225, text="r Acceleration", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_theta_acc = TextBox(canvas=self.canvas_field, x=580, y=250, text="theta Acceleration", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_r_acc_unit = TextBox(canvas=self.canvas_field, x=800, y=225, text="mm/s²", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_theta_acc_unit = TextBox(canvas=self.canvas_field, x=800, y=250, text="degree/s²", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_r_acc_num = TextBox(canvas=self.canvas_field, x=720, y=225, text="0.0", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")
        self.text_theta_acc_num = TextBox(canvas=self.canvas_field, x=720, y=250, text="0.0", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")

        # Moving Status
        self.text_rt_status = TextBox(canvas=self.canvas_field, x=580, y=275, text="Moving Status", font_name="Inter-Regular", font_size=font_size_detail, color=Color.darkgray, anchor="w")
        self.text_rt_status_value = TextBox(canvas=self.canvas_field, x=720, y=275, text="Idle", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")
        
        self.line_seperate_1 = Line(canvas=self.canvas_field, point_1=(580, 310), point_2=(850, 310), width=1, color=Color.lightgray)

        # Up/Down Toggle Button
        self.text_operation = TextBox(canvas=self.canvas_field, x=650, y=340, text="Pen Status", font_name="Inter-SemiBold", font_size=font_size_subtitle, color=Color.darkgray, anchor="center")
        self.toggle_up_down = ToggleButton(canvas=self.canvas_field, x=720, y=330, w=40, h=20, on_active_color=Color.blue, off_active_color=Color.gray, on_inactive_color=Color.lightgray,off_inactive_color=Color.darkgray,on_text="UP", off_text="DOWN", font_name="Inter-Regular",text_size=font_size_button_small,on_default=False)

        # Sensors
        self.text_sensor_1 = TextBox(canvas=self.canvas_field, x=600, y=380, text="Limit UP: ", font_name="Inter-SemiBold", font_size=font_size_detail, color=Color.darkgray, anchor="center")
        self.text_sensor_2 = TextBox(canvas=self.canvas_field, x=750, y=380, text="Limit Down: ", font_name="Inter-SemiBold", font_size=font_size_detail, color=Color.darkgray, anchor="center")

        self.status_sensor_1 = TextBox(canvas=self.canvas_field, x=640, y=380, text="CLEAR", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")
        self.status_sensor_2 = TextBox(canvas=self.canvas_field, x=790, y=380, text="CLEAR", font_name="Inter-Medium", font_size=font_size_detail, color=Color.blue, anchor="w")

        
        self.line_seperate_1 = Line(canvas=self.canvas_field, point_1=(580, 420), point_2=(850, 420), width=1, color=Color.lightgray)

        # Operation Mode
        self.operation_mode = "Jog"
        self.text_operation = TextBox(canvas=self.canvas_field, x=720, y=450, text="Operation Mode", font_name="Inter-SemiBold", font_size=font_size_subtitle, color=Color.darkgray, anchor="center")

        self.radio_jog = RadioButton(canvas=self.canvas_field, x=600, y=490, r=14, active_color=Color.blue, inactive_color=Color.lightgray, text="Jog Mode", font_name="Inter-Regular", text_size=font_size_button_small, on_default=True)
        self.radio_point = RadioButton(canvas=self.canvas_field, x=730, y=490, r=14, active_color=Color.blue, inactive_color=Color.lightgray, text="Point Mode", font_name="Inter-Regular", text_size=font_size_button_small, on_default=False)

        self.label_point = TextBox(canvas=self.canvas_field,x=570, y=550,text="Target Point", font_name="Inter-SemiBold",font_size=9, color=Color.darkgray, anchor="w")
        self.label_mm_r = TextBox(canvas=self.canvas_field,x=705, y=550,text="mm", font_name="Inter-SemiBold",font_size=9, color=Color.darkgray, anchor="w")
        self.label_dg_theta = TextBox(canvas=self.canvas_field,x=810, y=550,text="degree", font_name="Inter-SemiBold",font_size=9, color=Color.darkgray, anchor="w")

        self.entry_r = Entry(master=self, canvas=self.canvas_field, x=650, y=540, w=50, h=20, color=Color.blue)
        self.entry_theta = Entry(master=self, canvas=self.canvas_field, x=755, y=540, w=50, h=20, color=Color.blue)
        self.entry_r.hide()
        self.entry_theta.hide()
        self.point_entries.append((self.entry_r, self.entry_theta))
        self.entry_r.bind("<FocusOut>", self.out_entry)
        self.entry_r.bind("<Return>", self.out_entry)
        self.entry_theta.bind("<FocusOut>", self.out_entry)
        self.entry_theta.bind("<Return>", self.out_entry)

        # Home & Run Buttons
        self.press_home = PressButton(canvas=self.canvas_field, x=640, y=600, w=150, h=35, r=15, active_color=Color.gray, inactive_color=Color.lightgray, text="Home", font_name="Inter-SemiBold", text_size=font_size_button_home, active_default=True)
        self.press_run = PressButton(canvas=self.canvas_field, x=640, y=650, w=150, h=35, r=15, active_color=Color.blue, inactive_color=Color.lightgray, text="Run", font_name="Inter-SemiBold", text_size=font_size_button_run, active_default=False)

        # Error Message
        self.message_error = MessageBox(canvas=self.canvas_field, x=640, y=580, width_field=320, text="", color=Color.red, align="Center", font_name="Inter-Bold", size=font_size_message_error)
        self.message_error.hide()

        # Connection Message
        self.message_connection = MessageBox(canvas=self.canvas_field, x=550, y=700, width_field=320, text="\" Connection Disconnected \"", color=Color.red, align="Center", font_name="Inter-Bold", size=font_size_message_error)
        self.message_connection.hide()

        self.dot = None
        self.dot_point = None

        self.target_r = 0
        self.previous_r = 0
        self.previous_theta = 0

        self.grid_flag = False

        self.after_idle_flag = 0
        self.finish_run_flag = 2
        self.press_run_flag = False
        
        self.bind("<Return>", self.handle_radio_operation)
        self.canvas_field.bind("<Button-1>", lambda event: self.handle_press_home(event), add='+')
        self.canvas_field.bind("<Button-1>", lambda event: self.handle_press_run(event), add='+')


    def handle_jog_mode_movement(self, r, theta):
        x, y = self.polar_to_cartesian(r, theta)  
        if len(self.jog_points) >= 10:
            self.jog_points.pop(0)  
        
        self.jog_points.append((x, y))
        self.plot_graph()
        print(f"[Jog Mode] Saved point {len(self.jog_points)}/10: r = {r}, theta = {theta}, x = {x:.2f}, y = {y:.2f}")


    def handle_radio_operation(self, event=None):
        if self.radio_point.on and self.operation_mode != "Point":
            self.operation_mode = "Point"
            print("Switched to Point Mode")
            self.radio_jog.turn_off()

            self.jog_points.clear()  
            self.entry_r.show()
            self.entry_theta.show()
            self.entry_r.enable()
            self.entry_theta.enable()

            self.press_run.activate()

        elif self.radio_jog.on and self.operation_mode != "Jog":
            self.operation_mode = "Jog"
            print("Switched to Jog Mode")
            self.radio_point.turn_off()

            self.entry_r.hide()
            self.entry_theta.hide()
            self.entry_r.disable()
            self.entry_theta.disable()

            self.point_mode_points.clear() 
            self.press_run.deactivate()

    def handle_toggle_up_down(self):
        if self.toggle_up_down.pressed:
            if not self.toggle_up_down.on:
                print("🔼 Moving UP")
                if self.mode == "Protocol":
                    self.protocol_rt.write_up_down_order(up=1, down=0)
                self.toggle_up_down.toggle_on()
            else:
                print("🔽 Moving DOWN")
                if self.mode == "Protocol":
                    self.protocol_rt.write_up_down_order(up=0, down=1)
                self.toggle_up_down.toggle_off()

            # Read up/down status
            up, down = self.protocol_rt.read_up_down_order()  
            print(f"Up: {up}, Down: {down}")

            self.update_idletasks()
            self.toggle_up_down.pressed = False


    def out_entry(self, event):
        """
        This function is called when user clicks outside the entry or presses enter.
        """
        if self.operation_mode == "Point":
            print(self.validate_entry()) 
            if self.validate_entry() == "Normal":
                for i, (entry_r, entry_theta) in enumerate(self.point_entries):
                    try:
                        self.target_r = float(entry_r.get_value())
                        self.target_theta = float(entry_theta.get_value())

                        print(f"Point {i+1}: r = {self.target_r}, theta = {self.target_theta}")

                        entry_r.set_text(str(self.target_r))
                        entry_theta.set_text(str(self.target_theta))

                    except ValueError:
                        print(f"Invalid input at Point {i+1}")
                        continue 

            print(f"Final Target Position: r = {self.target_r}, theta = {self.target_theta}")

        elif self.operation_mode == "Jog":
            print("Jog Mode - No Entry Update Required")

    def validate_entry(self):
        """
        This function validates input in entry and shows an error message if invalid.
        """
        if self.operation_mode == "Point":
            validate_point_result = "Normal"

            for i, (entry_r, entry_theta) in enumerate(self.point_entries):
                try:
                    r_value = float(entry_r.get_value())
                    theta_value = float(entry_theta.get_value())

                    if r_value < 0 or r_value > 500:
                        entry_r.error()
                        validate_point_result = f"Error: r {r_value} is out of range (0-500 mm)"
                    else:
                        entry_r.normal()

                    if theta_value < -180 or theta_value > 180:
                        entry_theta.error()
                        validate_point_result = f"Error: θ {theta_value} is out of range (-180° to 180°)"
                    else:
                        entry_theta.normal()

                except ValueError:
                    entry_r.error()
                    entry_theta.error()
                    validate_point_result = f"Invalid input at Point {i+1}"


            if validate_point_result != "Normal":
                self.message_error.change_text(validate_point_result)
                self.message_error.show()
                self.press_run.deactivate()
            else:
                self.message_error.hide()
                if not self.running and not self.homing and self.connection:
                    if self.mode == "Protocol":
                        if self.protocol_rt.usb_connect and self.protocol_rt.routine_normal:
                            self.press_run.activate()
                    else:
                        self.press_run.activate()


            return validate_point_result

        else:
            self.message_error.hide()
            if not self.running and not self.homing and self.connection:
                self.press_run.activate()


    def interpret_validate(self, validate_result, operation_mode):
        """
        This function converts number code from validation result to error text
        """
        if self.operation_mode == "Point":
            if validate_result == "r_out_of_range":
                return "Error: r is out of range (0-500 mm)"
            elif validate_result == "theta_out_of_range":
                return "Error: θ is out of range (-180° to 180°)"
            elif validate_result == "invalid_input":
                return "Error: Invalid input detected"
        return "Normal"

                    
    def handle_press_home(self, event=None):
        """
        This function handles when user press "Home" button (กลับไปที่ตำแหน่ง Home)
        """
        if self.press_home.pressed:
            print("Returning to Home Position...")

            if self.mode == "Protocol": 
                self.protocol_rt.write_base_system_status("Home")

            self.previous_mode = self.operation_mode  
            self.homing = True
            self.radio_jog.deactivate()
            self.radio_point.deactivate()
            self.press_run.deactivate()
            self.press_home.deactivate()
            for entry_r, entry_theta in self.point_entries:
                entry_r.disable()

            self.press_home.pressed = False
            self.after(500, self.check_home_status)

    def check_home_status(self):
        """
        This function checks if the robot has completed the Home operation
        """
        if self.protocol_rt is not None and hasattr(self.protocol_rt, "r_theta_moving_status"):
            if self.protocol_rt.r_theta_moving_status == "Idle":  
                print("Home Position Reached!")

                self.homing = False
                self.radio_jog.activate()
                self.radio_point.activate()
                self.press_run.activate()
                self.press_home.activate()

                if self.previous_mode == "Jog":
                    self.radio_jog.turn_on()
                    self.radio_point.turn_off()
                    self.operation_mode = "Jog"
                    print("Switched back to Jog Mode")
                elif self.previous_mode == "Point":
                    self.radio_jog.turn_off()
                    self.radio_point.turn_on()
                    self.operation_mode = "Point"
                    print("Switched back to Point Mode")

                    for entry_r, entry_theta in self.point_entries:
                        entry_r.enable()
                        entry_theta.enable()
            else:
                self.after(500, self.check_home_status)
        else:
            print("Warning: Protocol_RT is not initialized. Skipping check_home_status.")
            self.homing = False
            self.radio_jog.activate()
            self.radio_point.activate()
            self.press_run.activate()
            self.press_home.activate()
        self.press_run.activate()

    
    def handle_press_run(self, event=None):
        if self.press_run.pressed:
            print("Starting movement command...")

            if self.operation_mode == "Point":
                try:
                    r = float(self.entry_r.get_value()) if self.entry_r.get_value() else 0.0
                    theta = float(self.entry_theta.get_value()) if self.entry_theta.get_value() else 0.0

                    self.point_mode_points.clear()  
                    self.point_mode_points.append((r, theta))  

                    print(f"[Point Mode] ส่งหุ่นยนต์ไปที่ r = {r}, theta = {theta}")

                    if self.mode == "Protocol" and self.protocol_rt is not None:
                        self.protocol_rt.write_goal_point(r, theta)

                    self.plot_graph()

                except ValueError:
                    print("Invalid input for r, theta")

            self.running = True
            self.radio_jog.deactivate()
            self.radio_point.deactivate()
            self.press_run.deactivate()
            self.press_home.deactivate()

            self.after(1000, self.check_run_status)
            self.press_run.pressed = False


    def check_run_status(self):
        """
        This function checks if the robot has completed its movement
        """
        if self.protocol_rt is None:
            print("Warning: Protocol is not initialized. Skipping check_run_status.")
            return  

        if self.protocol_rt.r_theta_moving_status == "Idle":  
            print("Movement Completed")

            self.running = False
            self.radio_jog.activate()
            self.radio_point.activate()
            self.press_run.activate()
            self.press_home.activate()  

            for entry_r, entry_theta in self.point_entries:
                entry_r.enable()
                entry_theta.enable()

            self.homing = False
            self.finish_run_flag = 2  
        else:
            self.after(500, self.check_run_status)

    
    def handle_finish_moving(self):
        """
        This function handles when the movement is finished and reactivates elements.
        """
        print("Movement Completed. Reactivating UI elements...")

        self.radio_jog.activate()
        self.radio_point.activate()
        self.press_run.activate()
        self.press_home.activate()  

        for entry_r, entry_theta in self.point_entries:
            entry_r.enable()
            entry_theta.enable()

        self.press_run_flag = False
        self.homing = False 



    def handle_connection_change(self):
        """
        This function handles when the connection changes (disconnected/reconnected).
        """
        current_connection = self.protocol_rt.usb_connect

        if self.connection != current_connection:
            self.connection = current_connection

            if not self.connection:  
                print("Connection Lost! Disabling UI elements...")
                self.handle_disconnected()
            else: 
                print("Connection Restored! Reactivating UI elements...")
                self.handle_connected()


    def handle_disconnected(self):
        """
        This function handles when the connection is lost.
        """
        self.message_connection.show()
        self.radio_jog.deactivate()
        self.radio_point.deactivate()
        self.press_run.deactivate()
        self.press_home.deactivate()

        for entry_r, entry_theta in self.point_entries:
            entry_r.disable()
            entry_theta.disable()

        
    def handle_connected(self):
        """
        This function handles when the connection is restored.
        """
        print("Enabling UI elements after connection is restored.")

        self.message_connection.hide()

        self.radio_jog.activate()
        self.radio_point.activate()
        self.press_run.activate()
        self.press_home.activate()

        for entry_r, entry_theta in self.point_entries:
            entry_r.enable()
            entry_theta.enable()

    def plot_graph(self):
        self.canvas_field.delete("plot") 

        if self.operation_mode == "Jog":
            for i, (r, theta) in enumerate(self.jog_points):
                print(f"[Jog Mode] Saved point {i+1}: r = {r}, theta = {theta}")

                x, y = self.polar_to_cartesian(r, theta)

                scale_factor = 1
                x_plot = 280 + x * scale_factor
                y_plot = 400 - y * scale_factor
                x_plot = max(40, min(x_plot, 520))
                y_plot = max(160, min(y_plot, 640))

                print(f"Calculated x, y = {x:.2f}, {y:.2f}")
                print(f"Plotting at x = {x_plot}, y = {y_plot}")

                self.canvas_field.create_oval(x_plot-3, y_plot-3, x_plot+3, y_plot+3, fill="blue", tags="plot")

        elif self.operation_mode == "Point":
            for r, theta in self.point_mode_points:
                print(f"[Point Mode] Drawing point at r = {r}, theta = {theta}")

                x, y = self.polar_to_cartesian(r, theta)

                scale_factor = 1
                x_plot = 280 + x * scale_factor
                y_plot = 400 - y * scale_factor
                x_plot = max(40, min(x_plot, 520))
                y_plot = max(160, min(y_plot, 640))

                print(f"Calculated x, y = {x:.2f}, {y:.2f}")
                print(f"Plotting at x = {x_plot}, y = {y_plot}")

                self.canvas_field.create_oval(x_plot-3, y_plot-3, x_plot+3, y_plot+3, fill="green", tags="plot")


    def handle_ui_change(self):
        """
        This function handles updating the UI (according to protocol status) 
        """ 
        if self.mode == "Protocol":  
            r_current = self.protocol_rt.r_position if hasattr(self.protocol_rt, 'r_position') else 0.0
            theta_current = self.protocol_rt.theta_position if hasattr(self.protocol_rt, 'theta_position') else 0.0

            self.text_r_pos_num.change_text(r_current)
            self.text_theta_pos_num.change_text(theta_current)


            
            up, down = self.protocol_rt.read_up_down_order()
            self.status_sensor_1.change_text("TRIGGERED" if up else "CLEAR",
                                            color=Color.red if up else Color.blue)
            self.status_sensor_2.change_text("TRIGGERED" if down else "CLEAR",
                                            color=Color.red if down else Color.blue)

            if self.operation_mode == "Jog":
                for i, (entry_r, entry_theta) in enumerate(self.point_entries):
                    if i == 0 or float(entry_r.get_value()) != 0.0 or float(entry_theta.get_value()) != 0.0:
                        continue

                    entry_r.set_text(str(r_current))
                    entry_theta.set_text(str(theta_current))
                    break 
            
        self.text_r_pos_num.change_text(self.protocol_rt.r_position if hasattr(self.protocol_rt, 'r_position') else "N/A")
        self.text_theta_pos_num.change_text(self.protocol_rt.theta_position if hasattr(self.protocol_rt, 'theta_position') else "N/A")

        self.text_r_spd_num.change_text(self.protocol_rt.v_r if hasattr(self.protocol_rt, 'v_r') else "N/A")  
        self.text_theta_spd_num.change_text(self.protocol_rt.v_theta if hasattr(self.protocol_rt, 'v_theta') else "N/A")  

        self.text_r_acc_num.change_text(self.protocol_rt.a_r if hasattr(self.protocol_rt, 'a_r') else "N/A")  
        self.text_theta_acc_num.change_text(self.protocol_rt.a_theta if hasattr(self.protocol_rt, 'a_theta') else "N/A")  

        self.text_rt_status_value.change_text(self.protocol_rt.r_theta_moving_status if hasattr(self.protocol_rt, 'r_theta_moving_status') else "N/A") 


        if self.protocol_rt.r_theta_moving_status == "Idle":
            if self.protocol_rt.r_theta_moving_status_before != "Idle":
                if (self.finish_run_flag == 0 or self.operation_mode == "Point") and self.press_run_flag:
                    self.handle_finish_moving()
                    self.finish_run_flag = 1

                if not self.press_home.pressed:
                    self.handle_finish_moving()

                if self.protocol_rt.r_theta_moving_status_before == "Go To Target":
                    self.running = False
                elif self.protocol_rt.r_theta_moving_status_before == "Home":
                    self.homing = False

            self.protocol_rt.r_theta_moving_status_before = "Idle"


    def handle_protocol_rt(self):
        """
        This function handles the protocol for R-Theta robot.
        """
        if self.protocol_rt is None:
            print("Protocol is not initialized. Skipping protocol.")
            return 
        
        if self.protocol_rt.usb_connect:
            if not self.protocol_rt.usb_connect_before:
                print("Reconnected to USB")
                self.handle_connected()
                self.protocol_rt.usb_connect_before = True

            if not self.protocol_rt.routine_normal:
                self.message_connection.change_text("Protocol Error from R-Theta Robot")
                self.handle_disconnected()
            else:
                if self.time_ms_rt >= 200:
                    self.time_ms_rt = 0
                    self.new_connection = self.protocol_rt.heartbeat()

                    if self.new_connection:  
                        self.protocol_rt.routine()  

                    self.end_time = time.time()
                    self.print_current_activity()
                    print(f"{(self.end_time - self.start_time) * 1000:.2f} ms\n")
                    self.start_time = time.time()

                self.handle_connection_change()
                self.handle_ui_change()

        else:
            self.message_connection.change_text("Please Connect the USB")
            self.handle_disconnected()
            self.protocol_rt.write_heartbeat()
            self.protocol_rt.usb_connect_before = False

    def print_current_activity(self):
        """
        This function prints the current activity for debugging in the terminal.
        """
        if self.running:
            print("Running")
        if self.homing:
            print("Homing")


if __name__ == "__main__":
    print("Starting Base System...") 
    app = App()  
    print("App initialized!") 
    app.task()   
    print("Task started!") 
    app.mainloop()