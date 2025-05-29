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
import threading

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

        self.data_lock = threading.Lock()
        self.shared_data = {
            "r": 0.0,
            "theta": 0.0,
            "v_r": 0.0,
            "v_theta": 0.0,
            "a_r": 0.0,
            "a_theta": 0.0,
            "status": "Idle",
            "status_before": "Idle",
            "up": False,
            "down": False,
            "ui_buttons": {           
                "home_pressed": False,
                "run_pressed": False, 
                "up_down_toggle": False,
                "mode_jog": True,
                "mode_point": False
                }
        }

        # create component
        self.create_components()
        # Counting Time
        self.time_ms_rt = 0 
        self.connection = True
        self.new_connection = True
        self.homing = False
        self.running = False  
        self.debug_mode = False

        # Prepare Protocol
        if self.mode == "Protocol":
            self.protocol_rt = Protocol_RT()
            self.connection = True
            self.new_connection = True
        else:
            self.protocol_rt = None  
            self.connection = False
            self.new_connection = False

    def debug_print(self, message):
        if self.debug_mode:
            print(f"[DEBUG] {message}")

    def protocol_loop(self):
        while True:
            try:
                loop_start = time.perf_counter()
                
                if self.protocol_rt is not None and hasattr(self.protocol_rt, 'usb_connect'):
                    if self.protocol_rt.usb_connect:
                        self.protocol_rt.heartbeat()
                        self.protocol_rt.routine()

                        with self.data_lock:
                            self.shared_data["r"] = getattr(self.protocol_rt, 'r_position', 0.0)
                            self.shared_data["theta"] = getattr(self.protocol_rt, 'theta_position', 0.0)
                            self.shared_data["v_r"] = getattr(self.protocol_rt, 'v_r', 0.0)
                            self.shared_data["v_theta"] = getattr(self.protocol_rt, 'v_theta', 0.0)
                            self.shared_data["a_r"] = getattr(self.protocol_rt, 'a_r', 0.0)
                            self.shared_data["a_theta"] = getattr(self.protocol_rt, 'a_theta', 0.0)
                            self.shared_data["status_before"] = self.shared_data["status"]
                            self.shared_data["status"] = getattr(self.protocol_rt, 'r_theta_moving_status', 'Unknown')
                            
                            # อ่าน up/down status แยกจาก UI buttons
                            try:
                                up, down = self.protocol_rt.read_up_down_order()
                                self.shared_data["up"] = up
                                self.shared_data["down"] = down
                            except Exception as e:
                                print(f"Error reading up/down status: {e}")
                                self.shared_data["up"] = False
                                self.shared_data["down"] = False
                            
                            # อ่าน UI button status แยกต่างหาก
                            try:
                                ui_buttons = getattr(self.protocol_rt, 'ui_buttons', {})
                                self.shared_data["ui_buttons"] = {
                                    "home_pressed": ui_buttons.get("home_pressed", False),
                                    "run_pressed": ui_buttons.get("run_pressed", False), 
                                    "up_down_toggle": ui_buttons.get("up_down_toggle", False),
                                    "mode_jog": ui_buttons.get("mode_jog", True),
                                    "mode_point": ui_buttons.get("mode_point", False)
                                }
                            except Exception as e:
                                print(f"Error updating UI button status: {e}")
                                # ตั้งค่า default เมื่อเกิด error
                                self.shared_data["ui_buttons"] = {
                                    "home_pressed": False,
                                    "run_pressed": False,
                                    "up_down_toggle": False,
                                    "mode_jog": True,
                                    "mode_point": False
                                }

                    else:
                        # เมื่อไม่ได้เชื่อมต่อ ให้ reset ข้อมูล
                        with self.data_lock:
                            self.shared_data["status"] = "Disconnected"
                            # Reset UI buttons เมื่อไม่เชื่อมต่อ
                            self.shared_data["ui_buttons"] = {
                                "home_pressed": False,
                                "run_pressed": False,
                                "up_down_toggle": False,
                                "mode_jog": True,
                                "mode_point": False
                            }
                            
                time.sleep(0.05)  # 50ms - ลดจาก 10ms เพื่อประสิทธิภาพ
                
            except Exception as e:
                print(f"Protocol loop error: {e}")
                with self.data_lock:
                    self.shared_data["status"] = "Error"
                    # Reset UI buttons เมื่อเกิด error
                    self.shared_data["ui_buttons"] = {
                        "home_pressed": False,
                        "run_pressed": False,
                        "up_down_toggle": False,
                        "mode_jog": True,
                        "mode_point": False
                    }
                time.sleep(0.1)


    def polar_to_cartesian(self, r, theta):
        try:
            theta_rad = math.radians(float(theta))
            x = float(r) * math.cos(theta_rad)
            y = float(r) * math.sin(theta_rad)
            return x, y
        except (ValueError, TypeError):
            return 0, 0
            

    def task(self):
        loop_start = time.perf_counter()
        
        # จัดการ UI events (ไม่เปลี่ยน)
        self.handle_toggle_up_down()
        self.handle_radio_operation() 
        self.handle_press_home()
        self.handle_press_run()
        self.handle_ui_change()
        
        # จัดการ connection status (ปรับให้เช็คน้อยลง)
        if self.mode == "Protocol" and self.protocol_rt:
            # เช็ค connection ทุก 5 รอบ แทนทุกรอบ
            if not hasattr(self, '_connection_check_counter'):
                self._connection_check_counter = 0
            
            self._connection_check_counter += 1
            if self._connection_check_counter >= 5:
                self.handle_connection_change()
                self._connection_check_counter = 0
        
        # Jog mode tracking (ปรับให้เรียกน้อยลง)
        if self.operation_mode == "Jog" and self.protocol_rt:
            with self.data_lock:
                r = self.shared_data["r"]
                theta = self.shared_data["theta"]
            
            # เช็คว่าตำแหน่งเปลี่ยนจริงๆ ก่อนจะเรียก handle_jog_mode_movement
            if not hasattr(self, '_last_jog_position'):
                self._last_jog_position = (0, 0)
                
            if (r, theta) != self._last_jog_position and (r != 0 or theta != 0):
                self.handle_jog_mode_movement(r, theta)
                self._last_jog_position = (r, theta)
        
        loop_end = time.perf_counter()
        elapsed_ms = (loop_end - loop_start) * 1000
        
        # แสดง performance เฉพาะเมื่อช้ามาก (เพิ่มจาก 20ms เป็น 50ms)
        if elapsed_ms > 50:  # เพิ่มจาก 20ms เป็น 50ms
            print(f"[Task Loop] took {elapsed_ms:.2f} ms (VERY SLOW)")
        
        # เพิ่ม sleep เล็กน้อยถ้า task ทำงานเร็วเกินไป
        if elapsed_ms < 5:  # ถ้าทำงานเร็วกว่า 5ms
            time.sleep(0.002)  # พัก 2ms
        
        self.after(10, self.task)

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
        self.message_info = MessageBox(canvas=self.canvas_field, x=555, y=700,width_field=320, text="", color=Color.blue, align="Center", font_name="Inter-Regular", size=8)
        self.message_info.hide()
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


    def setup_point_mode_button(self):
        """ตั้งค่าปุ่มสำหรับ Point Mode"""
        self.press_run.activate()
        self.press_run.change_text("Run")
        self.message_info.hide()

    def setup_jog_mode_button(self):
        """ตั้งค่าปุ่มสำหรับ Jog Mode"""
        self.press_run.deactivate()
        self.press_run.change_text("Jog Mode")
        self.message_info.change_text("Use manual controls in Jog Mode")
        self.message_info.show()


    def simulate_movement_complete(self):
        """จำลองการทำงานเสร็จสิ้นใน Graphic Mode"""
        print("Simulated movement completed!")
        self.running = False
        self.press_run.activate()
        
        # รีเซ็ตข้อความปุ่ม
        if self.operation_mode == "Point":
            self.press_run.change_text("Run")
        else:
            self.press_run.change_text("Run")

    def simulate_graphic_data(self):
        """จำลองข้อมูลสำหรับ Graphic Mode"""
        if self.mode == "Graphic":
            # จำลองข้อมูลตำแหน่ง
            import random
            self.text_r_pos_num.change_text(f"{random.uniform(0, 100):.2f}")
            self.text_theta_pos_num.change_text(f"{random.uniform(-45, 45):.2f}")
            self.text_rt_status_value.change_text("Simulation")

    def handle_jog_mode_movement(self, r, theta):
        if len(self.jog_points) >= 10:
            self.jog_points.pop(0)  

        self.jog_points.append((r, theta))

        # เมื่อครบ 10 จุด ส่งกลับไปหุ่นยนต์เพื่อ acknowledgment
        if self.protocol_rt and len(self.jog_points) == 10:
            self.protocol_rt.write_target_positions(self.jog_points)
            print("✅ 10 positions sent to robot for acknowledgment - Ready for joystick Run!")
            
        self.plot_graph()
        self.debug_print(f"[Jog Mode] Saved point {len(self.jog_points)}/10: r = {r}, theta = {theta}")  # ใช้ debug_print แทน print

    def handle_radio_operation(self, event=None):
        # ใช้การตรวจสอบแบบเดิมจากโค้ดต้นฉบับ
        if self.radio_point.on and self.operation_mode != "Point":
            self.operation_mode = "Point"
            print("Switched to Point Mode")
            
            # ปิด Jog Mode และรีเซ็ต
            self.radio_jog.turn_off()
            
            # ส่งคำสั่งไปหุ่นยนต์ (เฉพาะ Protocol Mode)
            if self.mode == "Protocol" and self.protocol_rt: 
                self.protocol_rt.write_base_system_status("Run Point Mode")
            
            # จัดการ UI สำหรับ Point Mode (ทำงานทั้ง Graphic และ Protocol)
            self.jog_points.clear()  
            self.entry_r.show()
            self.entry_theta.show()
            self.entry_r.enable()
            self.entry_theta.enable()
            self.press_run.activate()
            self.press_run.change_text("Run") 
            self.message_info.hide()

        # เช็ค Jog Mode กำลังถูกเลือก  
        elif self.radio_jog.on and self.operation_mode != "Jog":
            self.operation_mode = "Jog"
            print("Switched to Jog Mode")
            
            # ปิด Point Mode และรีเซ็ต
            self.radio_point.turn_off()
            
            # ส่งคำสั่งไปหุ่นยนต์ (เฉพาะ Protocol Mode)
            if self.mode == "Protocol" and self.protocol_rt:  
                self.protocol_rt.write_base_system_status("Run Jog Mode")
            
            # จัดการ UI สำหรับ Jog Mode (ทำงานทั้ง Graphic และ Protocol)
            self.entry_r.hide()
            self.entry_theta.hide()
            self.entry_r.disable()
            self.entry_theta.disable()
            self.point_mode_points.clear() 
            
            # ใน Graphic Mode ให้ปุ่ม Run ยังใช้งานได้ (จำลองการทำงาน)
            if self.mode == "Graphic":
                self.press_run.activate()
                self.press_run.change_text("Run")
                self.message_info.change_text("Graphic Mode - Simulation only")
            else:
                self.press_run.deactivate()
                self.press_run.change_text("Jog Mode")
                self.message_info.change_text("Use manual controls in Jog Mode")
            
            self.message_info.show()


    def handle_toggle_up_down(self):
        if self.toggle_up_down.pressed:
            if self.mode == "Graphic":
                if not self.toggle_up_down.on:
                    print("🔼 Moving UP (Graphic Mode)")
                    self.toggle_up_down.toggle_on()
                else:
                    print("🔽 Moving DOWN (Graphic Mode)")
                    self.toggle_up_down.toggle_off()

            elif self.mode == "Protocol" and self.protocol_rt is not None:
                # โค้ด Protocol เดิม...
                if not self.toggle_up_down.on:
                    print("🔼 Moving UP")
                    self.protocol_rt.write_up_down_order(up=1, down=0)
                    self.toggle_up_down.toggle_on()
                else:
                    print("🔽 Moving DOWN")
                    self.protocol_rt.write_up_down_order(up=0, down=1)
                    self.toggle_up_down.toggle_off()


                pass
                
            self.update_idletasks()
            self.toggle_up_down.pressed = False



    def out_entry(self, event):
        """
        This function is called when user clicks outside the entry or presses enter.
        """
        if self.operation_mode == "Point":
            validation_result = self.validate_entry()
            print(f"Validation result: {validation_result}")
            
            if validation_result == "Normal":
                # เก็บค่าปัจจุบันก่อนการอัพเดท
                current_r = self.entry_r.get_value()
                current_theta = self.entry_theta.get_value()
                
                try:
                    # อัพเดทค่า target เฉพาะเมื่อมีการกรอกข้อมูล
                    if current_r:
                        self.target_r = float(current_r)
                    if current_theta:
                        self.target_theta = float(current_theta)

                    print(f"Target Position: r = {self.target_r}, theta = {self.target_theta}")

                    # ไม่ต้อง set_text กลับไป เพราะจะทำให้ค่าหาย
                    # entry_r.set_text(str(self.target_r))  # ลบบรรทัดนี้
                    # entry_theta.set_text(str(self.target_theta))  # ลบบรรทัดนี้

                except ValueError as e:
                    print(f"Invalid input: {e}")
                    
            print(f"Final Target Position: r = {getattr(self, 'target_r', 0)}, theta = {getattr(self, 'target_theta', 0)}")

        elif self.operation_mode == "Jog":
            print("Jog Mode - No Entry Update Required")


    def validate_entry(self):
        """
        This function validates input in entry and shows an error message if invalid.
        แก้ไขให้ไม่ clear ค่าใน entry เมื่อ validation ผ่าน
        """
        if self.operation_mode == "Point":
            validate_point_result = "Normal"
            has_error = False

            for i, (entry_r, entry_theta) in enumerate(self.point_entries):
                # เก็บค่าปัจจุบันก่อนการตรวจสอบ
                r_text = entry_r.get_value()
                theta_text = entry_theta.get_value()
                
                # ตรวจสอบ r value
                if r_text:  # เช็คเฉพาะเมื่อมีการกรอกข้อมูล
                    try:
                        r_value = float(r_text)
                        if r_value < 0 or r_value > 500:
                            entry_r.error()
                            validate_point_result = f"Error: r {r_value} is out of range (0-500 mm)"
                            has_error = True
                        else:
                            entry_r.normal()
                    except ValueError:
                        entry_r.error()
                        validate_point_result = f"Invalid r input at Point {i+1}"
                        has_error = True
                else:
                    entry_r.normal()  # ไม่มี error เมื่อยังไม่กรอก

                # ตรวจสอบ theta value
                if theta_text:  # เช็คเฉพาะเมื่อมีการกรอกข้อมูล
                    try:
                        theta_value = float(theta_text)
                        if theta_value < -180 or theta_value > 180:
                            entry_theta.error()
                            validate_point_result = f"Error: θ {theta_value} is out of range (-180° to 180°)"
                            has_error = True
                        else:
                            entry_theta.normal()
                    except ValueError:
                        entry_theta.error()
                        validate_point_result = f"Invalid theta input at Point {i+1}"
                        has_error = True
                else:
                    entry_theta.normal()  # ไม่มี error เมื่อยังไม่กรอก

            # แสดงหรือซ่อน error message
            if has_error:
                self.message_error.change_text(validate_point_result)
                self.message_error.show()
                self.press_run.deactivate()
            else:
                self.message_error.hide()
                # เปิดปุ่ม Run เฉพาะเมื่อมีข้อมูลครบ
                if (self.entry_r.get_value() and self.entry_theta.get_value() and
                    not self.running and not self.homing and self.connection):
                    if self.mode == "Protocol":
                        if (self.protocol_rt and 
                            getattr(self.protocol_rt, 'usb_connect', False) and 
                            getattr(self.protocol_rt, 'routine_normal', False)):
                            self.press_run.activate()
                    else:
                        self.press_run.activate()

            return validate_point_result
            
        elif self.operation_mode == "Jog":
            self.message_error.hide()
            self.message_info.change_text("Manual control mode - Use joystick or keyboard")
            self.message_info.show()
            self.press_run.deactivate()
            return "Jog Mode"
        
        else:
            self.message_error.hide()
            self.message_info.hide()
            if not self.running and not self.homing and self.connection:
                self.press_run.activate()
            return "Normal"



    def setup_entry_bindings(self):
        """
        ตั้งค่า event bindings สำหรับ entry fields ให้เหมาะสม
        """
        # ใช้ <KeyRelease> แทน <FocusOut> เพื่อ real-time validation
        self.entry_r.bind("<KeyRelease>", self.on_entry_change)
        self.entry_r.bind("<Return>", self.out_entry)
        self.entry_theta.bind("<KeyRelease>", self.on_entry_change)  
        self.entry_theta.bind("<Return>", self.out_entry)

    def on_entry_change(self, event):
        """
        เรียกเมื่อมีการพิมพ์ใน entry field - ทำ validation แบบ real-time
        """
        if self.operation_mode == "Point":
            # Validation แบบไม่แสดง error message (เฉพาะสี)
            for entry_r, entry_theta in self.point_entries:
                r_text = entry_r.get_value()
                theta_text = entry_theta.get_value()
                
                # ตรวจสอบ r
                if r_text:
                    try:
                        r_value = float(r_text)
                        if 0 <= r_value <= 500:
                            entry_r.normal()
                        else:
                            entry_r.error()
                    except ValueError:
                        entry_r.error()
                else:
                    entry_r.normal()
                    
                # ตรวจสอบ theta  
                if theta_text:
                    try:
                        theta_value = float(theta_text)
                        if -180 <= theta_value <= 180:
                            entry_theta.normal()
                        else:
                            entry_theta.error()
                    except ValueError:
                        entry_theta.error()
                else:
                    entry_theta.normal()


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
        This function handles when user press "Home" button
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
        if self.protocol_rt is not None:
            with self.data_lock:
                current_status = self.shared_data["status"]
                
            if current_status == "Idle":  
                print("Home Position Reached!")
                self.homing = False
                self.radio_jog.activate()
                self.radio_point.activate()
                self.press_home.activate()

                if self.previous_mode == "Jog":
                    self.radio_jog.turn_on()
                    self.radio_point.turn_off()
                    self.operation_mode = "Jog"
                    # รีเซ็ตปุ่มให้ตรงกับ Jog Mode
                    self.setup_jog_mode_button()
                    print("Switched back to Jog Mode")
                    
                elif self.previous_mode == "Point":
                    self.radio_jog.turn_off()
                    self.radio_point.turn_on()
                    self.operation_mode = "Point"
                    # รีเซ็ตปุ่มให้ตรงกับ Point Mode
                    self.setup_point_mode_button()
                    print("Switched back to Point Mode")
                    for entry_r, entry_theta in self.point_entries:
                        entry_r.enable()
                        entry_theta.enable()
            else:
                self.after(500, self.check_home_status)
        else:
            print("Warning: Protocol_RT is not initialized.")
            self.homing = False
            self.radio_jog.activate()
            self.radio_point.activate()
            self.press_home.activate()

    def handle_press_run(self, event=None):
        if self.press_run.pressed:
            if self.operation_mode == "Point":
                try:
                    r = float(self.entry_r.get_value()) if self.entry_r.get_value() else 0.0
                    theta = float(self.entry_theta.get_value()) if self.entry_theta.get_value() else 0.0

                    self.point_mode_points.clear()  
                    self.point_mode_points.append((r, theta))  

                    print(f"[Point Mode] Moving to r = {r}, theta = {theta}")

                    # ส่งคำสั่งไปหุ่นยนต์จริง (เฉพาะ Protocol Mode)
                    if self.mode == "Protocol" and self.protocol_rt is not None:
                        self.protocol_rt.write_goal_point(r, theta)
                    elif self.mode == "Graphic":
                        print("[Graphic Mode] Simulating movement...")

                    self.plot_graph()

                except ValueError:
                    print("Invalid input for r, theta")
            
            elif self.operation_mode == "Jog" and self.mode == "Graphic":
                # ใน Graphic Mode, Jog Mode จะจำลองการทำงาน
                print("[Graphic Mode] Jog simulation - Use mouse to add points on grid")
                return  # ไม่ต้องทำอะไรเพิ่ม

            # จำลองการทำงานใน Graphic Mode
            if self.mode == "Graphic":
                print("Simulating robot movement...")
                self.running = True
                self.press_run.deactivate()
                # จำลองการทำงาน 2 วินาที
                self.after(2000, self.simulate_movement_complete)
            else:
                # Protocol Mode ทำงานตามปกติ
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

        # ใช้ shared_data แทน protocol_rt โดยตรง
        with self.data_lock:
            current_status = self.shared_data["status"]
            
        if current_status == "Idle":  
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
        if self.protocol_rt is None:
            return
            
        current_connection = getattr(self.protocol_rt, 'usb_connect', False)

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

    def auto_reconnect(self):
        try:
            if not self.client.is_socket_open():
                self.client.connect()
                print("Reconnection successful")
        except Exception as e:
            print(f"Reconnection failed: {e}")

    def plot_graph(self):
        self.canvas_field.delete("plot") 

        if self.operation_mode == "Jog":
            for i, (r, theta) in enumerate(self.jog_points):
                # print(f"[Jog Mode] Saved point {i+1}: r = {r}, theta = {theta}")

                x, y = self.polar_to_cartesian(r, theta)

                scale_factor = 1
                x_plot = 280 + x * scale_factor
                y_plot = 400 - y * scale_factor
                x_plot = max(40, min(x_plot, 520))
                y_plot = max(160, min(y_plot, 640))

                # print(f"Calculated x, y = {x:.2f}, {y:.2f}")
                # print(f"Plotting at x = {x_plot}, y = {y_plot}")

                self.canvas_field.create_oval(x_plot-3, y_plot-3, x_plot+3, y_plot+3, fill="blue", tags="plot")

        elif self.operation_mode == "Point":
            for r, theta in self.point_mode_points:
                # print(f"[Point Mode] Drawing point at r = {r}, theta = {theta}")

                x, y = self.polar_to_cartesian(r, theta)

                scale_factor = 1
                x_plot = 280 + x * scale_factor
                y_plot = 400 - y * scale_factor
                x_plot = max(40, min(x_plot, 520))
                y_plot = max(160, min(y_plot, 640))

                # print(f"Calculated x, y = {x:.2f}, {y:.2f}")
                # print(f"Plotting at x = {x_plot}, y = {y_plot}")

                self.canvas_field.create_oval(x_plot-3, y_plot-3, x_plot+3, y_plot+3, fill="green", tags="plot")

    def handle_movement_finished(self, previous_status):
        """จัดการเมื่อการเคลื่อนไหวเสร็จสิ้น"""
        if (self.finish_run_flag == 0 or self.operation_mode == "Point") and self.press_run_flag:
            self.handle_finish_moving()
            self.finish_run_flag = 1

        if not self.press_home.pressed:
            self.handle_finish_moving()

        if previous_status == "Go To Target":
            self.running = False
        elif previous_status == "Home":
            self.homing = False
    

    def handle_ui_change(self):
        if self.mode != "Protocol":
            return

        # อ่านข้อมูลครั้งเดียวด้วย lock
        with self.data_lock:
            data_snapshot = self.shared_data.copy()

        # เช็คว่ามีข้อมูลเก่าสำหรับเปรียบเทียบหรือไม่
        if not hasattr(self, '_last_ui_data'):
            self._last_ui_data = {}

        r = data_snapshot["r"]
        theta = data_snapshot["theta"]
        v_r = data_snapshot["v_r"]
        v_theta = data_snapshot["v_theta"]
        a_r = data_snapshot["a_r"] 
        a_theta = data_snapshot["a_theta"]
        status = data_snapshot["status"]
        status_before = data_snapshot["status_before"]
        up = data_snapshot["up"]
        down = data_snapshot["down"]

        # อัพเดท UI เฉพาะเมื่อมีการเปลี่ยนแปลง
        if self._last_ui_data.get("r") != r:
            self.text_r_pos_num.change_text(f"{r:.2f}")
            
        if self._last_ui_data.get("theta") != theta:
            self.text_theta_pos_num.change_text(f"{theta:.2f}")
            
        if self._last_ui_data.get("v_r") != v_r:
            self.text_r_spd_num.change_text(f"{v_r:.2f}")
            
        if self._last_ui_data.get("v_theta") != v_theta:
            self.text_theta_spd_num.change_text(f"{v_theta:.2f}")
            
        if self._last_ui_data.get("a_r") != a_r:
            self.text_r_acc_num.change_text(f"{a_r:.2f}")
            
        if self._last_ui_data.get("a_theta") != a_theta:
            self.text_theta_acc_num.change_text(f"{a_theta:.2f}")
            
        if self._last_ui_data.get("status") != status:
            self.text_rt_status_value.change_text(status)

        # อัพเดท sensor status เฉพาะเมื่อเปลี่ยนแปลง
        if self._last_ui_data.get("up") != up:
            self.status_sensor_1.change_text("TRIGGERED" if up else "CLEAR", 
                                            color=Color.red if up else Color.blue)
            
        if self._last_ui_data.get("down") != down:
            self.status_sensor_2.change_text("TRIGGERED" if down else "CLEAR", 
                                            color=Color.red if down else Color.blue)

        # อัพเดทสถานะการเคลื่อนไหว (เช็คเฉพาะเมื่อ status เปลี่ยน)
        if (self._last_ui_data.get("status") != status and 
            status == "Idle" and status_before != "Idle"):
            self.handle_movement_finished(status_before)
        
        # เก็บข้อมูลปัจจุบันเป็นข้อมูลเก่าสำหรับครั้งถัดไป
        self._last_ui_data = {
            "r": r,
            "theta": theta,
            "v_r": v_r,
            "v_theta": v_theta,
            "a_r": a_r,
            "a_theta": a_theta,
            "status": status,
            "status_before": status_before,
            "up": up,
            "down": down
        }
        
        # อัพเดท status_before ใน shared_data เฉพาะเมื่อจำเป็น
        if status != status_before:
            with self.data_lock:
                self.shared_data["status_before"] = status

        try:
            ui_buttons = data_snapshot.get("ui_buttons", {})

            # ตัวอย่างการใช้งาน - sync hardware button กับ UI
            if ui_buttons.get("home_pressed") and not self.press_home.pressed:
                print("Hardware Home button pressed!")
                # สามารถเรียก self.handle_press_home() ได้
                
            if ui_buttons.get("run_pressed") and not self.press_run.pressed:
                print("Hardware Run button pressed!")
                # สามารถเรียก self.handle_press_run() ได้
        except KeyError as e:
            print(f"ui_buttons not found in shared_data: {e}")
        except Exception as e:
            print(f"Error handling UI buttons: {e}")
    
if __name__ == "__main__":
    print("Starting Base System...") 
    app = App()  
    print("App initialized!")
    
    if app.mode == "Protocol" and app.protocol_rt:
        threading.Thread(target=app.protocol_loop, daemon=True).start()
        print("Protocol thread started!")
    
    app.task()   
    print("Task started!") 
    app.mainloop()