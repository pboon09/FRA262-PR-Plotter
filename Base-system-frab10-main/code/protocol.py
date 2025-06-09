import platform
import struct
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.client import ModbusTcpClient

# ----------------------------------- Config this variable before using ----------------------------------- 
device_port = "COM14"
# example: for os -> device_port = "/dev/cu.usbmodem14103"
#          for window -> device_port = "COM3"
# ---------------------------------------------------------------------------------------------------------

class Binary():
    """
    Binary Class
    """
    def decimal_to_binary(self, decimal_num):
        """
        This function converts base 10 to base 2
        """
        binary_num = ""
        while decimal_num > 0:
            binary_num = str(decimal_num % 2) + binary_num
            decimal_num = decimal_num // 2
        # Fill to 16 digits with 0
        if len(binary_num) < 16:
            binary_num = "0"*(16-len(binary_num)) + binary_num
        return binary_num
        
    def binary_to_decimal(self, binary_num):
        """
        This function converts base 2 to base 10
        """
        decimal_num = 0
        for i in range(len(binary_num)):
            decimal_num += int(binary_num[i]) * (2 ** (len(binary_num)-i-1))
        return decimal_num
    
    def binary_crop(self, digit, binary_num):
        """
        This function crops the last n digits of the binary number
        """
        return binary_num[len(binary_num)-digit:]

    def binary_twos_complement(self, number):
        """
        This functions converts the (negative) number to its 16-bit two's complement representation
        """
        if number < 0:
            number = (1 << 16) + number  # Adding 2^16 to the negative number
        return number
    
    def binary_reverse_twos_complement(self, number):
        """
        This functions converts the 16-bit two's complement number back to its original signed representation 
        """
        if number & (1 << 15):  # Check if the most significant bit is 1
            number = number - (1 << 16)  # Subtract 2^16 from the number
        return number

class Protocol_RT(Binary):
    """
    Protocol RT (r-theta) Class
    """
    def __init__(self):
        self.os = platform.platform()[0].upper()
        if self.os == 'M': #Mac
            self.port = device_port
        elif self.os == 'W': #Windows        
            self.port = device_port

        self.usb_connect = False
        self.usb_connect_before = False

        self.slave_address = 0x15
        self.register = []
        self.routine_normal = True

        self.r_theta_moving_status_before = "Idle"
        self.r_theta_moving_status = "Idle"
        self.r_position = 0.0
        self.theta_position = 0.0
        self.v_r = 0.0         
        self.v_theta = 0.0      
        self.a_r = 0.0          
        self.a_theta = 0.0    


        self.target_positions = [0] * 10


        self.client = ModbusClient(method="rtu", port=self.port, stopbits=1, bytesize=8, parity="E", baudrate=115200)
        print('Modbus Connection Status :', self.client.connect())

        self.write_heartbeat() # Write heartbeat as "Hi"
        
    def heartbeat(self):
        """"
        Check heartbeat from robot; expects response "Ya" (22881), then sends "Hi"
        """
        if self.read_heartbeat() == 22881: # Read heartbeat as "Ya"
            self.write_heartbeat() # Write heartbeat as "Hi"
            return True
        else:
            return False

    # def read_ui_button_status(self):
    #     """
    #     Read UI button status from register 0x06 - แก้ไข error handling
    #     """
    #     try:
    #         response = self.client.read_holding_registers(
    #             address=0x06, 
    #             count=1, 
    #             slave=self.slave_address
    #         )
            
    #         # เช็ค response
    #         if hasattr(response, 'isError') and response.isError():
    #             print(f"Read UI button error: {response}")
    #             return {
    #                 "home_pressed": False,
    #                 "run_pressed": False,
    #                 "up_down_toggle": False,
    #                 "mode_jog": True,
    #                 "mode_point": False
    #             }
                
    #         if not hasattr(response, 'registers') or len(response.registers) == 0:
    #             print("Invalid UI button response")
    #             return {
    #                 "home_pressed": False,
    #                 "run_pressed": False,
    #                 "up_down_toggle": False,
    #                 "mode_jog": True,
    #                 "mode_point": False
    #             }
                
    #         status = response.registers[0]
            
    #         return {
    #             "home_pressed": (status & 0x01) > 0,     # Bit 0
    #             "run_pressed": (status & 0x02) > 0,      # Bit 1  
    #             "up_down_toggle": (status & 0x04) > 0,   # Bit 2
    #             "mode_jog": (status & 0x08) > 0,         # Bit 3
    #             "mode_point": (status & 0x10) > 0        # Bit 4
    #         }
            
    #     except Exception as e:
    #         print(f"Read UI button exception: {e}")
    #         return {
    #             "home_pressed": False,
    #             "run_pressed": False,
    #             "up_down_toggle": False,
    #             "mode_jog": True,
    #             "mode_point": False
    #         }

    def routine(self):
        try:

            if not self.client or not hasattr(self.client, 'connect'):
                print("Invalid Modbus client")
                self.routine_normal = False
                self.usb_connect = False
                return
            test_response = self.client.read_holding_registers(
                address=0x00, 
                count=1, 
                slave=self.slave_address
            )
            if hasattr(test_response, 'isError') and test_response.isError():
                print(f"Connection test failed: {test_response}")
                self.routine_normal = False
                self.usb_connect = False
                return
                
            if not hasattr(test_response, 'registers'):
                print("Invalid test response - no registers")
                self.routine_normal = False
                self.usb_connect = False
                return
            
            register_response = self.client.read_holding_registers(
                address=0x00, 
                count=0x46, 
                slave=self.slave_address
            )
            
            if hasattr(register_response, 'isError') and register_response.isError():
                print(f"Error reading registers: {register_response}")
                self.routine_normal = False
                self.usb_connect = False
                return
                
            if not hasattr(register_response, 'registers'):
                print(f"Invalid register response: {type(register_response)}")
                self.routine_normal = False
                self.usb_connect = False
                return
                
            self.register = register_response.registers
            if len(self.register) < 0x46:
                print(f"Incomplete register data: expected 70, got {len(self.register)}")
                self.routine_normal = False
                return
            
            print(f"Successfully read {len(self.register)} registers")
            
            try:
                self.read_r_theta_moving_status()
            except Exception as e:
                print(f"Error reading moving status: {e}")
                
            try:
                self.read_r_theta_actual_motion()
            except Exception as e:
                print(f"Error reading actual motion: {e}")
                
            try:
                self.read_target_positions()
            except Exception as e:
                print(f"Error reading target positions: {e}")

            try:
                limit_switch = self.read_limit_switch_status()
                if limit_switch["limit_up"]:
                    servo_status = "UP"
                elif limit_switch["limit_down"]:
                    servo_status = "DOWN"
                else:
                    servo_status = "MOVING"
            except Exception as e:
                print(f"Error reading limit switch: {e}")
                servo_status = "UNKNOWN"

            # try:
            #     self.ui_buttons = self.read_ui_button_status()
            # except Exception as e:
            #     print(f"Error reading UI button status: {e}")
            #     self.ui_buttons = {
            #         "home_pressed": False,
            #         "run_pressed": False,
            #         "up_down_toggle": False,
            #         "mode_jog": True,
            #         "mode_point": False
            #     }

            self.routine_normal = True
            self.usb_connect = True
            
        except Exception as e:
            print(f"Routine critical error: {e}")
            self.routine_normal = False
            self.usb_connect = False


    def read_limit_switch_status(self):
        """
        Read limit switch status - แก้ไข error handling
        """
        try:
            response = self.client.read_holding_registers(
                address=0x03, 
                count=1, 
                slave=self.slave_address
            )
            
            if hasattr(response, 'isError') and response.isError():
                print(f"Limit switch read error: {response}")
                return {"limit_up": False, "limit_down": False}
                
            if not hasattr(response, 'registers') or len(response.registers) == 0:
                print("Invalid limit switch response")
                return {"limit_up": False, "limit_down": False}
            
            status = response.registers[0]
            limit_down = (status & 0x01) > 0  # Bit 0
            limit_up = (status & 0x02) > 0    # Bit 1
            
            return {
                "limit_up": limit_up,
                "limit_down": limit_down
            }
            
        except Exception as e:
            print(f"Limit switch exception: {e}")
            return {"limit_up": False, "limit_down": False}
    

    def read_heartbeat(self):
        """
        Read heartbeat value from robot; expects 22881 for "Ya"
        """
        try:
            response = self.client.read_holding_registers(
                address=0x00, 
                count=1, 
                slave=self.slave_address
            )
            
            if hasattr(response, 'isError') and response.isError():
                print(f"Read heartbeat error: {response}")
                return "Error"
            
            if not hasattr(response, 'registers'):
                print(f"Response has no registers attribute: {type(response)}")
                return "Error"
                
            if len(response.registers) == 0:
                print("Empty registers response")
                return "Error"
                
            heartbeat_value = response.registers[0]
            print(f"Heartbeat response: {heartbeat_value}")
            return heartbeat_value
            
        except Exception as e:
            print(f"Read heartbeat exception: {e}")
            return "Error"
    
    def write_heartbeat(self):
        """
        Send heartbeat signal to robot (writes 18537 for "Hi")
        """
        try:
            result = self.client.write_register(
                address=0x00, 
                value=18537,  # "Hi"
                slave=self.slave_address
            )
            
            if hasattr(result, 'isError') and result.isError():
                print(f"Write heartbeat error: {result}")
                self.usb_connect = False
            else:
                self.usb_connect = True
                print("Heartbeat sent successfully")
                    
        except Exception as e:
            print(f"Write heartbeat exception: {e}")
            self.usb_connect = False


    def write_base_system_status(self, command):
        if command == "Home":
            self.base_system_status_register = 0b0001
        elif command == "Run Jog Mode":
            self.base_system_status_register = 0b0010
        elif command == "Run Point Mode":
            self.base_system_status_register = 0b0100
        elif command == "Go To Target":
            self.base_system_status_register = 0b1000
        
        else:
            print(f"Invalid Command: {command}")
            return  

        self.client.write_register(address=0x01, value=self.base_system_status_register, slave=self.slave_address)
        print(f"Write Base System Status: {command}")


    def read_r_theta_actual_motion(self):
        """
        Read the actual motion values from the robot.
        """
        try:
            if len(self.register) <= 0x16:
                print("register ยังไม่พอสำหรับอ่าน r-theta motion")
                return

            self.r_position = self.binary_reverse_twos_complement(self.register[0x11]) / 10
            self.theta_position = self.binary_reverse_twos_complement(self.register[0x12]) / 10
            self.v_r = self.register[0x13] / 10
            self.v_theta = self.register[0x14] / 10
            self.a_r = self.register[0x15] / 10
            self.a_theta = self.register[0x16] / 10
        except Exception as e:
            print("read_r_theta_actual_motion Error:", e)


    def write_up_down_order(self, up, down):
        if up == down:
            print("Invalid command: UP and DOWN cannot be the same")
            return
        print(f"Writing Up={up}, Down={down} to registers 0x04 and 0x05")
        try:
            self.client.write_register(address=0x04, value=up, slave=self.slave_address)
            self.client.write_register(address=0x05, value=down, slave=self.slave_address)
            print("Write successful")
        except Exception as e:
            print(f"Error writing to registers: {e}")

    def read_up_down_order(self):
        """แก้ไขให้ handle exception ได้ดีขึ้น"""
        try:
            response = self.client.read_holding_registers(
                address=0x03, 
                count=1, 
                slave=self.slave_address
            )
            
            if hasattr(response, 'isError') and response.isError():
                print(f"Read up/down error: {response}")
                return False, False
                
            if not hasattr(response, 'registers') or len(response.registers) == 0:
                print("Invalid up/down response")
                return False, False
                
            status = response.registers[0]
            limit_down = status & 0x01      # Bit 0
            limit_up = (status >> 1) & 0x01 # Bit 1
            return limit_up, limit_down
            
        except Exception as e:
            print(f"Read up/down exception: {e}")
            return False, False


    def read_r_theta_moving_status(self):
        """ Read motion of r and theta status"""
        self.r_theta_moving_status_before = self.r_theta_moving_status
        moving_status_binary = self.binary_crop(6, self.decimal_to_binary(self.register[0x10]))[::-1]

        if moving_status_binary[0] == "1":
            self.r_theta_moving_status = "Home"
        elif moving_status_binary[1] == "1":
            self.r_theta_moving_status = "Run Jog Mode"
        elif moving_status_binary[2] == "1":
            self.r_theta_moving_status = "Run Point Mode"
        elif moving_status_binary[3] == "1":
            self.r_theta_moving_status = "Go To Target"
        else:
            self.r_theta_moving_status = "Idle"

        if hasattr(self, 'app'):
            self.app.handle_ui_change() 


    def read_target_positions(self):
        """
        Read the target positions of the robot (Target Positions 1-10)
        Each target position will store (r, theta)
        """
        old_positions = getattr(self, 'target_positions', [(0, 0)] * 10)
        self.target_positions = [(0, 0)] * 10 

        positions_changed = False
        
        for i in range(10):  
            r_value = self.binary_reverse_twos_complement(self.register[0x20 + i * 2]) / 10  
            theta_value = self.binary_reverse_twos_complement(self.register[0x20 + i * 2 + 1]) / 10 
            self.target_positions[i] = (r_value, theta_value)
            
            if old_positions[i] != self.target_positions[i]:
                positions_changed = True

        if positions_changed:
            print("Target Positions Changed:")
            for i, (r_value, theta_value) in enumerate(self.target_positions):
                if old_positions[i] != (r_value, theta_value):
                    print(f"  Point {i + 1}: r = {r_value} mm, theta = {theta_value}Degree")

    
    def write_goal_point(self, r, theta):
        """
        Write goal position (r, theta) to the robot and trigger Go To Target command.
        """
        try:
            self.client.write_register(address=0x40, value=int(r * 10), slave=self.slave_address)
            self.client.write_register(address=0x41, value=int(theta * 10), slave=self.slave_address)
            
            self.client.write_register(address=0x01, value=0x08, slave=self.slave_address)
            print(f"Goal Point Sent: r={r} mm, theta={theta}Degree")
        except Exception as e:
            print(f"Goal Point Send Failed: {e}")