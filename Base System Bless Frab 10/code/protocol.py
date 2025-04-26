import platform
import struct
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.client import ModbusTcpClient

# ----------------------------------- Config this variable before using ----------------------------------- 
device_port = "COM5"
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


        self.client = ModbusClient(method="rtu", port=self.port, stopbits=1, bytesize=8, parity="E", baudrate=19200)
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


    def routine(self):
        try:
            self.register = self.client.read_holding_registers(address=0x00, count=0x46, slave=self.slave_address).registers
            self.read_r_theta_moving_status()
            self.read_r_theta_actual_motion()
            self.read_target_positions()

            limit_switch = self.read_limit_switch_status()
            if limit_switch["limit_up"]:
                servo_status = "UP"
            elif limit_switch["limit_down"]:
                servo_status = "DOWN"
            else:
                servo_status = "MOVING"

            print("Servo Status:", servo_status)
            print("Position: r =", self.r_position, "theta =", self.theta_position)
            print("Speed: v_r =", self.v_r, "v_theta =", self.v_theta)
            print("Acceleration: a_r =", self.a_r, "a_theta =", self.a_theta)
            print("Moving Status:", self.r_theta_moving_status)
            self.routine_normal = True
        except Exception as e:
            print("Routine Error", e)
            self.routine_normal = False


    def read_limit_switch_status(self):
        """
        Read the status of the limit switches from register 0x03.
        """
        try:
            status = self.client.read_holding_registers(address=0x03, count=1, slave=self.slave_address).registers[0]
            
            limit_down = (status & 0x01) > 0
            limit_up = (status & 0x02) > 0
            
            return {
                "limit_up": limit_up,
                "limit_down": limit_down
            }
        except Exception as e:
            print(f"Error reading limit switch status: {e}")
            return {
                "limit_up": False,
                "limit_down": False
            }
    
    def read_heartbeat(self):
        """
        Read heartbeat value from robot; expects 22881 for "Ya"
        """
        try:
            heartbeat_value = self.client.read_holding_registers(address=0x00, count=1, slave=self.slave_address).registers
            return heartbeat_value[0]
        except Exception as e:
            print("Heartbeat Error", e)
            return "Error"

    
    def write_heartbeat(self):
        """
        Send heartbeat signal to robot (writes 18537 for "Hi")
        """
        try:
            self.client.write_register(address=0x00, value=18537, slave=self.slave_address)
            self.usb_connect = True
        except:
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
        try:
            status = self.client.read_holding_registers(address=0x03, count=1, slave=self.slave_address).registers[0]
            
            up = status & 0x01
            down = (status >> 1) & 0x01
            
            return up, down
        except Exception as e:
            print(f"Error reading up/down order from register: {e}")
            return None, None



    def read_r_theta_actual_motion(self):
        """
        Read the actual motion values from the robot.
        """
        self.r_position = self.binary_reverse_twos_complement(self.register[0x11]) / 10
        self.theta_position = self.binary_reverse_twos_complement(self.register[0x12]) / 10
        self.v_r = self.register[0x13] / 10
        self.v_theta = self.register[0x14] / 10
        self.a_r = self.register[0x15] / 10
        self.a_theta = self.register[0x16] / 10

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
        """
        self.target_positions = [0] * 10 

        for i in range(10):
            self.target_positions[i] = self.binary_reverse_twos_complement(self.register[0x20 + i]) / 10

        print("Target Positions Read:", self.target_positions)
    
    def write_goal_point(self, r, theta):
        """
        Write goal position (r, theta) to the robot and trigger Go To Target command.
        """
        try:
            self.client.write_register(address=0x30, value=int(r * 10), slave=self.slave_address)
            self.client.write_register(address=0x31, value=int(theta * 10), slave=self.slave_address)
            
            self.client.write_register(address=0x01, value=0x08, slave=self.slave_address)
            
            print(f"Goal Point Sent: r={r} mm, theta={theta}°")
        except Exception as e:
            print(f"Goal Point Send Failed: {e}")


