from pymycobot import MyCobot280
from utils import bcolors
serial_port = '/dev/ttyACM0'  # Adjust this to your actual serial port
try:  
    mc = MyCobot280(serial_port)
    print(f"{bcolors.OKGREEN}Connection successful{bcolors.ENDC}")
except Exception as e:
    print(f"{bcolors.FAIL}Failed to connect to MyCobot: {e}{bcolors.ENDC}")