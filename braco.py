import socket 
import time
import math

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

PI = math.pi

HOST = "169.254.101.232"
PORT = 30002

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

rtde_receive = RTDEReceiveInterface(HOST)
rtde_control = RTDEControlInterface(HOST)

base = 90
ombro = -90
cotovelo = 90
p1 = -270
p2 =90
p3 = 90

base = str(((base * PI)/180))
ombro = str(((ombro * PI)/180))
cotovelo = str(((cotovelo * PI)/180))
p1 = str(((p1 * PI)/180))
p2 = str(((p2 * PI)/180))
p3 = str((p3 * PI)/180)

acceleration = "5.0"
velocity = "0.5"

positions = "[" + base + "," + ombro + "," + cotovelo + "," + p1 + "," + p2 + "," + p3 + "]"

move_text = "movej(" + positions + "," + "a="  + acceleration + "," + "v=" + velocity + ")" 

print(move_text)

time.sleep(1)

s.send((move_text + "\n").encode('utf8'))

robot_mode = rtde_receive.getRobotMode()
print(robot_mode)

# time.sleep(0.5)  # Check every 0.5 seconds



# temperature = rtde_receive.getJointTemperatures()
# pose = rtde_receive.getTargetTCPPose()
# success = rtde_control.recoverFromProtectiveStop()
        
# print("\n Temp: ", temperature)
# print("\n Pose: ", pose)

##base, ombro, cotovelo, pulso1, pulso2, pulso3
# s.send(("movej ([0.540537125683036, -2.0, -1.0986348160112505, \
# -2.6437150406384227, 3.352864759694935, -1.2294883935868013],\
# a=1.3962634015954636, v=1.0471975511965976)" + "\n").encode('utf8'))

s.close()
