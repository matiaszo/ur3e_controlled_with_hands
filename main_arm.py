import cv2 
import mediapipe as mp
import time
import requests
import threading
import URBasic
from rtde_receive import RTDEReceiveInterface
import socket
import datetime

# Constants for hand tracking
SCREEN_WIDTH = 700
SCREEN_HEIGHT = 400

# Initialize MediaPipe Hands module
hands = mp.solutions.hands
Hands = hands.Hands(max_num_hands=1)
mpDraw = mp.solutions.drawing_utils

# Initialize robot
ROBOT_IP = '169.254.41.22'
ACCELERATION = 0.9              # Robot acceleration value
VELOCITY = 0.8                  # Robot speed value
initial_joint_positions = [-1.7075, -1.6654, -1.5655, -0.1151, 1.5962, -0.0105]


# Initialize robot with URBasic
print("Initializing robot...")

rtde_receive = RTDEReceiveInterface(ROBOT_IP)

robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((ROBOT_IP, 29999))

# printa mensagem de retorno primária de conexão socket com o robô
welcome_message = s.recv(1024).decode()
print('Welcome message:', welcome_message)

robot.reset_error()

print("Robot initialized!")
time.sleep(1)

robot.movej(q=initial_joint_positions, a=ACCELERATION, v=VELOCITY)

robot.init_realtime_control()
time.sleep(0.5)


vs = cv2.VideoCapture(0)
vs.set(cv2.CAP_PROP_FRAME_WIDTH, SCREEN_WIDTH)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, SCREEN_HEIGHT)

API_KEY = "AIzaSyDmJLy9G9E6UGmJTy-2dfXJfRB8Kz1Jqtw"
DATABASE_URL = "https://ur3e-dashboard-default-rtdb.firebaseio.com/"



def find_hands(image):
    frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = Hands.process(frame_rgb)
    hand_points = results.multi_hand_landmarks
    if hand_points:
        for points in hand_points:
            mpDraw.draw_landmarks(image, points, hands.HAND_CONNECTIONS)
    

    return hand_points, image

def get_hand_size(hand_landmarks, frame):
    h, w, _ = frame.shape

    hand_distance_size_x = w * abs(hand_landmarks[1].x - hand_landmarks[17].x)
    hand_distance_size_y = h * abs(hand_landmarks[1].y - hand_landmarks[17].y)
    hand_size = (hand_distance_size_x**2 + hand_distance_size_y**2) ** 0.5

    return hand_size

def get_rotate_wrist(hand_landmarks, frame, hand_size):
    h, w, _ = frame.shape

    sx = hand_landmarks[0].x 
    sy = hand_landmarks[0].y
    hx = hand_landmarks[13].x
    hy = hand_landmarks[13].y

    dx = hx - sx
    dy = hy - sy

    return ((dy ** 0 ) * dx) * 8



def move_robot(hand_landmarks, frame):
    if not hand_landmarks:
        return
    
    h, w, _ = frame.shape

    hand_center = {
        'x': int((((hand_landmarks[1].x + hand_landmarks[17].x) / 2 * w) + ((hand_landmarks[0].x + hand_landmarks[5].x) / 2 * w)) / 2),  
        'y': int((((hand_landmarks[1].y + hand_landmarks[17].y) / 2 * h) + ((hand_landmarks[0].y + hand_landmarks[5].y) / 2 * h)) / 2)
    }


    hand_size = get_hand_size(hand_landmarks, frame)

    rotate_wrist = get_rotate_wrist(hand_landmarks, frame, hand_size)

    displacement_x = (hand_center['x'] - (SCREEN_WIDTH/2)) / SCREEN_WIDTH
    displacement_y = (hand_center['y'] - (SCREEN_HEIGHT/2)) / SCREEN_HEIGHT

    scale = 2.5  

    x_pos = displacement_x * scale
    y_pos = displacement_y * scale * -1
    w_pos = rotate_wrist   * scale


    move_joints = initial_joint_positions[:]
    move_joints[0] += x_pos  
    if move_joints[1] + y_pos > -2.37:
        move_joints[1] += y_pos
    else:
        move_joints[1] = -2.37
        move_joints[3] = -0.9
    move_joints[5] += w_pos 

    if not isThreadRunning('Move Robot'):
        t_move_robot = threading.Thread(
            target=robot.movej,
            args=(move_joints, ACCELERATION, VELOCITY),
            name='Move Robot')
        t_move_robot.start()


def send_data():
    # Endpoint do Realtime Database
    url = f"{DATABASE_URL}/lastData.json?auth={API_KEY}"

    while True: 
        # capture all data needed from the robot using RTDERecieveInterface
        current_data = {
            "main_voltage": rtde_receive. getActualMainVoltage(),
            "robot_ac": rtde_receive.getActualRobotCurrent(),
            "robot_voltage": rtde_receive.getActualRobotVoltage(),
            "status": rtde_receive.getSafetyStatusBits(),               
            "tcp_pose": rtde_receive.getActualTCPPose(),
            "temperatures": rtde_receive.getJointTemperatures()
        }
        response = requests.put(url, json=current_data)


def send_graph():
    # Endpoint do Realtime Database
    url = f"{DATABASE_URL}/data.json?auth={API_KEY}"
    delay = 60
    stack_data = 6

    history = [0 for _ in range(stack_data)]


    while True: 
        # capture all data needed from the robot using RTDERecieveInterface
        current_data = {
            "main_voltage": rtde_receive. getActualMainVoltage(),
            "robot_ac": rtde_receive.getActualRobotCurrent(),
            "robot_voltage": rtde_receive.getActualRobotVoltage(),
            "status": rtde_receive.getSafetyStatusBits(),               
            "tcp_pose": rtde_receive.getActualTCPPose(),
            "temperatures": rtde_receive.getJointTemperatures()
        }
        

        for i in range(len(history)-1):
            history[i] = history[i+1]
        history[stack_data-1] = current_data
        
        hour = str(datetime.datetime.now().hour)
        minute = str(datetime.datetime.now().minute)
        seconds = str(datetime.datetime.now().second)
        timestamp = hour + ':' + minute + ':' + seconds
        
        data = {
            "data" : history,
            "timestamp" : timestamp
        }
        response = requests.put(url,json=data)
        print(data)
        time.sleep(delay)

        

def get_data():
        # Endpoint do Realtime Database
        url = f"{DATABASE_URL}/userAction.json?auth={API_KEY}"
        while True:
            response = requests.get(url).json()
            if response['action']:
                s.sendall('unlock protective stop\n'.encode())
                response = s.recv(1024)
                data = {
                    'action' : False
                }
                response = requests.put(url, json=data)


        #----------------------SHOW----------------------------
        # print(commands)




def isThreadRunning(name):
    for thread in threading.enumerate():
        if thread.name == name:
            # if thread.is_alive():
            #     print('IS RUNNING')
            # else:
            #     print('IS NOT RUNNING')
            return thread.is_alive()
    
    return False




#---------------------------------------MAIN--------------------------------------------
t_send_stats = threading.Thread(target=send_data, name='Send stats to database')
t_send_stats.start()

t_get_commands = threading.Thread(target=get_data, name='Get commands from database')
t_get_commands.start()

t_send_graph = threading.Thread(target=send_graph, name='Send data to graph')
t_send_graph.start()

while True:
    ret, frame = vs.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)
    hand_points, frame = find_hands(frame)

    results = Hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0].landmark
        move_robot(hand_landmarks, frame)

    cv2.imshow("Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


vs.release()
cv2.destroyAllWindows()
rtde_receive.disconnect()