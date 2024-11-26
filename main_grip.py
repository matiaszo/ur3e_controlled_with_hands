import cv2
import mediapipe as mp
import time
import threading
from librarys.gripper import Gripper

# Constants for hand tracking
SCREEN_WIDTH = 700
SCREEN_HEIGHT = 400

# Initialize MediaPipe Hands module
hands = mp.solutions.hands
Hands = hands.Hands(max_num_hands=1)
mpDraw = mp.solutions.drawing_utils


# Initialize gripepr
IP = '169.254.108.43'
gripper = Gripper(IP)

vs = cv2.VideoCapture(0) 
vs.set(cv2.CAP_PROP_FRAME_WIDTH, SCREEN_WIDTH)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, SCREEN_HEIGHT)


def find_hands(image):
    frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = Hands.process(frame_rgb)
    hand_points = results.multi_hand_landmarks
    if hand_points:
        for points in hand_points:
            mpDraw.draw_landmarks(image, points, hands.HAND_CONNECTIONS)
    

    return hand_points, image

def get_hand_grip(hand_landmarks, frame, hand_size):
    h, w, _ = frame.shape

    hand_grip = 0
    for i in range(8,21,4):
        if i < 15:
            point = 1
        else:
            point = 0
        
        crr_center_x = int((w * (hand_landmarks[i-2].x + hand_landmarks[point].x)) / 2)
        crr_center_y = int((h * (hand_landmarks[i-2].y + hand_landmarks[point].y)) / 2)

        finger_distance_x = abs(crr_center_x - (w * hand_landmarks[i].x))
        finger_distance_y = abs(crr_center_y - (h * hand_landmarks[i].y))

        finger_distance = (finger_distance_x*finger_distance_x + finger_distance_y*finger_distance_y) ** 0.5
        hand_grip += finger_distance

    hand_grip /= 4

    hand_grip = (int((1 - hand_grip / (hand_size * 1) ) * 100))
    hand_grip = int(int(hand_grip / 20 + 1) * 20)

    if(hand_grip <= 3):
        hand_grip = 4
    elif(20>=hand_grip>60):
        hand_grip = 40
    elif(hand_grip >= 60):
        hand_grip = 90

    
    return hand_grip

def get_hand_size(hand_landmarks, frame):
    h, w, _ = frame.shape

    hand_distance_size_x = w * abs(hand_landmarks[1].x - hand_landmarks[17].x)
    hand_distance_size_y = h * abs(hand_landmarks[1].y - hand_landmarks[17].y)
    hand_size = (hand_distance_size_x**2 + hand_distance_size_y**2) ** 0.5

    return hand_size


def move_grip(hand_landmarks, frame):
    if not hand_landmarks:
        return
    
    h, w, _ = frame.shape


    hand_size = get_hand_size(hand_landmarks, frame)

    hand_grip = get_hand_grip(hand_landmarks, frame, hand_size)
    if not isThreadRunning('Move Grip'):
        t_move_grip = threading.Thread(target=gripper.set_position, args=(hand_grip,), name='Move Grip')
        t_move_grip.start()
        print(hand_grip)


def isThreadRunning(name):
    for thread in threading.enumerate():
        if thread.name == name:
            if thread.is_alive():
                print('IS RUNNING')
            else:
                print('IS NOT RUNNING')
            return thread.is_alive()
    
    return False


#---------------------------------------MAIN--------------------------------------------
while True:
    ret, frame = vs.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)
    hand_points, frame = find_hands(frame)

    results = Hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0].landmark
        move_grip(hand_landmarks, frame)

    cv2.imshow("Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


vs.release()
cv2.destroyAllWindows()