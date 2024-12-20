import cv2 
import mediapipe as mp
import time
import requests
import threading
import URBasic
from rtde_receive import RTDEReceiveInterface
import socket
import datetime


SCREEN_WIDTH = 700      # tamanho do comprimento da guia
SCREEN_HEIGHT = 400     # tamanho da altura da guia


hands = mp.solutions.hands              # variavel que contem o modulo das funções que envolvem mão do mediapipe
Hands = hands.Hands(max_num_hands=1)    # declaração da mao, e o maximo de mão detectadas na captura de imagem
mpDraw = mp.solutions.drawing_utils     # Variavel que ira conter as funções de desenho na tela



ROBOT_IP = '169.254.41.22'      # IP setado na interface do robo, que deve ser declarado aqui para a conexão remota
ACCELERATION = 0.9              # Valor de aceleração do Robo
VELOCITY = 0.8                  # Valor da velocidade maxima do robo


initial_joint_positions = [-1.7075, -1.6654, -1.5655, -0.1151, 1.5962, -0.0105]     # Posição em radianos de cada junta do robo 


print("Initializing robot...")


rtde_receive = RTDEReceiveInterface(ROBOT_IP)   # Variavel que será responsavel por pegar informações do robo, conectando pelo ip


robotModel = URBasic.robotModel.RobotModel()    # Variavel que contem um objeto RobotModel, que define as informações basicas do robo
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)   # Variavel que cria um objeto UrScriptExt, que contem as funções que serão passadas ao robo por script 
# o host contém o IP do robo para se conectar e o robotModel contem o modelo de robo que será utilizado


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # Variavel que cria uma instancia do socket, define o tipo de ip(socket.AF_INET = ipv4), e o protocolo de comunicação(socket.SOCK_STREAM = TCP)
s.connect((ROBOT_IP, 29999))    # Variavel que conecta com o robo através da porta 29999


# printa mensagem de retorno primária de conexão socket com o robô
welcome_message = s.recv(1024).decode()
print('Welcome message:', welcome_message)

robot.reset_error()

print("Robot initialized!")
time.sleep(1)


robot.movej(q=initial_joint_positions, a=ACCELERATION, v=VELOCITY)  # Função responsavel por mover o robo, o q define a posição das juntas, o a define a aceleração e o v a velocidade

robot.init_realtime_control()  # Função responsavel por iniciar o controle do robo em tempo real
time.sleep(0.5)


vs = cv2.VideoCapture(0)    # Variavel que inicializa a classe VideoCapture, o 0 é referente a primeira camera do dispositivo

vs.set(cv2.CAP_PROP_FRAME_WIDTH, SCREEN_WIDTH)  # Função que seta o comprimento da guia
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, SCREEN_HEIGHT)    # Função que seta a altura da guia


# Variaveis de configuração para conexão com banco de dados Firebase
API_KEY = "AIzaSyDmJLy9G9E6UGmJTy-2dfXJfRB8Kz1Jqtw"
DATABASE_URL = "https://ur3e-dashboard-default-rtdb.firebaseio.com/"


# Função que ira processar a imagem e captar se existe uma mão, se existir ele vai retornar a imagem e um array de 0 a 20 relacionados aos pontos da mão
def find_hands(image):
    frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = Hands.process(frame_rgb)
    hand_points = results.multi_hand_landmarks
    if hand_points:
        for points in hand_points:
            mpDraw.draw_landmarks(image, points, hands.HAND_CONNECTIONS)

    return hand_points, image


# Função que recece os pontos da mao, e retorna um valor de acordo com o giro da sua mão
def get_rotate_wrist(hand_landmarks):

    sx = hand_landmarks[0].x 
    sy = hand_landmarks[0].y
    hx = hand_landmarks[13].x
    hy = hand_landmarks[13].y

    dx = hx - sx
    dy = hy - sy

    return ((dy ** 0 ) * dx) * 8


# Função que será responsavel por receber a imagem e os pontos da mão, e movimentar o robo de acordo com isso
def move_robot(hand_landmarks, frame):
    # Verifica se a mão esta na imagem
    if not hand_landmarks:
        return
    
    # Cada variavel contem uma informação da imagem, h é para HEIGHT, w é WIDTH, e o _ seria para as cores que foi ignorado
    h, w, _ = frame.shape

    # Define o ponto central da mão
    hand_center = {
        'x': int((((hand_landmarks[1].x + hand_landmarks[17].x) / 2 * w) + ((hand_landmarks[0].x + hand_landmarks[5].x) / 2 * w)) / 2),  
        'y': int((((hand_landmarks[1].y + hand_landmarks[17].y) / 2 * h) + ((hand_landmarks[0].y + hand_landmarks[5].y) / 2 * h)) / 2)
    }

    # Variavel que irá conter o valor de rotação da mão
    rotate_wrist = get_rotate_wrist(hand_landmarks)

    # Variaveis que contem o afastamento da mão em relação ao centro da tela
    displacement_x = (hand_center['x'] - (SCREEN_WIDTH/2)) / SCREEN_WIDTH
    displacement_y = (hand_center['y'] - (SCREEN_HEIGHT/2)) / SCREEN_HEIGHT

    # Escala de conversão para a movimentação adequada do robo
    scale = 2.5  

    # Variaveis que definem posição x,y e de giro da mão convertidas para o robo
    x_pos = displacement_x * scale
    y_pos = displacement_y * scale * -1
    w_pos = rotate_wrist   * scale

    # Variavel que será responsavel por atualizar os valores da juntas e mover o robo
    move_joints = initial_joint_positions[:]
    move_joints[0] += x_pos  

    # Verificação para descer mais a garra e alinhar totalmente para baixo
    if move_joints[1] + y_pos > -2.37:
        move_joints[1] += y_pos
    else:
        move_joints[1] = -2.37
        move_joints[3] = -0.9
    move_joints[5] += w_pos 


    # Caso a thread que executa essa função nao estiver funcionando, ele coloca ela para funcionar de novo
    if not isThreadRunning('Move Robot'):
        t_move_robot = threading.Thread(
            target=robot.movej,
            args=(move_joints, ACCELERATION, VELOCITY),
            name='Move Robot')
        t_move_robot.start()

# Função que enviará os dados do robo para o banco de dados, que posteriormente 
def send_data():
    # Endpoint do Realtime Database - Firebase
    url = f"{DATABASE_URL}/lastData.json?auth={API_KEY}"


    while True: 
        # Captura todos os dados necessarios do robo usando RTDERecieveInterface
        current_data = {
            "main_voltage": rtde_receive. getActualMainVoltage(),
            "robot_ac": rtde_receive.getActualRobotCurrent(),
            "robot_voltage": rtde_receive.getActualRobotVoltage(),
            "status": rtde_receive.getSafetyStatusBits(),               
            "tcp_pose": rtde_receive.getActualTCPPose(),
            "temperatures": rtde_receive.getJointTemperatures()
        }
        # Variavel responsavel por atualizar os dados no banco
        response = requests.put(url, json=current_data)

# Função que enviara as mesmas informações para o banco, e serão usadas em outra parte do front
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

        
# Função que recebe a situação do robo, e se ele tiver em parada de proteção, é mandado para o robo o desbloqueio da trava
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



# Função que verifica se a thread esta rodando
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

# Inicialização das threads com as funções previamente declaradas
t_send_stats = threading.Thread(target=send_data, name='Send stats to database')
t_send_stats.start()

t_get_commands = threading.Thread(target=get_data, name='Get commands from database')
t_get_commands.start()

t_send_graph = threading.Thread(target=send_graph, name='Send data to graph')
t_send_graph.start()

#Loop Principal
while True:
    # Variaveis de leitura da imagem capturada
    ret, frame = vs.read()
    if not ret:
        break

    # variavel que flipa a imagem mostrada ao usuario
    frame = cv2.flip(frame, 1)

    # Variaveis que contem informações da mão e imagem
    hand_points, frame = find_hands(frame)

    # Variavel que processa a imagem da mão
    results = Hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Verificação que move o robo de acordo com a posição do pulso, caso ela esteja na imagem
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0].landmark
        move_robot(hand_landmarks, frame)

    # Mostra a imagem para o Usuario        
    cv2.imshow("Hand Tracking", frame)

    # Fecha a imagem se apertado a tecla q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Fecha todos os processos de mão, imagem e conexão
vs.release()
cv2.destroyAllWindows()
rtde_receive.disconnect()