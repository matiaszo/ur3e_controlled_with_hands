import socket 
import time

HOST = "169.254.58.132"
PORT = 30002

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


# s.send(("movej ([0, 0, 0, \
# 0, 0, 0],\
# a=1.3962634015954636, v=1.0471975511965976)" + "\n").encode('utf8'))

s.send(("set_digital_out(0,True)" + "\n").encode('utf8'))
# s.send(("set_digital_out(0,True)" + "\n").encode('utf8'))
print("Mandou o true")

time.sleep(3)
s.send(("set_digital_out(0,False)" + "\n").encode('utf8'))
print("Mandou o false")

s.send(("movej ([0.540537125683036, -2.0, -1.0986348160112505, \
-2.6437150406384227, 3.352864759694935, -1.2294883935868013],\
a=1.3962634015954636, v=1.0471975511965976)" + "\n").encode('utf8'))


s.close()

# import urx
# import time

# def connect_to_ur3e(ip_address):
#     try:
        # Conecta ao cobot usando o endereço IP
        # rob = urx.Robot(ip_address)
        
        # Verifica o estado de conexão
        # if rob.is_program_running():
        #     print("Conexão estabelecida com o UR3e")
        # else:
        #     print("UR3e conectado, mas não está executando um programa.")
        
        # Espera alguns segundos para garantir a estabilidade da conexão
        # time.sleep(1)
        
        # Desconectar após o teste
#         rob.close()
#         print("Conexão fechada com sucesso.")
#     except Exception as e:
#         print(f"Erro na conexao ao UR3e: {e}")

# connect_to_ur3e("169.254.58.132")