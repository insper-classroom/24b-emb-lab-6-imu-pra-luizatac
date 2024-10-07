import serial
import uinput
import time

# Configura a porta serial (ajuste conforme necessário)
ser = serial.Serial('/dev/ttyACM0', 115200)

# Cria um novo dispositivo de mouse
device = uinput.Device([
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT,
    uinput.REL_X,
    uinput.REL_Y,
])

# Sensibilidade ajustada para suavizar o movimento do mouse
SENSITIVITY = 0.006
# Limite para detectar um "movimento brusco" no eixo Y (para a frente)
BRUSQUE_MOVEMENT_THRESHOLD = 5000  # Ajuste esse valor conforme necessário
# Tempo mínimo entre cliques (para evitar cliques repetidos)
MIN_TIME_BETWEEN_CLICKS = 0.5
last_click_time = time.time()

def parse_mpu6050_data(data_line):
    """Interpreta os dados recebidos do MPU6050"""
    try:
        # Exemplo de dados: "Acc. X = 123, Y = 456, Z = 789\n"
        parts = data_line.split(',')
        accel_x = int(parts[0].split('=')[1].strip())
        accel_y = int(parts[1].split('=')[1].strip())
        accel_z = int(parts[2].split('=')[1].strip())

        return accel_x, accel_y, accel_z
    except Exception as e:
        print(f"Erro ao interpretar dados: {e}")
        return None, None, None

def move_mouse_based_on_acceleration(accel_x, accel_y):
    """Move o mouse com base nos valores de aceleração X e Y"""
    device.emit(uinput.REL_X, int(accel_x * SENSITIVITY))
    device.emit(uinput.REL_Y, int(accel_y * SENSITIVITY))

def click_right_button():
    """Função para clicar com o botão direito do mouse"""
    device.emit(uinput.BTN_LEFT, 1)  # Pressiona o botão direito
    time.sleep(0.1)                    # Tempo para simular a pressão
    device.emit(uinput.BTN_LEFT, 0)  # Libera o botão direito

def detect_brusque_movement_and_click(accel_y):
    """Detecta um movimento brusco para frente e executa um clique do mouse"""
    global last_click_time
    current_time = time.time()

    # Detecta um movimento brusco para frente
    if accel_y > BRUSQUE_MOVEMENT_THRESHOLD and (current_time - last_click_time > MIN_TIME_BETWEEN_CLICKS):
        click_right_button()  # Chama a função para clicar com o botão direito
        print(f"Clique direito detectado! Aceleração Y: {accel_y}")
        last_click_time = current_time

try:
    # Loop principal para ler e processar os dados
    while True:
        # Aguarda uma linha completa de dados (do printf no C)
        data_line = ser.readline().decode('utf-8').strip()
        print(f"Dados recebidos: {data_line}")
        
        if "Acc." in data_line:
            accel_x, accel_y, accel_z = parse_mpu6050_data(data_line)
            if accel_x is not None and accel_y is not None:
                # Mover o mouse com base na aceleração
                move_mouse_based_on_acceleration(accel_x, accel_y)
                
                # Detectar movimento brusco e executar clique
                detect_brusque_movement_and_click(accel_y)

except KeyboardInterrupt:
    print("Programa encerrado pelo usuário.")
except Exception as e:
    print(f"Ocorreu um erro: {e}")
finally:
    ser.close()
