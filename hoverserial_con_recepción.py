import serial
import struct
import time

# Definir constantes
HOVER_SERIAL_BAUD = 115200
START_FRAME = 0xABCD
TIME_SEND = 0.05  # En segundos
SPEED_MAX_TEST = 500
SPEED_STEP = 20

# Configurar la comunicación serial (cambiar el puerto según sea necesario)
hover_serial = serial.Serial('/dev/serial0', HOVER_SERIAL_BAUD, timeout=1)

# Estructura para los comandos
def create_command(steer, speed):
    start = START_FRAME
    checksum = (start ^ steer ^ speed) & 0xFFFF
    # Usamos 'H' para unsigned short y 'h' para signed short
    return struct.pack('<HhhH', start, steer, speed, checksum)

# Función para enviar el comando
def send_command(steer, speed):
    command = create_command(steer, speed)
    hover_serial.write(command)

# Estructura para los datos de feedback
def parse_feedback(data):
    # El formato sigue el mismo patrón: <HhhhhhhHH
    feedback = struct.unpack('<HhhhhhhHH', data)
    start, cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed, checksum = feedback
    calc_checksum = (start ^ cmd1 ^ cmd2 ^ speedR_meas ^ speedL_meas ^ batVoltage ^ boardTemp ^ cmdLed) & 0xFFFF

    # Verificar validez de los datos
    if start == START_FRAME and calc_checksum == checksum:
        print(f"Received: start={start}, cmd1={cmd1}, cmd2={cmd2}, speedR_meas={speedR_meas}, "
              f"speedL_meas={speedL_meas}, batVoltage={batVoltage}, boardTemp={boardTemp}, cmdLed={cmdLed}, "
              f"received_checksum={checksum}, calculated_checksum={calc_checksum}")
    else:
        print("Non-valid data skipped")
    

# Función para recibir datos
def receive_feedback():
    if hover_serial.in_waiting >= 18:  # 18 bytes para toda la estructura
        data = hover_serial.read(18)
        parse_feedback(data)

# Bucle principal
def main():
    last_send_time = time.time()  # Marca el tiempo inicial de envío.

    while True:
        # Recibir datos
        receive_feedback()

        # Enviar comandos periódicamente
        if time.time() - last_send_time >= TIME_SEND:
            send_command(0,60)
            last_send_time = time.time()

        time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        hover_serial.close()
        print("Program terminated.")
