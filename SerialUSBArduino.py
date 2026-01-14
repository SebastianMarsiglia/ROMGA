# Arduino Mega conectado a puerto USB "/dev/ttyUSB0", establecer comunicación bidireccional
import serial
import time

try:
    # Inicializar conexión serial
    arduinomega = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
    time.sleep(1)
    print("Arduino conectado")
except serial.SerialException:
    print("Error al conectar Arduino")
    exit()

while True:
    if arduinomega.in_waiting > 0:
        try:
            # Leer línea enviada por Arduino
            line = arduinomega.readline().decode('utf-8').rstrip()
            if line:  # Verifica que no esté vacía
                # Separar distancia y peso
                distance, weight = map(int, line.split(","))
                weight = weight/1000  #Gramos a kilogramos
                print(f"Distancia: {distance} cm, Peso: {weight:.1f} kg")

                # Validar valores dentro de rango esperado
                if distance < 40:
                    print("Advertencia: Obstaculo detectado")
                if weight > 200:
                    print("¡ALERTA! Peso máximo excedido")

            # Enviar respuesta al Arduino
            arduinomega.write(b"Recibido\n")
        except Exception as e:
            print(f"Error: {e}")
