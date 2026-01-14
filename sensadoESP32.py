# ESP32-S3 conectado a puerto USB "/dev/ttyACM0", establecer comunicación bidireccional
# Usb c derecho en esp32s3 a usb a en raspberry pi 4. Conectar y esperar 10 segundos.
import serial
import json
import time

try:
    # Inicializar conexión serial
    ESP32 = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
    time.sleep(1)
    print("✅ ESP32 conectado")
except serial.SerialException:
    print("❌ Error al conectar ESP32")
    exit()

# Límites para alarmas
PESO_MAX = 150000        # gramos
ANGULO_MAX = 10.0        # grados
DISTANCIA_MIN = 40.0     # cm
TEMP_MAX = 50.0          # °C
TEMP_MIN = 10.0          # °C

while True:
    try:
        if ESP32.in_waiting > 0:
            # Leer línea enviada por ESP32
            line = ESP32.readline().decode('utf-8').rstrip()
            if line:
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    print("⚠️ Error al decodificar JSON:", line)
                    continue

                # Asignar valores
                p1 = data["p1"]
                d1 = data["d1"]
                d2 = data["d2"]
                t1 = data["t1"]
                aX = data["aX"]
                aY = data["aY"]
                aZ = data["aZ"]

                # Imprimir datos
                print("Peso:", p1 / 1000, "kg")
                print("Distancia 1(DD):", d1, "cm")
                print("Distancia 2(DI):", d2, "cm")
                print("Temp Bat:", t1, "°C")
                print("Ángulo X(F):", aX, "°")
                print("Ángulo Y(L):", aY, "°")
                
                
                #print("Ángulo Z:", aZ, "°")

                # === Alarmas ===
                if p1 >= PESO_MAX:
                   print("🚨 ¡Peso máximo excedido!")
                if t1 >= TEMP_MAX or t1 <= TEMP_MIN:
                    print("🌡️ ¡Temperatura de batería fuera de rango!")
                if abs(aY) > ANGULO_MAX:
                    print("⚠️ ¡Inclinación lateral peligrosa!")
                if abs(aX) > ANGULO_MAX:
                    print("⚠️ ¡Inclinación frontal peligrosa!")
                if d1 < DISTANCIA_MIN or d2 < DISTANCIA_MIN:
                    print("🚧 ¡Obstáculo detectado!")

                print("-" * 40)

                # Enviar respuesta al ESP32
                ESP32.write(b"Recibido\n")

    except KeyboardInterrupt:
        print("\n🔌 Programa detenido por el usuario.")
        break
