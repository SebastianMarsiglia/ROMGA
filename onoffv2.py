import RPi.GPIO as GPIO
import time

# Configuración del modo y el pin GPIO
GPIO.setmode(GPIO.BCM)
PIN = 23  # Con resistencia de 1kohm conectada al cable negro del boton de encendido.
duracion = 0.5
GPIO.setup(PIN, GPIO.OUT)

# Asegura que el pin comience en LOW
GPIO.output(PIN, GPIO.LOW)

# Función para encender el GPIO por un instante
def pulso_gpio(pin, duracion):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(duracion)
    GPIO.output(pin, GPIO.LOW)
    
try:
    # Pulso inicial al iniciar el programa
    print("Encendiendo GPIO al inicio...")
    pulso_gpio(PIN, duracion)
    
    while True:
        print("\nMenú de control:")
        print("1. Ejecutar pulso GPIO")
        print("2. Salir")
        opcion = input("Selecciona una opción (1/2): ")

        if opcion == "1":
            print("Ejecutando pulso...")
            pulso_gpio(PIN, duracion)
        elif opcion == "2":
            print("Saliendo del programa...")
            break
        else:
            print("Opción no válida. Intenta de nuevo.")

except KeyboardInterrupt:
    print("\nInterrupción por teclado. Saliendo...")

finally:
    # Pulso final antes de salir
    print("Encendiendo GPIO al finalizar...")
    pulso_gpio(PIN, duracion)

    # Limpieza de los GPIO
    GPIO.cleanup()
    print("GPIO limpio. Programa finalizado.")
