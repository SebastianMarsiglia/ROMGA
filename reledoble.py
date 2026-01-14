import pigpio
import time
import cv2
import subprocess
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

def print_status(front_on, back_on):
    estado_f = "ON" if front_on else "OFF"
    estado_b = "ON" if back_on else "OFF"
    print(f"Estado → Luz delantera: {estado_f}, Luz trasera: {estado_b}\n")

def main():
    if subprocess.run(["pgrep", "pigpiod"]).returncode != 0: subprocess.run(["sudo", "pigpiod"]); time.sleep(2)
    pi = pigpio.pi()
    pin = 7 #IN1 - Rele 1 - Luz delantera
    pin2 = 8 #IN2 - Rele 2 - Luz trasera
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_mode(pin2, pigpio.OUTPUT)
    pi.write(pin, 1) 
    pi.write(pin2, 1)
    cv2.namedWindow("Control Rele")
    
    front_light_on = False
    back_light_on = False
    
    print("[r] Encender luz delantera")
    print("[e] Apagar luz delantera")
    print("[y] Encender luz trasera")
    print("[t] Apagar luz trasera")
    print("[q] Salir")

    while True:
        key = cv2.waitKey(1) & 0xFF  
        if key == ord('r'):
            pi.write(pin, 0) #Encender luz delantera
            front_light_on = True
            print("→ Encender luz delantera")
            print_status(front_light_on, back_light_on)
        elif key == ord('e'):
            pi.write(pin, 1) #Apagar luz delantera
            front_light_on = False
            print("→ Apagar luz delantera")
            print_status(front_light_on, back_light_on)
        elif key == ord('y'):
            pi.write(pin2, 0) #Encender luz trasera
            back_light_on = True
            print("→ Encender luz trasera")
            print_status(front_light_on, back_light_on)
        elif key == ord('t'):
            pi.write(pin2, 1) #Apagar luz trasera  
            back_light_on = False
            print("→ Apagar luz trasera")
            print_status(front_light_on, back_light_on)
        elif key == ord('q'):
            print("→ Saliendo del sistema...")
            break  
    
    pi.write(pin, 1)
    pi.write(pin2, 1)
    pi.stop()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
