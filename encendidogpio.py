import pigpio
import os
import time
import cv2

def main():
    if os.system("pgrep pigpiod") != 0: os.system("sudo pigpiod"); time.sleep(2)
    pi = pigpio.pi()
    pin = 18
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0) 
    cv2.namedWindow("Control GPIO")

    while True:
        key = cv2.waitKey(1) & 0xFF  
        if key == ord('l'):
            pi.write(pin, 1)  
            time.sleep(0.5)
            pi.write(pin, 0)
        elif key == ord('q'):
            break  
            
    pi.stop()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
