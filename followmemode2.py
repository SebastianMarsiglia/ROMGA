from picamera2 import Picamera2
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import time

# Inicializar y configurar la cámara
picam2 = Picamera2()
cam_config = picam2.create_preview_configuration(main={"size": (640,360), "format": "RGB888"})
picam2.configure(cam_config)
picam2.start()
time.sleep(2)  # Espera para estabilizar la cámara

# Cargar el modelo MoveNet Lightning en formato TFLite
model_path = "/home/sebasmarsiglia2024/movenet_lightning.tflite"
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Obtener información del modelo
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
# Se asume que el modelo espera entrada de forma (1, 192, 192, 3)
input_size = (input_details[0]['shape'][2], input_details[0]['shape'][1])  # (192, 192)
input_dtype = input_details[0]['dtype']

# Precalcular factores de escala para mapear coordenadas de keypoints a la imagen original (640x360)
scale_x = 640 / input_size[0]
scale_y = 360 / input_size[1]

# Inicializar variables para calcular FPS
last_time = time.time()
frame_count = 0

while True:
    # Capturar imagen de la cámara (RGB, 640x360)
    frame = picam2.capture_array()

    # Redimensionar la imagen a 192x192 para el modelo
    img_resized = cv2.resize(frame, input_size, interpolation=cv2.INTER_LINEAR)

    # (Opcional) Verifica que la imagen tenga 3 canales
    if img_resized.shape[-1] == 4:
        img_resized = img_resized[:, :, :3]

    # Preparar la imagen de entrada según el tipo de dato requerido
    if input_dtype == np.float32:
        # Normalizar a rango [0,1] y expandir dimensiones para batch
        input_data = np.expand_dims(img_resized.astype(np.float32) / 255.0, axis=0)
    else:
        input_data = np.expand_dims(img_resized.astype(input_dtype), axis=0)

    # Ejecutar la inferencia
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    # Se asume que el output es de forma (1, N, 3) con [y, x, conf] para cada keypoint
    keypoints = interpreter.get_tensor(output_details[0]['index'])[0][0]

    # Dibujar los keypoints en la imagen original (640x360)
    for kp in [5, 11]:  
        y, x, conf = keypoints[kp]
        if conf > 0.3:  # Solo se muestran los puntos con suficiente confianza
            # Escalar coordenadas: el modelo trabaja en 192x192 y la visualización es en 640x360
            x_int = int(x * input_size[0] * scale_x)
            y_int = int(y * input_size[1] * scale_y)
            cv2.circle(frame, (x_int, y_int), 5, (0, 255, 0), -1)

    # Mostrar la imagen procesada
    cv2.imshow("MoveNet Lightning Pose Estimation", frame)

    # Calcular y mostrar FPS cada 10 frames
    frame_count += 1
    if frame_count >= 10:
        current_time = time.time()
        fps = frame_count / (current_time - last_time)
        print(f"FPS: {fps:.2f}")
        frame_count = 0
        last_time = current_time

    # Salir al presionar la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()

