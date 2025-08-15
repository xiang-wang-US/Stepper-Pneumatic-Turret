import cv2
import numpy as np
import urllib.request
import mediapipe as MediaPipe
import socket
import time

ESP32_IP = ""
ESP32_PORT = 1234

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5) 

url = f'http://{ESP32_IP}/stream'

# Initialize MediaPipe Pose
mp_pose = MediaPipe.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# (Landmark index constants remain the same)
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_HIP, RIGHT_HIP = 23, 24

def main():
    cap = cv2.VideoCapture(0) # url if streaming

    # How often we send data 
    last_send_time = 0
    send_interval = 0.1

    if not cap.isOpened():
        print("epic webcam fail")
        return

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("could not get frame")
            continue

        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            h, w, _ = frame.shape

            # (Landmark coordinate and torso center calculations remain the same)
            l_shoulder = int(landmarks[LEFT_SHOULDER].x * w), int(landmarks[LEFT_SHOULDER].y * h)
            r_shoulder = int(landmarks[RIGHT_SHOULDER].x * w), int(landmarks[RIGHT_SHOULDER].y * h)
            l_hip = int(landmarks[LEFT_HIP].x * w), int(landmarks[LEFT_HIP].y * h)
            r_hip = int(landmarks[RIGHT_HIP].x * w), int(landmarks[RIGHT_HIP].y * h)
            torso_center_x = (l_shoulder[0] + r_shoulder[0] + l_hip[0] + r_hip[0]) // 4
            torso_center_y = (l_shoulder[1] + r_shoulder[1] + l_hip[1] + r_hip[1]) // 4
            torso_center = (torso_center_x, torso_center_y)

            screen_center = (w // 2, h // 2)
            cv2.arrowedLine(frame, screen_center, torso_center, (0, 255, 255), 3)

            raw_vector_x = (torso_center_x - screen_center[0]) / (w / 2)
            raw_vector_y = -1 * (torso_center_y - screen_center[1]) / (h / 2)
            
            vector_text = f"Vector: ({raw_vector_x:.2f}, {raw_vector_y:.2f})"
            cv2.putText(frame, vector_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            # PACKET STUFF HERE
            current_time = time.time()
            if current_time - last_send_time > send_interval:

                message = f"{raw_vector_x:.4f},{raw_vector_y:.4f}"
                
                sock.sendto(message.encode('utf-8'), (ESP32_IP, ESP32_PORT))
                last_send_time = current_time
                
                try:
                    data, server = sock.recvfrom(1024)
                except socket.timeout:
                    print("No reply from ESP32")



        cv2.imshow('Live Landmark Tracking Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    pose.close()

if __name__ == "__main__":
    main()
