import cv2
import numpy as np
import urllib.request

url = 'http://<live-feed-ip>/stream' # Replace with whatever you get
import cv2
import mediapipe as MediaPipe
import numpy as np 

# Initialize MediaPipe Pose
mp_pose = MediaPipe.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = MediaPipe.solutions.drawing_utils

# Index constants for landmarks
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_HIP = 23
RIGHT_HIP = 24
NOSE = 0 

def main():
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("epic webcam fail")
        return

    while cap.isOpened():
        success, frame = cap.read()

        if not success:
            print("could not get frame")
            continue

        # process RGB frame with MediaPipe Pose
        results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark 
            h, w, _ = frame.shape

            # draw torso box
            l_shoulder = int(landmarks[LEFT_SHOULDER].x * w), int(landmarks[LEFT_SHOULDER].y * h)
            r_shoulder = int(landmarks[RIGHT_SHOULDER].x * w), int(landmarks[RIGHT_SHOULDER].y * h)
            l_hip = int(landmarks[LEFT_HIP].x * w), int(landmarks[LEFT_HIP].y * h)
            r_hip = int(landmarks[RIGHT_HIP].x * w), int(landmarks[RIGHT_HIP].y * h)

            cv2.line(frame, l_shoulder, r_shoulder, (0, 255, 0), 2)
            cv2.line(frame, l_hip, r_hip, (0, 255, 0), 2)
            cv2.line(frame, l_shoulder, l_hip, (0, 255, 0), 2)
            cv2.line(frame, r_shoulder, r_hip, (0, 255, 0), 2)

            # draw torso center
            torso_center_x = (abs((l_shoulder[0] + l_hip[0]) // 2) + abs((r_shoulder[0] + r_hip[0]) // 2)) // 2
            torso_center_y = (abs((l_shoulder[1] + l_hip[1]) // 2) + abs((r_shoulder[1] + r_hip[1]) // 2)) // 2
            torso_center = (torso_center_x, torso_center_y)
            cv2.circle(frame, torso_center, 5, (255, 0, 0), -1)

            # draw head circle  
            nose = int(landmarks[NOSE].x * w), int(landmarks[NOSE].y * h)
            head_radius = int(np.linalg.norm(np.array(l_shoulder) - np.array(r_shoulder)) * 0.5)
            cv2.circle(frame, nose, head_radius, (0, 0, 255), 2)
            cv2.circle(frame, nose, 5, (0, 0, 255), -1)
            
        cv2.imshow('Live Landmark Tracking Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    pose.close()

if __name__ == "__main__":
    main() 
