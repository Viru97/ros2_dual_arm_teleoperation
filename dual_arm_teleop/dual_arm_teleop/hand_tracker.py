import cv2
import mediapipe as mp

class HandTracker:
    def __init__(self, max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.7):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_draw = mp.solutions.drawing_utils

    def process_frame(self, frame):
        """Processes an image frame and returns hand landmarks."""
        # MediaPipe requires RGB images, OpenCV uses BGR
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        
        # Draw the landmarks on the original frame
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS
                )
        return frame, results

# --- Standalone Test ---
if __name__ == "__main__":
    tracker = HandTracker()
    cap = cv2.VideoCapture(0) # 0 is usually the default laptop webcam

    print("Opening webcam... Press 'q' to quit.")
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip the image horizontally for a selfie-view display
        frame = cv2.flip(frame, 1)
        
        # Track hands and draw
        annotated_frame, _ = tracker.process_frame(frame)

        cv2.imshow('MediaPipe Hand Tracker', annotated_frame)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()