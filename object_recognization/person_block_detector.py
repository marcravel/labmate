
import cv2
import numpy as np
from ultralytics import YOLO


class PersonBlockDetector:

    def __init__(self, model_path="yolov8n.pt"):

        # Load pretrained YOLO model
        self.model = YOLO(model_path)

        # Path obstruction settings
        self.blocked_frames = 0
        self.clear_frames = 0

        self.block_threshold = 5
        self.clear_threshold = 10

        self.is_blocked = False

        # Detection confidence
        self.confidence = 0.35

    def detect(self, frame):

        height, width = frame.shape[:2]

        annotated = frame.copy()

        # Robot's forward walking corridor
        # Adjust these coordinates according to camera angle
        corridor = np.array([
            [int(width * 0.25), int(height * 0.30)],
            [int(width * 0.75), int(height * 0.30)],
            [int(width * 0.95), int(height * 0.98)],
            [int(width * 0.05), int(height * 0.98)]
        ], dtype=np.int32)

        # Create corridor mask
        corridor_mask = np.zeros(
            (height, width),
            dtype=np.uint8
        )

        cv2.fillPoly(
            corridor_mask,
            [corridor],
            255
        )

        # Draw walking corridor
        cv2.polylines(
            annotated,
            [corridor],
            True,
            (255, 255, 0),
            2
        )

        # Detect ALL available COCO classes
        results = self.model.predict(
            frame,
            conf=self.confidence,
            verbose=False
        )

        person_detected = False
        detected_objects = []

        # Process detections
        for box in results[0].boxes:

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0].tolist()
            )

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])

            object_name = self.model.names[class_id]

            detected_objects.append(object_name)

            color = (0, 255, 0)
            label = f"{object_name} {confidence:.2f}"

            # Special processing for people
            if object_name == "person":

                person_height = y2 - y1

                # Focus on lower 60% of person
                lower_y = int(
                    y1 + person_height * 0.40
                )

                # Clamp coordinates to image boundaries
                x1c = max(0, x1)
                x2c = min(width, x2)
                lower_y = max(0, lower_y)
                y2c = min(height, y2)

                inside_corridor = False

                if x2c > x1c and y2c > lower_y:

                    person_region = corridor_mask[
                        lower_y:y2c,
                        x1c:x2c
                    ]

                    overlap_pixels = cv2.countNonZero(
                        person_region
                    )

                    total_pixels = person_region.size

                    overlap_ratio = (
                        overlap_pixels / max(total_pixels, 1)
                    )

                    # Person overlaps the walking corridor
                    inside_corridor = overlap_ratio >= 0.10

                if inside_corridor:

                    person_detected = True

                    color = (0, 0, 255)
                    label = "PERSON BLOCKING PATH"

                else:

                    color = (255, 0, 0)
                    label = f"PERSON {confidence:.2f}"

            # Draw object bounding box
            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                color,
                2
            )

            # Draw object label
            cv2.putText(
                annotated,
                label,
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        # Temporal filtering for person obstruction
        if person_detected:

            self.blocked_frames += 1
            self.clear_frames = 0

        else:

            self.clear_frames += 1
            self.blocked_frames = 0

        # Activate STOP
        if self.blocked_frames >= self.block_threshold:

            self.is_blocked = True

        # Clear STOP
        if self.clear_frames >= self.clear_threshold:

            self.is_blocked = False

        # Display robot path status
        if self.is_blocked:

            status = "PATH BLOCKED - STOP"
            status_color = (0, 0, 255)

        else:

            status = "PATH CLEAR"
            status_color = (0, 255, 0)

        cv2.putText(
            annotated,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            status_color,
            3
        )

        # Display number of detected objects
        object_count = len(detected_objects)

        cv2.putText(
            annotated,
            f"Objects detected: {object_count}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        return annotated, self.is_blocked, detected_objects


def main():

    detector = PersonBlockDetector()

    # Open webcam
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():

        print("ERROR: Camera not available")
        return

    print("-----------------------------------")
    print("LABMATE OBJECT DETECTION STARTED")
    print("-----------------------------------")
    print("Detecting all available COCO classes")
    print("Person path obstruction detection ON")
    print("Press Q to quit")
    print("-----------------------------------")

    while True:

        ret, frame = camera.read()

        if not ret:
            print("ERROR: Cannot read camera frame")
            break

        output, blocked, objects = detector.detect(frame)

        # Print robot status
        if blocked:

            print("[ROBOT] STOP: Person blocking path")

        # Display camera output
        cv2.imshow(
            "LabMate - Object and Person Detection",
            output
        )

        # Quit when Q is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()

    print("LabMate detection stopped")


if __name__ == "__main__":

    main()
