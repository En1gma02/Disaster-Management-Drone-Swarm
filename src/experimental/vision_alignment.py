import time
import math
from core.drone_interface import MavlinkDrone
from core.vision_system import ColorDetector
import cv2

# Config
SYSTEM_ADDR = "udp:127.0.0.1:14561"
ALIGNMENT_PIXEL_THRESHOLD = 75
FORWARD_SPEED = 0.15
ALIGNMENT_SPEED = 0.25
CAMERA_INDEX = 0

class VisionAlignmentDrop:
    def __init__(self):
        self.drone = MavlinkDrone(SYSTEM_ADDR)
        self.detector = ColorDetector(color_name="white") # Detect white target pad
        
    def set_body_velocity(self, vx, vy, vz):
        """Send velocity command in BODY frame (x=forward, y=right, z=down)."""
        self.drone.master.mav.set_position_target_local_ned_send(
            0, self.drone.master.target_system, self.drone.master.target_component,
            8, # MAV_FRAME_BODY_NED
            0b0000111111000111, # Only speeds enabled
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0, 0, 0)

    def execute_alignment_and_drop(self, timeout_s=45.0):
        print("-- [VISION] Commencing Visual Alignment...")
        cap = cv2.VideoCapture(CAMERA_INDEX)
        
        align_start = time.time()
        aligned = False

        while time.time() - align_start < timeout_s:
            ret, frame = cap.read()
            if not ret: continue
            
            frame_h, frame_w = frame.shape[:2]
            center_x, center_y = frame_w // 2, frame_h // 2
            
            detections = self.detector.detect(frame)
            
            if not detections:
                print("   [VISION] Target lost! Hovering.", end='\r')
                self.set_body_velocity(0, 0, 0)
                continue

            # Take the largest detection
            detections.sort(key=lambda d: d[2]*d[3], reverse=True)
            x, y, w, h, _ = detections[0]
            
            spot_cx = x + w//2
            spot_cy = y + h//2
            
            rel_x = spot_cx - center_x
            rel_y = spot_cy - center_y

            # Check alignment
            if abs(rel_x) <= ALIGNMENT_PIXEL_THRESHOLD and abs(rel_y) <= ALIGNMENT_PIXEL_THRESHOLD:
                print("\n-- [VISION] Alignment Complete. Hovering.")
                self.set_body_velocity(0, 0, 0)
                time.sleep(1.0)
                aligned = True
                break

            # Velocity Control (Proportional-ish)
            # Y-Axis (Forward/Back) -> rel_y > 0 means target is below center (behind drone)
            if rel_y > ALIGNMENT_PIXEL_THRESHOLD: vx = -ALIGNMENT_SPEED
            elif rel_y < -ALIGNMENT_PIXEL_THRESHOLD: vx = ALIGNMENT_SPEED
            else: vx = 0.0

            # X-Axis (Left/Right) -> rel_x > 0 means target is right of center
            if rel_x > ALIGNMENT_PIXEL_THRESHOLD: vy = ALIGNMENT_SPEED
            elif rel_x < -ALIGNMENT_PIXEL_THRESHOLD: vy = -ALIGNMENT_SPEED
            else: vy = 0.0
            
            print(f"   [VISION] Correcting: vx={vx:.2f} vy={vy:.2f} | offY={rel_y} offX={rel_x}", end='\r')
            self.set_body_velocity(vx, vy, 0)
            
            # Draw HUD
            cv2.rectangle(frame, (center_x-ALIGNMENT_PIXEL_THRESHOLD, center_y-ALIGNMENT_PIXEL_THRESHOLD), 
                                 (center_x+ALIGNMENT_PIXEL_THRESHOLD, center_y+ALIGNMENT_PIXEL_THRESHOLD), (0,255,0), 2)
            cv2.circle(frame, (spot_cx, spot_cy), 5, (0,0,255), -1)
            cv2.imshow("Delivery Alignment", frame)
            cv2.waitKey(1)

        cap.release()
        cv2.destroyAllWindows()

        if aligned:
            print("-- [VISION] Executing Payload Drop!")
            # Trigger servo logic here
        else:
            print("\n-- [VISION] Alignment Timeout. Aborting drop.")

if __name__ == "__main__":
    v = VisionAlignmentDrop()
    v.execute_alignment_and_drop()
