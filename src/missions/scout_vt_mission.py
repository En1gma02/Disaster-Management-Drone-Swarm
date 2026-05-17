import sys
import time
import cv2
from shapely.geometry import Point, Polygon as ShapelyPolygon

from core.drone_interface import MavlinkDrone
from core.path_planner import parse_kml, choose_best_entry_point, choose_best_overlap, prune_path_by_coverage_barrier, prune_return_with_low_gain, shortcut_redundant_waypoints, trim_redundant_tail
from core.geotag_engine import Geolocator
from core.vision_system import YoloDetector

# --- CONFIGURATION ---
KML_FILE = "config/geofences/path2.kml"
SYSTEM_ADDR = "udp:127.0.0.1:14560" # UDP Hub from unified GCS
FLIGHT_ALTITUDE = 15.0
SENSOR_WIDTH = 30
OVERLAP = 0.2
CAMERA_INDEX = 0

SOS_TUNE = "T255 O7 L8 c e c e c e P8 c e c e c e P8 c e c e"

def run_scout_mission():
    print("="*50)
    print("SCOUT (VTOL) MISSION INITIALIZING")
    print("="*50)

    # 1. Initialize Modules
    drone = MavlinkDrone(SYSTEM_ADDR)
    if not drone.connected:
        print("-- [SCOUT] Failed to connect to GCS Hub. Exiting.")
        return

    # Assuming mount pitch is -45 degrees (downward/forward)
    geolocator = Geolocator(mount_pitch=-45.0, latency_s=0.7) 
    detector = YoloDetector(model_path="models/human_detector.pt")

    # 2. Path Generation (MBR Optimization)
    print(f"-- [SCOUT] Parsing Geofence from {KML_FILE}...")
    try:
        geofence_coords = parse_kml(KML_FILE)
        geofence = ShapelyPolygon(geofence_coords)
        start_loc = Point(geofence_coords[0]) 

        entry_point, optimal_path, _, _, _, _ = choose_best_entry_point(
            geofence, start_loc, OVERLAP, SENSOR_WIDTH
        )
        
        best_overlap, best_overlap_path, _, _, _, _, _ = choose_best_overlap(
            geofence, entry_point, OVERLAP, SENSOR_WIDTH
        )
        if best_overlap_path and len(best_overlap_path) > 1:
            optimal_path = best_overlap_path
            
        optimal_path = prune_path_by_coverage_barrier(optimal_path, geofence, SENSOR_WIDTH, 0.95, 0.005, 2)
        optimal_path = prune_return_with_low_gain(optimal_path, entry_point, geofence, SENSOR_WIDTH, 0.95, 0.005, 2, 0.0003)
        optimal_path = shortcut_redundant_waypoints(optimal_path, geofence, SENSOR_WIDTH, 0.003)
        optimal_path = trim_redundant_tail(optimal_path, geofence, SENSOR_WIDTH, 0.005)
        
        print(f"-- [SCOUT] ✓ Generated optimized MBR path: {len(optimal_path)} waypoints.")
    except Exception as e:
        print(f"-- [SCOUT] Error generating path: {e}")
        return

    # 3. Execution
    drone.wait_gps()
    if not drone.upload_mission(optimal_path, FLIGHT_ALTITUDE):
        return

    drone.set_mode("GUIDED")
    time.sleep(1)
    if not drone.arm():
        return

    drone.takeoff(FLIGHT_ALTITUDE)
    time.sleep(10) # Wait for climb
    drone.start_mission_cmd()
    
    # 4. Surveillance & Geotagging Loop
    print("-- [SCOUT] Commencing Surveillance. Streaming via OpenHD...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    tracked_targets = []
    
    # Open log file
    with open("detected_targets.txt", "w") as f:
        f.write("LAT,LON,ALT\n")
        
    try:
        while drone.running:
            if cap.isOpened():
                ret, frame = cap.read()
                if not ret: continue
                
                detections = detector.detect(frame)
                state, velocity, yaw_rate, abs_alt = drone.get_state()
                
                for (x, y, w, h, label) in detections:
                    cx, cy = x + w/2, y + h/2
                    lat, lon, slant_dist = geolocator.pixel_to_gps(cx, cy, state, velocity, yaw_rate)
                    
                    if lat != 0.0:
                        # Simple uniqueness check (10 meters)
                        is_new = True
                        for (t_lat, t_lon) in tracked_targets:
                            # Rough distance approx
                            dist = ((lat - t_lat)**2 + (lon - t_lon)**2)**0.5 * 111132
                            if dist < 10.0:
                                is_new = False
                                break
                                
                        if is_new:
                            print(f"🎯 [SCOUT] Survivor Detected! {lat:.6f}, {lon:.6f} (Dist: {slant_dist:.1f}m)")
                            tracked_targets.append((lat, lon))
                            drone.play_tune(SOS_TUNE)
                            
                            with open("detected_targets.txt", "a") as f:
                                f.write(f"{lat:.7f},{lon:.7f},{abs_alt:.2f}\n")
                        
                        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,0,255), 2)
                        cv2.putText(frame, f"{label}", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                cv2.imshow("Scout OpenHD Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.1)

            # Check if mission complete (seq >= total)
            if drone.mission_total > 0 and drone.mission_current >= drone.mission_total:
                print("-- [SCOUT] Waypoints exhausted.")
                break

    except KeyboardInterrupt:
        print("\n-- [SCOUT] Mission aborted by user.")

    print("-- [SCOUT] Mission Complete. Signalling Mule and Returning to Base.")
    with open("detected_targets.txt", "a") as f:
        f.write("MISSION_COMPLETE\n")
        
    if cap.isOpened(): cap.release()
    cv2.destroyAllWindows()
    drone.return_to_launch()
    time.sleep(5)
    drone.close()

if __name__ == "__main__":
    run_scout_mission()
