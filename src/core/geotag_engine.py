import math
import numpy as np

class Geolocator:
    """
    Handles translation from Camera Pixel Coordinates to Global GPS Coordinates.
    Utilizes intrinsic camera matrices and drone telemetry (Euler angles, altitude)
    to compute intersections with a flat-earth plane.
    """
    def __init__(self, video_w=1920, video_h=1080, fov_h=67.55, fov_v=41.23, mount_pitch=-45.0, latency_s=0.150):
        self.latency_s = latency_s
        
        # 1. Setup Intrinsic Matrix K
        fx = (video_w / 2) / math.tan(math.radians(fov_h / 2))
        fy = (video_h / 2) / math.tan(math.radians(fov_v / 2))
        cx, cy = video_w / 2, video_h / 2
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.K_inv = np.linalg.inv(K)

        # 2. Setup Camera->Body Rotation (Static Mount)
        # Assuming R_pitch rotates around Y axis
        theta = math.radians(mount_pitch)
        c, s = math.cos(theta), math.sin(theta)
        R_pitch = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        
        # Base: Z=Look -> X=Fwd
        R_base = np.array([[0,0,1], [1,0,0], [0,1,0]])
        self.R_cam_body = R_pitch @ R_base

    def pixel_to_gps(self, u, v, drone_state, velocity_ned=(0,0,0), yaw_rate_deg_s=0):
        """
        Calculates the GPS coordinate of a pixel (u,v).
        Includes latency compensation by backtracking the drone's position based on velocity.
        """
        if drone_state.alt < 0.5: 
            return 0.0, 0.0, 0.0 # Safety: Too low to ground
        
        # Latency Compensation (Backtrack)
        vx, vy, vz = velocity_ned
        backtrack_lat = drone_state.lat - (vx * self.latency_s) / 111132.0
        backtrack_lon = drone_state.lon - (vy * self.latency_s) / (111132.0 * math.cos(math.radians(drone_state.lat)))
        backtrack_alt = drone_state.alt - (vz * self.latency_s)
        backtrack_yaw = drone_state.yaw - (yaw_rate_deg_s * self.latency_s)

        # Ray in Cam Frame
        ray_cam = self.K_inv @ np.array([u, v, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)
        
        # Ray in Body Frame
        ray_body = self.R_cam_body @ ray_cam
        
        # Ray in NED Frame (Rotation Matrices)
        r = math.radians(drone_state.roll)
        p = math.radians(drone_state.pitch)
        y = math.radians(backtrack_yaw)
        
        R_z = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0,0,1]])
        R_y = np.array([[math.cos(p), 0, math.sin(p)], [0,1,0], [-math.sin(p), 0, math.cos(p)]])
        R_x = np.array([[1,0,0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        
        ray_ned = R_z @ R_y @ R_x @ ray_body
        
        # Intersect Ground (Z=0 relative to home)
        if ray_ned[2] <= 0: 
            return 0.0, 0.0, 0.0 # Ray pointing at horizon/sky
        
        slant_range = backtrack_alt / ray_ned[2] 
        
        dn = slant_range * ray_ned[0]
        de = slant_range * ray_ned[1]
        
        # Coordinate Offset
        d_lat = dn / 111132.0
        d_lon = de / (111132.0 * math.cos(math.radians(backtrack_lat)))
        
        target_lat = backtrack_lat + d_lat
        target_lon = backtrack_lon + d_lon
        
        return target_lat, target_lon, slant_range
