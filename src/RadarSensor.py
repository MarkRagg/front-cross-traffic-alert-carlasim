import carla
import math
import weakref
import time
import threading

import numpy as np

FIVE_KMH = 1.38889
TEN_KMH = 2.77778
LEFT_TO_RIGHT_THRESHOLD = 30  # Example threshold for distance or velocity change to filter out movements
RIGHT_TO_LEFT_THRESHOLD = -30  # Negative threshold for opposite movement

class RadarSensor(object):
    side = None
    # TODO: reset this variable after a certain time
    lasts_detections = []

    @staticmethod
    def reset_detections_after_time(duration: int):
        time.sleep(duration)
        RadarSensor.lasts_detections = []
        print("Detections reset!")

    @staticmethod
    def start_timer(duration: int):
        # Create a thread that will call `reset_detections_after_time`
        reset_thread = threading.Thread(target=RadarSensor.reset_detections_after_time, args=(duration,))
        reset_thread.daemon = True  # Daemonize the thread to ensure it exits when the program exits
        reset_thread.start()


    def __init__(self, parent_actor, side, x=0.5, y=0.5, z=-0.5, pitch=5, yaw=0, roll=0):
        self.sensor = None
        self._parent = parent_actor

        bound_x = x + self._parent.bounding_box.extent.x
        bound_y = y + self._parent.bounding_box.extent.y
        bound_z = z + self._parent.bounding_box.extent.z

        side = side

        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(90))
        bp.set_attribute('vertical_fov', str(2))
        bp.set_attribute('range', '15')
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z, y=bound_y),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data, parent_actor.get_velocity().length(), side))

    def __del__(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()

    @staticmethod
    def _Radar_callback(weak_self, radar_data, ego_velocity, side):
        self = weak_self()
        if not self:
            return
        azis = []
        # Extract the radar data points from the radar sensor
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)  # Azimuth angle in degrees
            alt = math.degrees(detect.altitude)  # Altitude angle in degrees
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)  # Adjust distance slightly
            azis.append(azi)
            # Calculating velocity of target vehicle
            distance = detect.depth
            abs_detected_speed = abs(detect.velocity) - ego_velocity
            # print(f"{ego_velocity}", end="\r")
##w####
            points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (len(radar_data), 4))
            # print(points)

            # code convert array into list and measure distance
            L = []
            pointslist=points.tolist()
            for i in range(len(pointslist)):
                L.append(pointslist[i-1][-1])

            ave = sum(L)/len(L)
            # print(ave, end="\r")
######
            # if (abs_detected_speed > FIVE_KMH and (ego_velocity < 3) and ave < 20):
            # if (abs_detected_speed > FIVE_KMH and ave < 10):
                # print(f"{ego_velocity, abs_detected_speed}")
                # print(f"target distance: {distance}")
                # print(side, end="\r")
        
                # if azi > LEFT_TO_RIGHT_THRESHOLD and side == "left":
                #     # Vehicle is likely moving from left to right (depending on your sensor's orientation)
                #     print(f"Vehicle is moving Left to Right: {side}, Speed: {azi}")
                #     # Skip this vehicle
                #     continue
                # elif azi < RIGHT_TO_LEFT_THRESHOLD and side == "left":
                #     # Vehicle is likely moving from right to left
                #     print(f"Vehicle is moving Right to Left: {side}, Speed: {azi}")
                #     # Skip this vehicle
                #     continue
                # if azi > RIGHT_TO_LEFT_THRESHOLD and side == "right":
                #     # Vehicle is likely moving from left to right (depending on your sensor's orientation)
                #     print(f"Vehicle is moving Left to Right: {side}, Speed: {azi}")
                #     # Skip this vehicle
                #     continue
                # elif azi < LEFT_TO_RIGHT_THRESHOLD and side == "right":
                #     # Vehicle is likely moving from right to left
                #     print(f"Vehicle is moving Right to Left: {side}, Speed: {azi}")
                #     # Skip this vehicle
                #     continue

            # Get current rotation of radar sensor
            current_rot = radar_data.transform.rotation
            # Transform the radar point based on the azimuth and altitude
            carla.Transform(
                carla.Location(),
                carla.Rotation(pitch=current_rot.pitch + alt, yaw=current_rot.yaw + azi, roll=current_rot.roll)
            ).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range  # Normalize velocity
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(-1.0, 0.0, -1.0 - norm_velocity)) * 255.0)
            
            # Draw the radar point in the world
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))
        
        # print(ave, end="\r")
        # TODO: Implement logic to determine if vehicle is moving from left to right or right to left

        RadarSensor.lasts_detections

        if len(azis) > 7 and abs_detected_speed > FIVE_KMH and ego_velocity < 10 and ave < 10:
            azi_avg = sum(azis) / len(azis)
            if len(RadarSensor.lasts_detections) >= 3:
                lasts_detections_avg = sum(RadarSensor.lasts_detections[:-3]) / 3
            else:
                lasts_detections_avg = 0.5
            print(f"{RadarSensor.lasts_detections, azi_avg, lasts_detections_avg}", end="\r")
            if azi_avg > LEFT_TO_RIGHT_THRESHOLD and side == "left" and lasts_detections_avg <= 0.5:
                print(f"Vehicle is moving Left to Right: {side}")
                RadarSensor.lasts_detections.append(0)
            elif azi_avg < RIGHT_TO_LEFT_THRESHOLD and side == "right" and lasts_detections_avg >= 0.5:
                print(f"Vehicle is moving Right to Left: {side}")
                RadarSensor.lasts_detections.append(1)

            # elif azi_avg < RIGHT_TO_LEFT_THRESHOLD and side == "left":
                # print(f"Vehicle is moving Right to Left: {side}", end="\r")
            # elif azi_avg > LEFT_TO_RIGHT_THRESHOLD and side == "right":
            #     print(f"Vehicle is moving R to L: {side}")