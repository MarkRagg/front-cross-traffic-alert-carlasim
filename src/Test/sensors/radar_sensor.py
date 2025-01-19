import carla
import math
import weakref
import time
import statistics
import threading
import numpy as np
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

FIVE_KMH = 1.38889 # 5km/h in m/s
TEN_KMH = 2.77778 # 10km/h in m/s
DEPTH_THRESHOLD = 20 # Depth treshold for considering the target vehicle a threat for the ego vehicle
LEFT_TO_RIGHT_THRESHOLD = 7  # Threshold for delta of azi changes to filter out direction
LEFT_TO_RIGHT_MAX_TRESHOLD = 12 # Max threshold for valid detection
RIGHT_TO_LEFT_THRESHOLD = -7  # Negative threshold for opposite movement
RIGHT_TO_LEFT_MAX_TRESHOLD = -12 # Max threshold for valid detection


# MQTT settings
MQTT_BROKER = "broker.mqtt-dashboard.com"
MQTT_PORT = 1883
MQTT_LEFT_TOPIC = "front_cross_traffic_alert/left"
MQTT_RIGHT_TOPIC = "front_cross_traffic_alert/right"

class RadarSensor(object):
    side = None
    mqtt_client = mqtt.Client()
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

    # List to save old detections for calculating delta
    previous_avg_azis = None
    older_avg_azis = None

    def __init__(self, parent_actor, side, x=0.5, y=0.5, z=-0.5, pitch=5, yaw=0, roll=0):
        self.sensor = None
        self._parent = parent_actor
        self.side = side
        bound_x = x + self._parent.bounding_box.extent.x
        bound_y = y + self._parent.bounding_box.extent.y
        bound_z = z + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(100))
        bp.set_attribute('vertical_fov', str(2))
        bp.set_attribute('range', '20')
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z, y=bound_y),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)),
            attach_to=self._parent)
        # weak_self to avoid circular reference
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
            abs_detected_speed = abs(detect.velocity) - ego_velocity
            points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (len(radar_data), 4)) # the forth element of the array is the depth
            depthList = []
            pointsList=points.tolist()
            for i in range(len(pointsList)):
                depthList.append(pointsList[i-1][-1])

            depth_avg = sum(depthList)/len(depthList)

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

        if len(azis) > 5 and abs_detected_speed > FIVE_KMH and ego_velocity < TEN_KMH and depth_avg < DEPTH_THRESHOLD:
            azi_avg = sum(azis) / len(azis)
            if RadarSensor.older_avg_azis is not None:
                delta_azis = azi_avg - RadarSensor.older_avg_azis
                if delta_azis > LEFT_TO_RIGHT_THRESHOLD and delta_azis < LEFT_TO_RIGHT_MAX_TRESHOLD and side == "left" and not RadarSensor.right_detect:
                    print(f"Vehicle is moving Left to Right")
                    # Send message to MQTT broker 
                    publish.single(topic=MQTT_LEFT_TOPIC, payload="vehicle detected!", hostname=MQTT_BROKER)
                elif delta_azis < RIGHT_TO_LEFT_THRESHOLD and delta_azis > RIGHT_TO_LEFT_MAX_TRESHOLD and side == "right" and not RadarSensor.left_detect:
                    print(f"Vehicle is moving Right to Left") 
                    # Send message to MQTT broker  
                    publish.single(topic=MQTT_RIGHT_TOPIC, payload="vehicle detected!", hostname=MQTT_BROKER)
            
            RadarSensor.older_avg_azis = RadarSensor.previous_avg_azis
            # Update previous_avg_azis with the current azimuths
            RadarSensor.previous_avg_azis = azi_avg
