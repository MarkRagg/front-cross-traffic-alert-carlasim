import carla
import math
import weakref
import time
import threading
import numpy as np
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

FIVE_KMH = 1.38889
TEN_KMH = 2.77778
LEFT_TO_RIGHT_THRESHOLD = 25  # Example threshold for distance or velocity change to filter out movements
RIGHT_TO_LEFT_THRESHOLD = -25  # Negative threshold for opposite movement

# MQTT settings
MQTT_BROKER = "broker.mqtt-dashboard.com"
MQTT_PORT = 1883
MQTT_LEFT_TOPIC = "front_cross_traffic_alert/left"
MQTT_RIGHT_TOPIC = "front_cross_traffic_alert/right"

class RadarSensor(object):
    side = None
    left_detect = False
    right_detect = False
    mqtt_client = mqtt.Client()
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

    @staticmethod
    def reset_detections_after_time(duration: int):
        while True:
            time.sleep(duration)
            RadarSensor.left_detect = False
            RadarSensor.right_detect = False

    @staticmethod
    def start_timer(duration: int):
        reset_thread = threading.Thread(target=RadarSensor.reset_detections_after_time, args=(duration,))
        reset_thread.daemon = True
        reset_thread.start()


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
            points = np.reshape(points, (len(radar_data), 4))
            L = []
            pointsList=points.tolist()
            for i in range(len(pointsList)):
                L.append(pointsList[i-1][-1])

            ave = sum(L)/len(L)

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

        if len(azis) > 5 and abs_detected_speed > FIVE_KMH and ego_velocity < TEN_KMH and ave < 10:
            azi_avg = sum(azis) / len(azis)
            if azi_avg > LEFT_TO_RIGHT_THRESHOLD and side == "left" and not RadarSensor.right_detect:
                print(f"Vehicle is moving Left to Right: {side}")
                publish.single(topic=MQTT_LEFT_TOPIC, payload="vehicle detected!", hostname=MQTT_BROKER)
                RadarSensor.left_detect = True
            elif azi_avg < RIGHT_TO_LEFT_THRESHOLD and side == "right" and not RadarSensor.left_detect:
                print(f"Vehicle is moving Right to Left: {side}")
                publish.single(topic=MQTT_RIGHT_TOPIC, payload="vehicle detected!", hostname=MQTT_BROKER)
                RadarSensor.right_detect = True
