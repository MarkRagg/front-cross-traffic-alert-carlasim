import carla
import math
import weakref

class RadarSensor(object):
    def __init__(self, parent_actor, x=0.5, y=0.5, z=-0.5, pitch=5, yaw=0, roll=0):
        self.sensor = None
        self._parent = parent_actor

        bound_x = x + self._parent.bounding_box.extent.x
        bound_y = y + self._parent.bounding_box.extent.y
        bound_z = z + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(100))
        bp.set_attribute('vertical_fov', str(2))
        bp.set_attribute('range', '15')  # Maximum range of the radar
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z, y=bound_y),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data, parent_actor.get_velocity().length()))

    def __del__(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()

    @staticmethod
    def _Radar_callback(weak_self, radar_data, vehicle_velocity):
        self = weak_self()
        if not self:
            return
        
        # Extract the radar data points from the radar sensor
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)  # Azimuth angle in degrees
            alt = math.degrees(detect.altitude)  # Altitude angle in degrees
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)  # Adjust distance slightly

            # Calculating velocity of target vehicle
            distance = detect.depth
            absolute_speed = abs(detect.velocity) - vehicle_velocity

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