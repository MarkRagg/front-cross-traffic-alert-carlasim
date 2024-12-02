import carla
import time
from RadarSensor import RadarSensor 

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = -1 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(30))
        bp.set_attribute('vertical_fov', str(2))
        bp.set_attribute('range', '20')  # Maximum range of the radar
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data, parent_actor.get_velocity().length()))

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

            # for detection in radar_data:
            distance = detect.depth
            absolute_speed = abs(detect.velocity) - vehicle_velocity
            print(f" {len(radar_data)} | Distance: {distance:.2f} m, Speed: {absolute_speed:.2f} m/s", end="\r")
            # TTC_calculator(distance, absolute_speed)  # Assuming you have a function to calculate Time to Collision (TTC)

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

def radar_callback(data: carla.RadarMeasurement):
    global min_ttc, min_distance, absolute_speed
    min_ttc = float('inf')

    for detection, i in zip(data, range(len(data))):
        absolute_speed = abs(detection.velocity)

        # Calculate TTC
        if absolute_speed != 0:
            ttc = detection.depth / absolute_speed
            if ttc < min_ttc:
                min_ttc = ttc


def spawn_vehicle(vehicle_index=0, spawn_index=0, x_offset=0, y_offset=0, pattern='vehicle.*model3*') -> carla.Vehicle:
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    spawn_point.location.x += x_offset
    spawn_point.location.y += y_offset
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

# Spawn ego vehicle
ego_vehicle = spawn_vehicle(x_offset=155, y_offset=-30)

radar_left = RadarSensor(ego_vehicle, y=1, pitch=5, yaw=45)   # Attach left radar to 'vehicle'
radar_right = RadarSensor(ego_vehicle, y=-1, pitch=5, yaw=-45)  # Attach right radar to 'vehicle'

ego_vehicle.set_autopilot(True)

target_vehicle_array = []
# Spawn target vehicle for testing
for i in range (0, 25):
    target_vehicle = spawn_vehicle(spawn_index=i)
    target_vehicle.set_autopilot()


# Variable to store the minimum TTC
min_ttc = float('inf')

try:
    while True:
        # time.sleep(0.5)
        world.tick()

except KeyboardInterrupt:
    print("Keyboard interrupt detected.")

finally:
    vehicles = world.get_actors().filter('vehicle.*') 
    for vehicle in vehicles:
        vehicle.destroy()
