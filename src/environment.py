import carla
import time
from RadarSensor import RadarSensor 
from carla import Rotation

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town10HD')  # 1, 2, 3, 7
# spectator = world.get_spectator()

def move_spectator_to(transform, spectator, distance=5.0, x=0, y=0, z=3, yaw=0, pitch=-20, roll=0):
    back_location = transform.location - transform.get_forward_vector() * distance
    
    back_location.x += x
    back_location.y += y
    back_location.z += z
    transform.rotation.yaw += yaw
    transform.rotation.pitch = pitch
    transform.rotation.roll = roll
    
    spectator_transform = carla.Transform(back_location, transform.rotation)
    
    # spectator.set_transform(spectator_transform)

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


def spawn_vehicle(vehicle_index=0, spawn_index=0, x_offset=0, y_offset=0, pattern='vehicle.*model3*', rotation=None) -> carla.Vehicle:
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    spawn_point.location.x += x_offset
    spawn_point.location.y += y_offset
    if rotation is not None:
        spawn_point.rotation = rotation
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

# Spawn ego vehicle
# ego_vehicle = spawn_vehicle(x_offset=157, y_offset=-30)
ego_vehicle = spawn_vehicle(x_offset=181, y_offset=-20, rotation=Rotation(yaw=180, pitch=0, roll=0))
# ego_vehicle = spawn_vehicle(spawn_index=1, x_offset=10, y_offset=-9, rotation=Rotation(yaw=90, pitch=0, roll=0))

radar_right = RadarSensor(ego_vehicle, "right", y=1, pitch=5, yaw=45)   # Attach right radar to 'vehicle'
radar_left = RadarSensor(ego_vehicle, "left", y=-1, pitch=5, yaw=-45)  # Attach left radar to 'vehicle'

radar_right.start_timer(3)
radar_left.start_timer(3)

target_vehicle_array = []
# Spawn target vehicle for testing
for i in range (2, 50): # 0 is the ego vehicle
    target_vehicle = spawn_vehicle(spawn_index=i)
    target_vehicle.set_autopilot()


# Variable to store the minimum TTC
min_ttc = float('inf')

# time.sleep(4)
# ego_vehicle.set_autopilot(True)
try:
    while True:
        # move_spectator_to(ego_vehicle.get_transform(), spectator)
        world.tick()

except KeyboardInterrupt:
    print("Keyboard interrupt detected.")

finally:
    vehicles = world.get_actors().filter('vehicle.*') 
    for vehicle in vehicles:
        vehicle.destroy()
