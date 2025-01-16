import carla
from carla import Rotation
from sensors.radar_sensor import RadarSensor

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town10HD')
spectator = world.get_spectator()

def accelerate_vehicle(vehicle, throttle_value=0.3):
    control = carla.VehicleControl()
    control.throttle = throttle_value
    vehicle.apply_control(control)

def move_spectator_to(transform, spectator, distance=5.0, x=0, y=0, z=3, yaw=0, pitch=-20, roll=0):
    back_location = transform.location - transform.get_forward_vector() * distance
    
    back_location.x += x
    back_location.y += y
    back_location.z += z
    transform.rotation.yaw += yaw
    transform.rotation.pitch = pitch
    transform.rotation.roll = roll
    
    spectator_transform = carla.Transform(back_location, transform.rotation)
    spectator.set_transform(spectator_transform)

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

ego_vehicle = spawn_vehicle(x_offset=181, y_offset=-20, rotation=Rotation(yaw=180, pitch=0, roll=0))

radar_right = RadarSensor(ego_vehicle, "right", y=1, pitch=5, yaw=60)
radar_left = RadarSensor(ego_vehicle, "left", y=-1, pitch=5, yaw=-60)

# Spawn target vehicle for testing
target_vehicle = spawn_vehicle(x_offset=175, y_offset=-40, rotation=Rotation(yaw=90, pitch=0, roll=0))
accelerate_vehicle(target_vehicle)

try:
    while True:
        move_spectator_to(ego_vehicle.get_transform(), spectator)
        world.tick()
except KeyboardInterrupt:
    print("Keyboard interrupt detected.")
finally:
    vehicles = world.get_actors().filter('vehicle.*') 
    for vehicle in vehicles:
        vehicle.destroy()
