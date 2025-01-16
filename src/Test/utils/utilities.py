import carla
from test.sensors.radar_sensor import RadarSensor
from carla import Rotation

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
spectator = world.get_spectator()

def world_cleanup():
    vehicles = world.get_actors().filter('vehicle.*') 
    for vehicle in vehicles:
        vehicle.destroy()

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

def spawn_ego_vehicle():
    return spawn_vehicle(x_offset=181, y_offset=-20, rotation=Rotation(yaw=180, pitch=0, roll=0))

def spawn_left_vehicle():
    return spawn_vehicle(x_offset=175, y_offset=20, rotation=Rotation(yaw=270, pitch=0, roll=0))

def spawn_right_vehicle():
    return spawn_vehicle(x_offset=170, y_offset=-80, rotation=Rotation(yaw=90, pitch=0, roll=0))

def spawn_vehicles(num_vehicles=50):
    for i in range (1, num_vehicles): # 0 is the ego vehicle
        target_vehicle = spawn_vehicle(spawn_index=i)
        target_vehicle.set_autopilot()

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

def accelerate_vehicle(vehicle, throttle_value=0.3):
    control = carla.VehicleControl()
    control.throttle = throttle_value
    vehicle.apply_control(control)
