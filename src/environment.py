import carla
import time
from RadarSensor import RadarSensor 

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

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
