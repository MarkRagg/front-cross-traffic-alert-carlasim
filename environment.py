import carla
import time

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

# Add the radar sensor
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '10')  # Horizontal field of view
radar_bp.set_attribute('vertical_fov', '10')    # Vertical field of view
radar_bp.set_attribute('range', '20')           # Maximum range

radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
radar = world.spawn_actor(radar_bp, radar_transform, attach_to=ego_vehicle)

target_vehicle_array = []
# Spawn target vehicle for testing
for i in range (0, 2):
    target_vehicle = spawn_vehicle(x_offset=140, y_offset=-80)
    target_vehicle.set_autopilot()
    target_vehicle_array.append(target_vehicle)
    time.sleep(5) 


# Variable to store the minimum TTC
min_ttc = float('inf')

# Register the radar callback
radar.listen(radar_callback)

try:
    while True:
        time.sleep(0.5)
        print(f"Ego vehicle: {absolute_speed}")

except KeyboardInterrupt:
    print("Keyboard interrupt detected.")

finally:
    radar.stop()
    radar.destroy()
    ego_vehicle.destroy()
    for vehicle in target_vehicle_array:
        vehicle.destroy()
