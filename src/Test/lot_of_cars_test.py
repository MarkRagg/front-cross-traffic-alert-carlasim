import utils.utilities as utils
from carla import Rotation

def lot_of_cars_test():
    ego_vehicle = utils.spawn_vehicle(x_offset=181, y_offset=-20, rotation=Rotation(yaw=180, pitch=0, roll=0))
    # ego_vehicle = spawn_vehicle(x_offset=157, y_offset=-30)
    # ego_vehicle = spawn_vehicle(spawn_index=1, x_offset=10, y_offset=-9, rotation=Rotation(yaw=90, pitch=0, roll=0))

    utils.radar_setup(ego_vehicle)

    # Spawn target vehicles for testing
    for i in range (1, 50): # 0 is the ego vehicle
        target_vehicle = utils.spawn_vehicle(spawn_index=i)
        target_vehicle.set_autopilot()

    try:
        while True:
            utils.move_spectator_to(ego_vehicle.get_transform(), utils.spectator)
            utils.world.tick()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    finally:
        vehicles = utils.world.get_actors().filter('vehicle.*') 
        for vehicle in vehicles:
            vehicle.destroy()
