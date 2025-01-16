import utils.utilities as utils
from carla import Rotation

ego_vehicle = utils.spawn_vehicle(x_offset=181, y_offset=-20, rotation=Rotation(yaw=180, pitch=0, roll=0))

utils.radar_setup(ego_vehicle)

# Spawn target vehicle for testing
target_vehicle = utils.spawn_vehicle(x_offset=175, y_offset=20, rotation=Rotation(yaw=270, pitch=0, roll=0))
utils.accelerate_vehicle(target_vehicle)


