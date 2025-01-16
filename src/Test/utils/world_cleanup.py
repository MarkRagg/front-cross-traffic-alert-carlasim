import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

vehicles = world.get_actors().filter('vehicle.*') 
for vehicle in vehicles:
    vehicle.destroy()