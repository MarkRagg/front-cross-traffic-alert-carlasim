import os
import test.utils.utilities as utils
from test.sensors.radar_sensor import RadarSensor

def show_menu():
    print("FRONT CROSS TRAFFIC ALERT CARLA SIMULATOR TEST:")
    print("1. Test with a lot of cars")
    print("2. Left side test")
    print("3. Right side test")
    print("4. Cobined test")
    print("0. Exit")

def main():
    show_menu()
    ego_vehicle = utils.spawn_ego_vehicle()

    radar_right = RadarSensor(ego_vehicle, "right", y=1, pitch=5, yaw=50)
    radar_left = RadarSensor(ego_vehicle, "left", y=-1, pitch=5, yaw=-50)
    radar_right.start_timer(3)
    radar_left.start_timer(3)
    
    utils.move_spectator_to(ego_vehicle.get_transform(), utils.spectator)

    while True:
        choice = input("Enter the number of the option: ")
        if choice == '1':
            utils.spawn_vehicles(50)
            break
        elif choice == '2':
            utils.accelerate_vehicle(utils.spawn_left_vehicle())
            break
        elif choice == '3':
            utils.accelerate_vehicle(utils.spawn_right_vehicle())
            break
        elif choice == '4':
            utils.accelerate_vehicle(utils.spawn_left_vehicle())
            utils.accelerate_vehicle(utils.spawn_right_vehicle())
            break
        elif choice == '0':
            utils.world_cleanup()
            exit()
        else:
            print("Invalid choice, please try again.")

    os.system('cls')
    print("Open the CARLA server window!")

    try:
        while True:
            utils.world.tick()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    finally:
        utils.world_cleanup()
 

if __name__ == "__main__":
    main()