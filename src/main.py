import os
import test.utils.utilities as utils
from test.sensors.radar_sensor import RadarSensor

def show_menu():
    print("FRONT CROSS TRAFFIC ALERT CARLA SIMULATOR TEST:")
    print("1. Test with a lot of cars")
    print("2. Left side test")
    print("3. Right side test")
    print("4. Cobined test")
    print("5. Little town left side test")
    print("6. Little town right side test")
    print("0. Exit")

def main():
    show_menu()
    while True:
        choice = input("Enter the number of the option: ")
        if choice == '1':
            ego_vehicle = utils.spawn_ego_vehicle()
            utils.spawn_vehicles(30)
            break
        elif choice == '2':
            ego_vehicle = utils.spawn_ego_vehicle()
            utils.accelerate_vehicle(utils.spawn_left_vehicle())
            break
        elif choice == '3':
            ego_vehicle = utils.spawn_ego_vehicle()
            utils.accelerate_vehicle(utils.spawn_right_vehicle())
            break
        elif choice == '4':
            ego_vehicle = utils.spawn_ego_vehicle()
            utils.accelerate_vehicle(utils.spawn_left_vehicle(), 0.4)
            utils.accelerate_vehicle(utils.spawn_right_vehicle(), 0.3)
            break
        elif choice == '5':
            ego_vehicle = utils.spawn_ego_vehicle("Town01")
            utils.accelerate_vehicle(ego_vehicle, 0.15)
            utils.accelerate_vehicle(utils.spawn_left_vehicle("Town01"), 0.3)
            break
        elif choice == '6':
            ego_vehicle = utils.spawn_ego_vehicle("Town01")
            utils.accelerate_vehicle(ego_vehicle, 0.15)
            utils.accelerate_vehicle(utils.spawn_right_vehicle("Town01"), 0.3)
            break
        elif choice == '0':
            utils.world_cleanup()
            exit()
        else:
            print("Invalid choice, please try again.")

    os.system('cls')
    print("Open the CARLA server window!")

    radar_right = RadarSensor(ego_vehicle, "right", y=1, pitch=5, yaw=50)
    radar_left = RadarSensor(ego_vehicle, "left", y=-1, pitch=5, yaw=-50)

    utils.move_spectator_to(ego_vehicle.get_transform(), utils.spectator)

    try:
        while True:
            utils.world.tick()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    finally:
        utils.world_cleanup()
 

if __name__ == "__main__":
    main()