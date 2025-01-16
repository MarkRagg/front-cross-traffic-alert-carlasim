import utils.utilities as utils

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
    utils.radar_setup(ego_vehicle)
    utils.move_spectator_to(ego_vehicle.get_transform(), utils.spectator)

    choice = input("Enter the number of the option: ")

    if choice == '1':
        utils.spawn_vehicles(50)
    elif choice == '2':
        utils.accelerate_vehicle(utils.spawn_left_vehicle())
    elif choice == '3':
        utils.accelerate_vehicle(utils.spawn_right_vehicle())
    elif choice == '4':
        utils.accelerate_vehicle(utils.spawn_left_vehicle())
        utils.accelerate_vehicle(utils.spawn_right_vehicle())
    elif choice == '0':
        utils.world_cleanup()
        exit()
    else:
        print("Invalid choice, please try again.")

    try:
        while True:
            utils.world.tick()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    finally:
        utils.world_cleanup()
 

if __name__ == "__main__":
    main()