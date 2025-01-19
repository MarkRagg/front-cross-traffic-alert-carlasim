import os
from test.utils.utilities import Utilities
from test.sensors.radar_sensor import RadarSensor

def show_menu():
    print("\nFRONT CROSS TRAFFIC ALERT CARLA SIMULATOR TEST:")
    print("\n1. Lot of cars test")
    print("2. Left side test")
    print("3. Right side test")
    print("4. Cobined test")
    print("5. Little town left side test")
    print("6. Little town right side test")
    print("7. Little town bike test")
    print("0. Exit\n")

def main():
    show_menu()
    while True:
        choice = input("Enter the test number: ")
        if choice == '1':
            Utilities("Town10HD")
            ego_vehicle = Utilities.spawn_ego_vehicle()
            Utilities.spawn_vehicles(30)
            break
        elif choice == '2':
            Utilities("Town10HD")
            ego_vehicle = Utilities.spawn_ego_vehicle()
            Utilities.accelerate_vehicle(Utilities.spawn_left_vehicle())
            break
        elif choice == '3':
            Utilities("Town10HD")
            ego_vehicle = Utilities.spawn_ego_vehicle()
            Utilities.accelerate_vehicle(Utilities.spawn_right_vehicle())
            break
        elif choice == '4':
            Utilities("Town10HD")
            ego_vehicle = Utilities.spawn_ego_vehicle()
            Utilities.accelerate_vehicle(Utilities.spawn_left_vehicle(), 0.4)
            Utilities.accelerate_vehicle(Utilities.spawn_right_vehicle(), 0.3)
            break
        elif choice == '5':
            Utilities("Town01")
            ego_vehicle = Utilities.spawn_ego_vehicle()
            Utilities.accelerate_vehicle(ego_vehicle, 0.15)
            Utilities.accelerate_vehicle(Utilities.spawn_left_vehicle(), 0.3)
            break
        elif choice == '6':
            Utilities("Town01")
            ego_vehicle = Utilities.spawn_ego_vehicle()
            Utilities.accelerate_vehicle(ego_vehicle, 0.15)
            Utilities.accelerate_vehicle(Utilities.spawn_right_vehicle(), 0.3)
            break
        elif choice == '7':
            Utilities("Town01")
            ego_vehicle = Utilities.spawn_ego_vehicle(static=True)
            Utilities.accelerate_vehicle(Utilities.spawn_bike(), 0.6)
            break
        elif choice == '0':
            Utilities.world_cleanup()
            exit()
        else:
            print("Invalid choice, please try again.")

    os.system('cls')
    print("Open the CARLA server window!")

    radar_right = RadarSensor(ego_vehicle, "right", y=1, pitch=5, yaw=50)
    radar_left = RadarSensor(ego_vehicle, "left", y=-1, pitch=5, yaw=-50)

    Utilities.move_spectator_to(ego_vehicle.get_transform(), Utilities.spectator)

    try:
        while True:
            Utilities.world.tick()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    finally:
        Utilities.world_cleanup()
 

if __name__ == "__main__":
    main()