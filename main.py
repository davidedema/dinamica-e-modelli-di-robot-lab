from cobot_controller import CobotController

def main():
    cobot = CobotController(simulation=False)

    cobot.open_gripper()
    input("Insert the marker in the gripper and press Enter to continue...")
    cobot.close_gripper()

    cobot.compute_trajectory(pos=[0.22, 0.0])
    inp = input("Press Enter to execute the trajectory...")
    cobot.execute_trajectory(n_times=2)

    input("Press enter and take the marker out of the gripper to finish...")
    cobot.open_gripper()


if __name__ == "__main__":
    main()