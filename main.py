from cobot_controller import CobotController

MOUNT_PEN = False
REMOVE_PEN = False
SIMULATION_MODE = True

def main():
    cobot = CobotController(simulation=SIMULATION_MODE)
    if MOUNT_PEN:
        cobot.mount_pen()
    cobot.compute_trajectory(pos=[0.25, 0.0])
    cobot.execute_trajectory(n_times=4)
    cobot.compute_trajectory(pos=[0.22, 0.04])
    cobot.execute_trajectory(n_times=4)
    cobot.compute_trajectory(pos=[0.2, -0.06])
    cobot.execute_trajectory(n_times=4)
    if REMOVE_PEN:
        cobot.remove_pen()
    
    if SIMULATION_MODE:
        cobot.mc.close()

if __name__ == "__main__":
    
    main()