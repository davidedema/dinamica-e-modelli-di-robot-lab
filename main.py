from cobot_controller import CobotController
import argparse

def main(args):
    cobot = CobotController(simulation=args.simulation, serial_port=args.serial_port)
    if args.mount_pen:
        cobot.mount_pen()
    for point in args.points:
        pos = [float(coord) for coord in point.split(',')]
        cobot.compute_trajectory(pos=pos)
        cobot.execute_trajectory(n_times=args.n_times)
    if args.remove_pen:
        cobot.remove_pen()
    
    if args.simulation:
        cobot.mc.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mount-pen", action="store_true", help="Mount the pen before executing the trajectory.")
    parser.add_argument("--remove-pen", action="store_true", help="Remove the pen after executing the trajectory.")
    parser.add_argument("--simulation", action="store_true", help="Run in simulation mode (no real robot).")
    parser.add_argument("--points", nargs='+', required=True, help="List of target points in the format x1,y1 x2,y2 ...")
    parser.add_argument("--n-times", type=int, default=4, help="Number of times to execute the trajectory.")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="Serial port for real robot connection (ignored in simulation mode).")
    args = parser.parse_args()
    main(args)