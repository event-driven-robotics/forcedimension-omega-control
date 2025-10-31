from omega_robot import OmegaRobot
import time
import math

if __name__ == "__main__":

    # Parameters

    # Motion parameters
    target_axis = "z"
    target_force = -1.0 # N


    robot = OmegaRobot(verbose=True)

    robot.to_origin()

    robot.start_recording(freq=1000)   # record at 1 kHz

    robot.set_pos([0.0, 0.0, -0.052])


    # Preload Control
    robot.preload(target_axis=target_axis, target_force=target_force)

    robot.stop_recording()

    filename = robot.save_log(filename="trajectory")                   # saves to 'omega_log.h5'
    robot.close()

    # time.sleep(2)

    robot.plot_log(filename=filename)


