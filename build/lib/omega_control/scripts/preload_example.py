from omega_control import OmegaRobot

if __name__ == "__main__":

    # Parameters

    # Motion parameters
    target_axis = "z"
    target_force = -1.0  # N

    robot = OmegaRobot(verbose=True)

    robot.to_origin()

    robot.start_recording(freq=1000)   # record at 1 kHz

    robot.set_pos([0.0, 0.0, -0.052])

    # Preload Control
    robot.preload(target_axis=target_axis, target_force=target_force)

    robot.stop_recording()

    # saves to 'omega_log.h5'
    filename = robot.save_log(filename="trajectory")
    robot.close()

    # time.sleep(2)

    robot.plot_log(filename=filename)
