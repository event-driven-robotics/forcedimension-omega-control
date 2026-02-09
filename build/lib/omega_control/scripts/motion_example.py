from omega_control import OmegaRobot

if __name__ == "__main__":

    robot = OmegaRobot(verbose=False)

    robot.to_origin()

    robot.start_recording(freq=1000)   # record at 1 kHz

    # Send position command (cartesian)
    robot.set_pos([0.05, 0.0, 0.0])
    robot.to_origin()
    robot.set_pos([0.0, 0.05, 0.0])
    robot.to_origin()
    robot.set_pos([0.0, 0.0, 0.05])
    robot.to_origin()

    robot.stop_recording()

    filename = robot.save_log(filename="axis_finding")
    robot.close()

    robot.plot_log(filename=filename)
