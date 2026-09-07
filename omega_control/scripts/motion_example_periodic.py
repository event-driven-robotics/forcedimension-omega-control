import time

from omega_control import OmegaRobot

if __name__ == "__main__":

    N = 50                # number of repetitions
    amplitude = 0.09      # 9 cm
    pause = 0.3           # seconds between moves
    vmax = 0.1

    robot = OmegaRobot(verbose=False)

    robot.to_origin()
    time.sleep(0.5)

    robot.start_recording(freq=1000)

    robot.set_pos_param(vmax, 1.0, 1.0)

    try:
        for i in range(N):
            # +Y
            robot.set_pos([0.0, amplitude, 0.0])

            # -Y
            robot.set_pos([0.0, -amplitude, 0.0])

    except KeyboardInterrupt:
        pass

    robot.stop_recording()

    filename = robot.save_log(filename="line_fast_robot")
    robot.close()

    robot.plot_log(filename=filename)
