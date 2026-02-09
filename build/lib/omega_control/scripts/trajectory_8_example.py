import math
import time

from omega_control import OmegaRobot

if __name__ == "__main__":

    # Motion parameters
    amplitude = 0.03      # 3c cm
    frequency = 0.25      # Hz
    duration = 25.0        # seconds

    robot = OmegaRobot(verbose=False)

    robot.to_origin()
    time.sleep(0.5)       # let it settle

    robot.start_recording(freq=1000)   # 1 kHz logging

    t0 = time.time()

    try:
        while (time.time() - t0) < duration:
            t = time.time() - t0

            omega = 2 * math.pi * frequency

            # Figure-8 trajectory
            x = amplitude * math.sin(2 * omega * t)
            y = amplitude * math.sin(omega * t)
            z = 0.0   # keep constant height

            robot.set_pos([x, y, z], tracking=True)

            time.sleep(0.001)  # ~1 kHz

    except KeyboardInterrupt:
        pass

    robot.stop_recording()

    filename = robot.save_log(filename="figure8")
    robot.close()

    robot.plot_log(filename=filename)
