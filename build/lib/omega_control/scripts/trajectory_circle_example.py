import math
import time

from omega_control import OmegaRobot

if __name__ == "__main__":

    # Motion parameters
    radius = 0.04        # 3 cm
    frequency = 0.25     # Hz
    duration = 60.0       # seconds

    robot = OmegaRobot(verbose=False)

    # Move to starting point on the circle
    robot.set_pos([radius, 0.0, 0.0], tracking=False)
    time.sleep(0.5)

    robot.start_recording(freq=1000)   # 1 kHz logging

    t0 = time.time()
    omega = 2 * math.pi * frequency

    try:
        while (time.time() - t0) < duration:
            t = time.time() - t0

            x = radius * math.cos(omega * t)
            y = radius * math.sin(omega * t)
            z = 0.0

            robot.set_pos([x, y, z], tracking=True)
            time.sleep(0.001)  # ~1 kHz

    except KeyboardInterrupt:
        pass

    robot.stop_recording()

    filename = robot.save_log(filename="circle_slow")
    robot.close()

    robot.plot_log(filename=filename)
