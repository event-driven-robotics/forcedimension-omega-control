import math
import time

from omega_control import OmegaRobot

if __name__ == "__main__":

    # Parameters

    # Motion parameters
    amplitude = 0.03   # 3 cm front/back
    frequency = 0.25   # 0.25 Hz (1 cycle every 4 seconds)
    duration = 5.0    # run for 5 seconds
    # t0 = time.time()

    robot = OmegaRobot(verbose=False)

    robot.to_origin()

    robot.start_recording(freq=1000)   # record at 1 kHz

    # Trajectory Control
    t0 = time.time()
    try:
        while (time.time() - t0) < duration:
            t = time.time() - t0
            x = amplitude * math.sin(2 * math.pi * frequency * t)
            y = 1.5*amplitude * math.sin(2 * math.pi * frequency * t)
            z = 0.5*amplitude * math.sin(2 * math.pi * frequency * t)

            # Send position command (cartesian)
            robot.set_pos([x, y, z], tracking=True)
            time.sleep(0.001)  # ~1 kHz control loop

    except KeyboardInterrupt:
        pass

    robot.stop_recording()

    filename = robot.save_log(filename="trajectory")
    robot.close()

    robot.plot_log(filename=filename)
