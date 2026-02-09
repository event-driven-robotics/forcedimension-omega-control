import time

from omega_control import OmegaRobot

if __name__ == "__main__":
    robot = OmegaRobot(verbose=True)

    robot.to_origin()

    robot.start_recording(freq=10000)   # record at 10 kHz

    time.sleep(2)                      # collect data for 2 seconds
    robot.stop_recording()

    filename = robot.save_log()                   # saves to 'omega_log.h5'
    robot.close()

    time.sleep(2)

    robot.plot_log(filename=filename)
