from omega_control import OmegaRobot, PreloadStatus

if __name__ == "__main__":

    # Parameters

    # Motion parameters
    target_axis = "z"
    target_force = -1.0  # N

    robot = OmegaRobot(verbose=True)
    filename = None
    try:
        robot.to_origin()
        robot.start_recording(freq=1000)
        robot.set_pos([0.0, 0.0, -0.052])

        # Start conservatively. Tune kp first, then add ki if necessary.
        result = robot.preload(
            target_axis=target_axis,
            target_force=target_force,
            kp=1e-3,
            ki=0.0,
            max_force=2.0,
            max_displacement=0.01,
            plot_results=True,
        )
        if result.status is not PreloadStatus.SETTLED:
            print(f"Preload stopped: {result.status.value}: {result.message}")

        robot.stop_recording()
        filename = robot.save_log(filename="trajectory")
    finally:
        robot.close()

    if filename is not None:
        robot.plot_log(filename=filename)
