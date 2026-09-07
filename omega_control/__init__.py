"""Omega Control - Python framework for controlling Force Dimension OmegaX haptic devices.

This package provides a comprehensive Python framework for controlling Force Dimension 
OmegaX haptic devices. It includes high-level abstractions for device initialization, 
motion control, trajectory generation, force feedback, and real-time data recording.

Classes:
    OmegaRobot: Main interface class for controlling and reading state from 
                Force Dimension robots.

Example:
    Basic usage of the robot control interface::

        from omega_control import OmegaRobot
        
        robot = OmegaRobot(verbose=True)
        robot.to_origin()
        robot.start_recording(freq=1000)
        # ... perform robot operations ...
        robot.stop_recording()
        robot.save_log("session_data.h5")
        robot.close()

Authors:
    Simon Mueller-Cleve (simon.mullercleve@iit.it)
    Quentin Halbach (halbach.quentin@iit.it)

Institution:
    Istituto Italiano di Tecnologia (IIT)
"""

from ._sdk_config import configure_forcedimension_sdk

configure_forcedimension_sdk()

from .omega_control import OmegaRobot, PreloadResult, PreloadSample
from .pid import PIDConfig, PIDController, PIDOutput, PreloadStatus

__version__ = "1.0.0"
__author__ = "Simon Mueller-Cleve, Quentin Halbach"
__email__ = "simon.mullercleve@iit.it, halbach.quentin@iit.it"
__license__ = "GPL-3.0"
__copyright__ = "Copyright (c) 2025 Istituto Italiano di Tecnologia (IIT)"

__all__ = [
    "OmegaRobot",
    "PIDConfig",
    "PIDController",
    "PIDOutput",
    "PreloadResult",
    "PreloadSample",
    "PreloadStatus",
]
