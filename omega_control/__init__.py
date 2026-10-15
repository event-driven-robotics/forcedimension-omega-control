"""Python control library for Force Dimension omega.x devices.

Importing this package does not require the separately installed vendor SDK.
"""
from .pid import PIDConfig, PIDController, PIDOutput, PreloadStatus

__version__ = "1.0.0"
__author__ = "Simon Mueller-Cleve, Quentin Halbach"
__email__ = "simon.mullercleve@iit.it, halbach.quentin@iit.it"
__license__ = "GPL-3.0"
__copyright__ = "Copyright (c) 2025 Istituto Italiano di Tecnologia (IIT)"

_HARDWARE_API = {"OmegaRobot", "PreloadResult", "PreloadSample"}

def __getattr__(name: str):
    if name not in _HARDWARE_API:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from ._sdk_config import configure_forcedimension_sdk
    configure_forcedimension_sdk()
    from . import omega_control as implementation
    result = getattr(implementation, name)
    globals()[name] = result
    return result

__all__ = ["OmegaRobot", "PIDConfig", "PIDController", "PIDOutput",
           "PreloadResult", "PreloadSample", "PreloadStatus"]
