"""SwarmFlow 2D kinematic simulator (design §6.7, §12.3). Stdlib + PyYAML only; no ROS."""

from .backend import Sim2DBackend
from .runner import Sim2DResult, run

__all__ = ["Sim2DBackend", "Sim2DResult", "run"]
