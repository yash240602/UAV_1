"""UAV Strategic Deconfliction System.

A system that verifies drone flight safety in shared airspace by checking
for conflicts in both space and time against other drone flight paths.
"""

from .models import Waypoint, Mission, ConflictResult
from .core import DeconflictionSystem

__all__ = ['Waypoint', 'Mission', 'ConflictResult', 'DeconflictionSystem'] 