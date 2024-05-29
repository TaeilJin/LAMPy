import torch
import numpy as np
import theseus as th

from PyMBS import MBS
from VisualData import Vis

robot = MBS.from_urdf_file("Data/panda_no_gripper.urdf")