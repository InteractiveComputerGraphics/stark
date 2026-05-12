# -*- coding: utf-8 -*-
import numpy as np

# Import all
from .pystark import *
from .serialize import *

ZERO = np.array([0.0, 0.0, 0.0])
ONES = np.array([1.0, 1.0, 1.0])
UNITX = np.array([1.0, 0.0, 0.0])
UNITY = np.array([0.0, 1.0, 0.0])
UNITZ = np.array([0.0, 0.0, 1.0])
MM = 0.001