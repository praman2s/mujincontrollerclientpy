# -*- coding: utf-8 -*-
# Copyright (C) 2013-2015 MUJIN Inc.
# Mujin controller client for bin picking task

import json

# logging
import logging
log = logging.getLogger(__name__)

# mujin imports
from . import controllerclientbase
from . import ugettext as _

class RobotControllerClient(controllerclientbase.ControllerClientBase):
    """mujin controller client for robot controller tasks
    """
    tasktype = 'laserworker'
    _robotControllerUri = None  # URI of the robot controller, e.g. tcp://192.168.13.201:7000?densowavearmgroup=5
    _robotDeviceIOUri = None  # the device io uri (usually PLC used in the robot bridge)
