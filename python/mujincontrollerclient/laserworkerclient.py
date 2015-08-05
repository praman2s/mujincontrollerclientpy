# -*- coding: utf-8 -*-
# Copyright (C) 2013-2015 MUJIN Inc.
# Mujin controller client for itl planning/laserworker task

import json

# logging
import logging
log = logging.getLogger(__name__)

# mujin imports
from . import controllerclientbase
from . import ugettext as _

class LaserWorkerClient(controllerclientbase.ControllerClientBase):
    """mujin controller client for robot controller tasks
    """
    tasktype = 'laserworker'
    _robotControllerUri = None  # URI of the robot controller, e.g. tcp://192.168.13.201:7000?densowavearmgroup=5
    _robotDeviceIOUri = None  # the device io uri (usually PLC used in the robot bridge)

    def __init__(self, controllerurl, controllerusername, controllerpassword, robotControllerUri, scenepk, robotname, robotspeed, regionname, targetname, toolname, laserworkerzmqport=None, laserworkerheartbeatport=None, laserworkerheartbeattimeout=None, usewebapi=True, initializezmq=False, ctx=None, robotDeviceIOUri=None,  robotaccelmult=None):
        
        """logs into the mujin controller, initializes binpicking task, and sets up parameters
        :param controllerurl: url of the mujin controller, e.g. http://controller14
        :param controllerusername: username of the mujin controller, e.g. testuser
        :param controllerpassword: password of the mujin controller
        :param laserworkerzmqport: port of the laserworker task's zmq server, e.g. 7110
        :param laserworkerheartbeatport: port of the laserworker task's zmq server's heartbeat publisher, e.g. 7111
        :param laserworkerheartbeattimeout: seconds until reinitializing laserworker task's zmq server if no hearbeat is received, e.g. 7
        :param scenepk: pk of the bin picking task scene, e.g. irex2013.mujin.dae
        :param robotname: name of the robot, e.g. VP-5243I
        :param robotspeed: speed of the robot, e.g. 0.4
        :param regionname: name of the bin, e.g. container1
        :param targetname: name of the target, e.g. plasticnut-center
        :param toolname: name of the manipulator, e.g. 2BaseZ
        :param usewebapi: whether to use webapi for controller commands
        :param robotaccelmult: optional multiplier for forcing the acceleration
        """
        super(LaserWorkerClient, self).__init__(controllerurl, controllerusername, controllerpassword, binpickingzmqport,beatport, laserworkerbeattimeout, self.tasktype, scenepk, initializezmq, usewebapi, ctx)
        
        # robot controller
        self._robotControllerUri = robotControllerUri
        self._robotDeviceIOUri = robotDeviceIOUri
        
        # bin picking task
        self.robotname = robotname
        self.robotspeed = robotspeed
        self.robotaccelmult = robotaccelmult
        self.regionname = regionname
        self.targetname = targetname
        self.toolname = toolname
        
        
        def SetRobotControllerUri(self, robotControllerUri):
            self._robotControllerUri = robotControllerUri
        
        def SetRobotDeviceIOUri(self, robotDeviceIOUri):
            self._robotDeviceIOUri = robotDeviceIOUri
    
        def GetRobotControllerUri(self):
            return self._robotControllerUri
        
        def GetRobotDeviceIOUri(self):
            return self._robotDeviceIOUri
    
        def ReloadModule(self, timeout=10, **kwargs):
            return self.ExecuteCommand({'command': 'ReloadModule'}, timeout=timeout, **kwargs)



    #########################
    # robot commands
    #########################

    def ExecuteRobotCommand(self, taskparameters, robotspeed=None, usewebapi=None, timeout=10):
        """wrapper to ExecuteCommand with robot info set up in taskparameters

        executes a command on the task.

        :return: a dictionary that contains:
        - robottype: robot type,string
        - currentjointvalues: current joint values, DOF floats
        - elapsedtime: elapsed time in seconds, float
        - numpoints: the number of points, int
        - error: optional error info, dictionary
          - desc: error message, string
          - type: error type, string
          - errorcode: error code, string
        """
        robotname = self.robotname
        taskparameters['robot'] = robotname
        taskparameters['robotControllerUri'] = self._robotControllerUri
        taskparameters['robotDeviceIOUri'] = self._robotDeviceIOUri
        taskparameters['gripperControlInfo'] = self.gripperControlInfo 
        
        if taskparameters.get('speed', None) is None:
            # taskparameters does not have robotspeed, so set the global speed
            if robotspeed is not None:
                taskparameters['robotspeed'] = robotspeed
            elif self.robotspeed is not None:
                taskparameters['robotspeed'] = float(self.robotspeed)
            
        if taskparameters.get('robotaccelmult', None) is None and self.robotaccelmult is not None:
            taskparameters['robotaccelmult'] = float(self.robotaccelmult)
        return self.ExecuteCommand(taskparameters, usewebapi, timeout=timeout)
    
    def ExecuteTrajectory(self, trajectoryxml, robotspeed=None, timeout=10, **kwargs):
        """Executes a trajectory on the robot from a serialized Mujin Trajectory XML file.
        """
        taskparameters = {'command': 'ExecuteTrajectory',
                          'trajectory': trajectoryxml,
                          }
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, robotspeed=robotspeed, timeout=timeout)
    
    def MoveJoints(self, jointvalues, jointindices=None, robotspeed=None, execute=1, startvalues=None, timeout=10, **kwargs):
        """moves the robot to desired joint angles specified in jointvalues
        :param jointvalues: list of joint values
        :param jointindices: list of corresponding joint indices, default is range(len(jointvalues))
        :param robotspeed: value in [0,1] of the percentage of robot speed to move at
        :param envclearance: environment clearance in milimeter
        """
        if jointindices is None:
            jointindices = range(len(jointvalues))
            log.warn(u'no jointindices specified, moving joints with default jointindices: %s', jointindices)
        taskparameters = {'command': 'MoveJoints',
                          'goaljoints': list(jointvalues),
                          'jointindices': list(jointindices),
                          'envclearance': self.envclearance,
                          'execute': execute,
                          }
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, robotspeed=robotspeed, timeout=timeout)
    
    def GetJointValues(self, timeout=10, **kwargs):
        """gets the current robot joint values
        :return: current joint values in a json dictionary with
        - currentjointvalues: [0,0,0,0,0,0]
        """
        taskparameters = {'command': 'GetJointValues',
                          }
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    
    def GetManipulatorTransformInRobotFrame(self, timeout=10):
        """gets the transform of the manipulator in robot frame
        :return: current transform of the manipulator in robot frame in a json dictionary, e.g. {'translation': [100,200,300], 'rotationmat': [[1,0,0],[0,1,0],[0,0,1]], 'quaternion': [1,0,0,0]}
        """
        taskparameters = {'command': 'GetManipTransformToRobot',
                          }
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    
    def PickAndPlace(self, goaltype, goals, targetnamepattern=None, approachoffset=30, departoffsetdir=[0, 0, 50], destdepartoffsetdir=[0, 0, 30], deletetarget=0, debuglevel=4, movetodestination=1, freeinc=[0.08], worksteplength=None, densowavearmgroup=5, regionname=None, cameranames=None, envclearance=15, toolname=None, robotspeed=0.5, timeout=1000, **kwargs):
        """picks up an object with the targetnamepattern and places it down at one of the goals. First computes the entire plan from robot moving to a grasp and then moving to its destination, then runs it on the real robot. Task finishes once the real robot is at the destination.

        :param desttargetname: The destination target name where the destination goal ikparams come from
        :param destikparamnames: A list of lists of ikparam names for the destinations of the target. Only destikparamnames[0] is looked at and tells the system to place the part in any of the ikparams in destikparamnames[0]

        :param targetnamepattern: regular expression describing the name of the object, default is '%s_\d+'%(self.targetname). See https://docs.python.org/2/library/re.html
        :param approachoffset: distance in milimeter to move straight to the grasp point, e.g. 30 mm
        :param departoffsetdir: the direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50]
        :param destdepartoffsetdir: the direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system.
        :param leaveoffsetintool: If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.
        :param deletetarget: whether to delete target after pick and place is done
        :param toolname: name of the manipulator
        :param regionname: name of the region of the objects
        :param cameranames: the names of the cameras to avoid occlusions with the robot, list of strings
        :param envclearance: environment clearance in milimeter

        Low level planning parameters:
        :param debuglevel: sets debug level of the task
        :param movetodestination: planning parameter
        :param freeinc: planning parameter
        :param worksteplength: planning parameter
        :param densowavearmgroup: planning parameter
        :param graspsetname: the name of the grasp set belong to the target objects to use for the target. Grasp sets are a list of ikparams

        Manual Destination Specification (deprecated)
        :param goaltype: type of the goal, e.g. translationdirection5d or transform6d
        :param goals: flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
        """
        if worksteplength is None:
            worksteplength = 0.01
        if toolname is None:
            toolname = self.toolname
        if targetnamepattern is None:
            targetnamepattern = '%s_\d+' % (self.targetname)
        if regionname is None:
            regionname = self.regionname
        if robotspeed is None:
            robotspeed = self.robotspeed
        taskparameters = {'command': 'PickAndPlace',
                          'toolname': toolname,
                          'goaltype': goaltype,
                          'envclearance': envclearance,
                          'movetodestination': movetodestination,
                          'goals': goals,
                          'approachoffset': approachoffset,
                          'departoffsetdir': departoffsetdir,
                          'destdepartoffsetdir': destdepartoffsetdir,
                          'freeinc': freeinc,
                          'worksteplength': worksteplength,
                          'targetnamepattern': targetnamepattern,
                          'containername': regionname,
                          'deletetarget': deletetarget,
                          'robotspeed': robotspeed,
                          'debuglevel': debuglevel,
                          }
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, robotspeed=robotspeed, timeout=timeout)
    
    def StartPickAndPlaceThread(self, goaltype=None, goals=None, targetnamepattern=None, approachoffset=30, departoffsetdir=[0, 0, 50], destdepartoffsetdir=[0, 0, 30], deletetarget=0, debuglevel=4, movetodestination=1, worksteplength=None, regionname=None, envclearance=15, toolname=None, robotspeed=None, timeout=10, **kwargs):
        """Start a background loop to continuously pick up objects with the targetnamepattern and place them down at the goals. The loop will check new objects arriving in and move the robot as soon as it finds a feasible grasp. The thread can be quit with StopPickPlaceThread.

        :param desttargetname: The destination target name where the destination goal ikparams come from
        :param destikparamnames: A list of lists of ikparam names for the ordered destinations of the target. destikparamnames[0] is where the first picked up part goes, desttargetname[1] is where the second picked up target goes.
        :param cycledests: When finished cycling through all destikparamnames, will delete all the targets and start from the first index again doing this for cycledests times. By default it is 1.

        :param targetnamepattern: regular expression describing the name of the object, default is '%s_\d+'%(self.targetname). See https://docs.python.org/2/library/re.html
        :param approachoffset: distance in milimeter to move straight to the grasp point, e.g. 30 mm
        :param departoffsetdir: the direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50]
        :param destdepartoffsetdir: the direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system.
        :param leaveoffsetintool: If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.
        :param deletetarget: whether to delete target after pick and place is done
        :param toolname: name of the manipulator
        :param regionname: name of the region of the objects
        :param cameranames: the names of the cameras to avoid occlusions with the robot, list of strings
        :param envclearance: environment clearance in milimeter
        Low level planning parameters:
        :param debuglevel: sets debug level of the task
        :param movetodestination: planning parameter
        :param worksteplength: planning parameter
        :param densowavearmgroup: robot parameters
        :param graspsetname: the name of the grasp set belong to the target objects to use for the target. Grasp sets are a list of ikparams

        :param goaltype: type of the goal, e.g. translationdirection5d
        :param goals: flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]

        :param useworkspaceplanner: If 1 is set, will try the workspace planner for moving the hand straight. If 2 is set, will try the RRT for moving straight. Can set 3 for trying both.
        """
        if worksteplength is None:
            worksteplength = 0.01
        if toolname is None:
            toolname = self.toolname
        if targetnamepattern is None:
            targetnamepattern = '%s_\d+' % (self.targetname)
        if regionname is None:
            regionname = self.regionname
        if robotspeed is None:
            robotspeed = self.robotspeed
        taskparameters = {'command': 'StartPickAndPlaceThread',
                          'toolname': toolname,
                          'envclearance': envclearance,
                          'movetodestination': movetodestination,
                          'approachoffset': approachoffset,
                          'departoffsetdir': departoffsetdir,
                          'destdepartoffsetdir': destdepartoffsetdir,
                          'worksteplength': worksteplength,
                          'targetnamepattern': targetnamepattern,
                          'containername': regionname,
                          'deletetarget': deletetarget,
                          'robotspeed': robotspeed,
                          'debuglevel': debuglevel,
                          }
        if goals is not None:
            taskparameters['orderedgoals'] = goals
            taskparameters['goaltype'] = goaltype
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, robotspeed=robotspeed, timeout=timeout)
    
    def StopPickPlaceThread(self, timeout=10, **kwargs):
        """stops the pick and place thread started with StartPickAndPlaceThread
        :params resetstate: if True, then reset the order state variables
        """
        taskparameters = {'command': 'StopPickPlaceThread',
                          }
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    
    def GetPickPlaceStatus(self, timeout=10, **kwargs):
        """gets the status of the pick and place thread
        :return: status (0: not running, 1: no error, 2: error) of the pick and place thread in a json dictionary, e.g. {'status': 2, 'error': 'an error happened'}
        """
        taskparameters = {'command': 'GetPickPlaceStatus',
                          }
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    

    def ComputeIK(self, timeout=10, **kwargs):
        """
        :param toolname: tool name, string
        :param limit: number of solutions to return, int
        :param iktype: grasp (but basically the just the ikparam), string
        :param quaternion: grasp (but basically the just the ikparam) quaternion in world cooordinates, float array
        :param translation: grasp (but basically the just the ikparam) translation in world cooordinates in mm, float array
        :param direction: grasp (but basically the just the ikparam) direction in world cooordinates, float array
        :param angle: grasp (but basically the just the ikparam) angle in world cooordinates, float
        :param freeincvalue: float, the discretization of the free joints of the robot when computing ik.
        :param filteroptions: OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked, int
        :param preshape: If the tool has fingers after the end effector, specify their values. The gripper DOFs come from **gripper_dof_pks** field from the tool., float array

        :return: A dictionary of:
        - solutions: array of IK solutions (each of which is an array of DOF values), sorted by minimum travel distance and truncated to match the limit
        """
        taskparameters = {'command': 'ComputeIK',
                          }
        taskparameters.update(kwargs)
        if 'toolname' not in taskparameters:
            taskparameters['toolname'] = self.toolname
        if 'envclearance' not in taskparameters:
            taskparameters['envclearance'] = self.envclearance
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    
    def ComputeIKFromParameters(self, timeout=10, **kwargs):
        """
        :param toolname: tool name, string
        :param limit: number of solutions to return, int
        :param ikparamnames: the ikparameter names, also contains information about the grasp like the preshape
        :param targetname: the target object name that the ikparamnames belong to
        :param freeincvalue: float, the discretization of the free joints of the robot when computing ik.
        :param filteroptions: OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked, int

        :return: A dictionary of:
        - solutions: array of IK solutions (each of which is an array of DOF values), sorted by minimum travel distance and truncated to match the limit
        """
        taskparameters = {'command': 'ComputeIKFromParameters',
                          }
        taskparameters.update(kwargs)
        if 'toolname' not in taskparameters:
            taskparameters['toolname'] = self.toolname
        if 'envclearance' not in taskparameters:
            taskparameters['envclearance'] = self.envclearance
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    

    def ShutdownRobotBridge(self, timeout=10, **kwargs):
        taskparameters = {'command': 'ShutdownRobotBridge',
                          }
        taskparameters.update(kwargs)
        return self.ExecuteRobotCommand(taskparameters, timeout=timeout)
    
    ####################
    # scene commands
    ####################
    
    def GetTransform(self, targetname, unit='mm', timeout=10):
        """gets the transform of the CAD object
        :param targetname: name of the object
        :param unit: unit of the result translation
        :return: transform of the object in a json dictionary, e.g. {'translation': [100,200,300], 'rotationmat': [[1,0,0],[0,1,0],[0,0,1]], 'quaternion': [1,0,0,0]}
        """
        taskparameters = {'command': 'GetTransform',
                          'targetname': targetname,
                          'unit': unit,
                          }
        return self.ExecuteCommand(taskparameters, timeout=timeout)
    

    def SaveScene(self, timeout=10, **kwargs):
        """saves the current scene to file
        :param filename: e.g. /tmp/testscene.mujin.dae, if not specified, it will be saved with an auto-generated filename
        :param preserveexternalrefs: If True, any bodies currently that are being externally referenced from the environment will be saved as external references.
        :param externalref: If '*', then will save each of the objects as externally referencing their original filename. Otherwise will force saving specific bodies as external references
        :param saveclone: If 1, will save the scenes for all the cloned environments
        :return: the actual filename the scene is saved to in a json dictionary, e.g. {'filename': '2013-11-01-17-10-00-UTC.dae'}
        """
        taskparameters = {'command': 'SaveScene',
                          }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)
    
    def GetTrajectoryLog(self, timeout=10, **kwargs):
        """Gets the recent trajectories executed on the binpicking server. The internal server keeps trajectories around for 10 minutes before clearing them.

        :param startindex: int, start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last element.
        :param num: int, number of trajectories from startindex to return. If 0 will return all the trajectories starting from startindex
        :param includejointvalues: bool, If True will include timedjointvalues, if False will just give back the trajectories. Defautl is False

        :return:

        total: 10
        trajectories: [
        {
        "timestarted": 12345215
        "name": "movingtodest",
        "numpoints": 100,
        "duration": 0.8,
        "timedjointvalues": [0, 0, 0, .....]
        },
        { ... }
        ]

        Where timedjointvalues is a list joint values and the trajectory time. For a 3DOF robot sampled at 0.008s, this is
        [J1, J2, J3, 0, J1, J2, J3, 0.008, J1, J2, J3, 0.016, ...]

        """
        taskparameters = {'command': 'GetTrajectoryLog',
                          }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

        
