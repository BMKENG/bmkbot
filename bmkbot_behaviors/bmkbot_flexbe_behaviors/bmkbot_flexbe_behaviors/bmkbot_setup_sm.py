#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 ggh-png
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define bmkbot_setup.

Created on Mon Jan 15 2024
@author: ggh-png
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from bmkbot_flexbe_states.bmkbot_set_waypoint_state import BMKBOTSetWaypointState
from bmkbot_flexbe_states.init_pose_state import InitPoseState
from bmkbot_flexbe_states.joystick_state import JoystickState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class bmkbot_setupSM(Behavior):
    """
    Define bmkbot_setup.

    bmkbot_setup

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'bmkbot_setup'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        BMKBOTSetWaypointState.initialize_ros(node)
        InitPoseState.initialize_ros(node)
        JoystickState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        waypoint = [                     [-1.8364053160843656, -9.688648169709293, 0.9999802312257832, 0.0062878579523772925],             [7.61449, -9.713, -0.7012, 0.71287],             [0.048, 9.937, 0.999, 0.3342],                     [-1.6023, 1.6007, 0.666, 0.74548],             [-3.6185, -4.063, -0.780, 0.6253],             [-3.35, 1.622, 0.7253, 0.688],              [0.351, -0.35, 0.96, 0.27],                         [4.83580242066541, -5.045123953309873, -0.9989866245556849, 0.04500804326827613]]
        # x:30 y:458
        _state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['waypoint'])
        _state_machine.userdata.waypoint = waypoint

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('init',
                                       InitPoseState(init_pose_topic="/initialpose", init_pose_cmd_topic="/init_pose_cmd", waypoint=[0.0,0.0,0.0,1.0]),
                                       transitions={'done': 'joy'},
                                       autonomy={'done': Autonomy.Off})

            # x:30 y:117
            OperatableStateMachine.add('joy',
                                       JoystickState(set_angular_vel=0.5, set_linear_vel=0.5, joystick_topic="/joy_cmd", cmd_vel_topic="/cmd_vel", set_vel_topic="/set_vel"),
                                       transitions={'done': 'setwaypoint'},
                                       autonomy={'done': Autonomy.Off})

            # x:136 y:217
            OperatableStateMachine.add('setwaypoint',
                                       BMKBOTSetWaypointState(amcl_pose_topic="/amcl_pose", navigator_topic="/navi_cmd"),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
