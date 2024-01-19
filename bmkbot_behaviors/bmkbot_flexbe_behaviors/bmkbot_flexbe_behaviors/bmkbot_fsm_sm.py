#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2023 ggh-png
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
Define bmkbot_fsm.

Created on Tue Dec 05 2023
@author: ggh-png
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from bmkbot_flexbe_behaviors.bmkbot_setup_sm import bmkbot_setupSM
from bmkbot_flexbe_states.bmkbot_charging_state import BMKBOTChargingState
from bmkbot_flexbe_states.bmkbot_get_order_table_num_state import BMKBOTGetOrderTableNnumState
from bmkbot_flexbe_states.bmkbot_navi_to_pose_state import BMKBOTNavigateToPoseState
from bmkbot_flexbe_states.bmkbot_start_order_wait_state import BMKBOTStartOrder_waitState
from bmkbot_flexbe_states.tts_state import TTSState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class bmkbot_fsmSM(Behavior):
    """
    Define bmkbot_fsm.

    bmkbot_fsm

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'bmkbot_fsm'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        BMKBOTChargingState.initialize_ros(node)
        BMKBOTGetOrderTableNnumState.initialize_ros(node)
        BMKBOTNavigateToPoseState.initialize_ros(node)
        BMKBOTStartOrder_waitState.initialize_ros(node)
        TTSState.initialize_ros(node)
        self.add_behavior(bmkbot_setupSM, 'bmkbot_setup', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        go_home = 7
        go_charging = 8
        table = 0
        kitchen = 1
        battery = 100
        # x:1664 y:799, x:630 y:407
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.go_home = go_home
        _state_machine.userdata.go_charging = go_charging
        _state_machine.userdata.kitchen = kitchen
        _state_machine.userdata.table = table
        _state_machine.userdata.battery = battery

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:62 y:45
            OperatableStateMachine.add('bmkbot_setup',
                                       self.use_behavior(bmkbot_setupSM, 'bmkbot_setup'),
                                       transitions={'finished': 'get_order_tts'},
                                       autonomy={'finished': Autonomy.Inherit},
                                       remapping={'waypoint': 'waypoint'})

            # x:188 y:255
            OperatableStateMachine.add('charjing_finish',
                                       TTSState(text="충전이 완료되었습니다.", service_topic='/bmkbot/io'),
                                       transitions={'done': 'go_to_home_state', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:460 y:492
            OperatableStateMachine.add('charjing_start',
                                       TTSState(text="충전을 시작하겠습니다.", service_topic='/bmkbot/io'),
                                       transitions={'done': 'battery_charging_state', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:721 y:666
            OperatableStateMachine.add('delivery_success',
                                       TTSState(text="배달이 완료되었습니다. 맛있게 드세요.", service_topic='/bmkbot/io'),
                                       transitions={'done': 'go_to_home_wait_state', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:95 y:167
            OperatableStateMachine.add('get_order_tts',
                                       TTSState(text="안녕하세요 서빙로봇 한에리 입니다. 주문을 해주시면 빠르게 배달해드리겠습니다. ", service_topic='/bmkbot/io'),
                                       transitions={'done': 'get_order_wait_state', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:293 y:155
            OperatableStateMachine.add('get_order_wait_state',
                                       BMKBOTGetOrderTableNnumState(order_table_topic="/order_table", battery_topic="/battery"),
                                       transitions={'serving': 'go_to_kitchen_wait_state', 'charging': 'go_to_charging_station'},
                                       autonomy={'serving': Autonomy.Off, 'charging': Autonomy.Off},
                                       remapping={'battery': 'battery', 'table_num': 'table_num', 'order_table_num': 'order_table_num'})

            # x:630 y:231
            OperatableStateMachine.add('go_to_charging_station',
                                       BMKBOTNavigateToPoseState(timeout=120.0, navi_to_pos_topic="/navigate_to_pose", emergency_topic="/emergency_cmd", battery_topic="/battery", amcl_pose_topic="/amcl_pose", navi_progress_topic="/remaining_waypoint", arrived_table_topic="/arrived_table", table_num=0, table_or_kitchen=0),
                                       transitions={'failed': 'failed', 'done': 'charjing_start'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint', 'table_num': 'go_charging', 'order_table_num': 'order_table_num', 'table_or_kitchen': 'kitchen', 'battery': 'battery'})

            # x:77 y:479
            OperatableStateMachine.add('go_to_home_state',
                                       BMKBOTNavigateToPoseState(timeout=120.0, navi_to_pos_topic="/navigate_to_pose", emergency_topic="/emergency_cmd", battery_topic="/battery", amcl_pose_topic="/amcl_pose", navi_progress_topic="/remaining_waypoint", arrived_table_topic="/arrived_table", table_num=0, table_or_kitchen=0),
                                       transitions={'failed': 'failed', 'done': 'get_order_tts'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint', 'table_num': 'go_home', 'order_table_num': 'order_table_num', 'table_or_kitchen': 'kitchen', 'battery': 'battery'})

            # x:348 y:619
            OperatableStateMachine.add('go_to_home_wait_state',
                                       BMKBOTStartOrder_waitState(order_table_topic="/order_table", start_navi_topic="/start_nav", arrived_table_topic="/arrived_table"),
                                       transitions={'done': 'go_to_home_state'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'table_num': 'table_num', 'order_table_num': 'order_table_num'})

            # x:1191 y:202
            OperatableStateMachine.add('go_to_kitchen_state',
                                       BMKBOTNavigateToPoseState(timeout=120.0, navi_to_pos_topic="/navigate_to_pose", emergency_topic="/emergency_cmd", battery_topic="/battery", amcl_pose_topic="/amcl_pose", navi_progress_topic="/remaining_waypoint", arrived_table_topic="/arrived_table", table_num=0, table_or_kitchen=0),
                                       transitions={'failed': 'failed', 'done': 'go_to_table_wait_state'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint', 'table_num': 'table_num', 'order_table_num': 'order_table_num', 'table_or_kitchen': 'kitchen', 'battery': 'battery'})

            # x:611 y:107
            OperatableStateMachine.add('go_to_kitchen_wait_state',
                                       BMKBOTStartOrder_waitState(order_table_topic="/order_table", start_navi_topic="/start_nav", arrived_table_topic="/arrived_table"),
                                       transitions={'done': 'go_to_kitchen_state'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'table_num': 'table_num', 'order_table_num': 'order_table_num'})

            # x:1311 y:621
            OperatableStateMachine.add('go_to_table_state',
                                       BMKBOTNavigateToPoseState(timeout=120.0, navi_to_pos_topic="/navigate_to_pose", emergency_topic="/emergency_cmd", battery_topic="/battery", amcl_pose_topic="/amcl_pose", navi_progress_topic="/remaining_waypoint", arrived_table_topic="/arrived_table", table_num=0, table_or_kitchen=0),
                                       transitions={'failed': 'failed', 'done': 'delivery_success'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                       remapping={'waypoint': 'waypoint', 'table_num': 'table_num', 'order_table_num': 'order_table_num', 'table_or_kitchen': 'table', 'battery': 'battery'})

            # x:1514 y:284
            OperatableStateMachine.add('go_to_table_wait_state',
                                       BMKBOTStartOrder_waitState(order_table_topic="/order_table", start_navi_topic="/start_nav", arrived_table_topic="/arrived_table"),
                                       transitions={'done': 'go_to_table_state'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'table_num': 'table_num', 'order_table_num': 'order_table_num'})

            # x:265 y:347
            OperatableStateMachine.add('battery_charging_state',
                                       BMKBOTChargingState(battery_topic="/battery"),
                                       transitions={'done': 'charjing_finish'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'battery': 'battery'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
