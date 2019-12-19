#!/usr/bin/env python

import rospy
import smach
import smach_ros
import signal
import sys
import os
import time
import threading
from std_msgs.msg import UInt8, Bool

    # 0 -> manual (defalut) -> blue
    # 1 -> autonomy -> red
    # 2 -> idle -> green

class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['light_up2'],
                                   output_keys=['current_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')
        userdata.current_state = 2
        time.sleep(1.2)
        return 'light_up2'

class MANUAL(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['light_up0'],
                                   output_keys=['current_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MANUAL')
        time.sleep(1.2)
        userdata.current_state = 0
        return 'light_up0'

class AUTONOMOUS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['light_up1'],
                                   output_keys=['current_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AUTONOMOUS')
        userdata.current_state = 1
        time.sleep(1.2)
        return 'light_up1'


class ROVER_STATE(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['0','1', '2'],
                             input_keys=['r_state_input'],
                             output_keys=['r_state_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ROVER_STATE')
        time.sleep(1.2)
        print(userdata.r_state_input)
        if userdata.r_state_input == 0:
            userdata.r_state_output = 0
            return '0'
        if userdata.r_state_input == 1:
            userdata.r_state_output = 1
            return '1'
        if userdata.r_state_input == 2:
            userdata.r_state_output = 2
            return '2'

class Dioda(smach.State):

    def callback(self, state):
         self.x = state.data

    def __init__(self):
        smach.State.__init__(self, outcomes=['change'],
                                   output_keys = ['r_state_input'],
                                   input_keys = ['current_state'])
        self.Subsriber = rospy.Subscriber("/lazik_state", UInt8, self.callback)
        self.x = 0

    def execute(self, userdata):
        cos = 0
        while self.x == userdata.current_state:
            cos = 0
        userdata.r_state_input = self.x
        return 'change'



sm = smach.StateMachine(outcomes=[])

sm.userdata.r_state_input = 0
sm.userdata.current_state = 0

def main():
    rospy.init_node('smach_example_state_machine')

    with sm:

        smach.StateMachine.add('ROVER_STATE', ROVER_STATE(),
                               transitions={'0':'MANUAL', '1':'AUTONOMOUS', '2':'IDLE'}
                             # remapping={'r_state_input': 'sm_dupa'}
                              )

        smach.StateMachine.add('IDLE', IDLE(),
                                   transitions={
                                                'light_up2':'Dioda2'})

        smach.StateMachine.add('MANUAL', MANUAL(),
                               transitions={'light_up0':'Dioda0'})

        smach.StateMachine.add('AUTONOMOUS', AUTONOMOUS(), transitions={'light_up1':'Dioda1'})

        smach.StateMachine.add('Dioda0', Dioda(), transitions={'change':'ROVER_STATE'})
        smach.StateMachine.add('Dioda1', Dioda(), transitions={'change':'ROVER_STATE'})
        smach.StateMachine.add('Dioda2', Dioda(), transitions={'change':'ROVER_STATE'})


    #rospy.Subscriber("/lazik_state", UInt8, callback)

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
