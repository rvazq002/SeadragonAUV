'''
		State machine template using the smach library
		Written by: Rogelio Vazquez lol
		Another irrelevant comment

		To Run this code: 
		0. Save this file to any directory/Folder on your desktop
		1. Go to directory where you have this file in terminal
		2. From terminal run $ sudo chmod u+x [yourfilename.py] 
		3. Open new terminal and run $ roscore
		4. From terminal where you have your sm file run $ python filename.py
		5. In a new terminal, run rosrun smach_viewer smach_viewer.py 
		6. A gui should pop up showing you your state machine

		7. At any time during state A or state B type the following command
		8. In a new terminal run $ rostopic pub /reset std_msgs/Bool "data: True"
					    command ... topic ... var type ... data ...
		
		Be careful with your indentations! Indentations indicate a new
		block of code in python.  You've got no semicolon but you've 
		got indentations. 
'''
#!/usr/bin/env python

##------------------------------- IMPORTS ---------------------------------------##
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16
##----------------------------- END IMPORTS -------------------------------------##


##------------------------- STATE DEFINITIONS -----------------------------------##

# Define State Init
class init(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['start','track1','reset1'])
		
		'''	 
			Publishers, Subscribers & Local variables go here
		'''

		# Local variable example
		self.timer = 0
	
	def execute(self, userdata):
	
		'''
			State Actions go here
		'''

		# Check if condition for transitioning to next state was met here.
		self.timer += 1
		if self.reset == True:
            return 'reset' 
        else self.timer > 20000:
			self.timer = 0
			return 'start'
		else:
			return 'track1'

# Define State Track1
# Purpose: The state tracks the location of the first bouy 
# which contains the image of the vampire it is the single flat 
# bouy. 
class Track1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['track1','touch1', 'reset'])
		
		# Publishers, Subscribers
		self.reset_subscriber = rospy.Subscriber('/close', Bool, self.reset_callback) 
		
		# Local Variables
		self.timer = 0
		self.reset = False

	def reset_callback(self,msg):
		self.reset = msg.data

	def execute(self, userdata):
			
		self.timer += 1
		if self.reset == True:
			return 'reset'
		elif self.timer > 20000:
			self.timer = 0
			return 'track1'
		else:
			return 'touch1'	

# Define State Touch1
# Purpose: The state uses a preset timer to get the 
# sub to touch the flat bouy which contains the image 
# of a vampire.
# NOTE: The timer was set based on the distance the 
# sub has to travel to touch the bouy 
class Touch1(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['touch1','turn','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False

        def reset_callback(self,msg):
                self.reset = msg.data

        def execute(self, userdata):

                self.timer += 1
                if self.reset == True:
                        return 'reset'
                elif self.timer > 20000:
                        self.timer = 0
                        return 'touch1'
                else:
                        return 'turn'	

# Define State Turn
# Purpose: The state resets the sub to face the the second 
# bouy which rotates at a certain speed. 
class Turn(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['turn','wait2','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False

        def reset_callback(self,msg):
                self.reset = msg.data

        def execute(self, userdata):

                self.timer += 1
                if self.reset == True:
                        return 'reset'
                elif self.timer > 20000:
                        self.timer = 0
                        return 'turn'
                else:
                        return 'wait2'

# Define State Wait2
# Purpose: The state waits until the computer vision finds the 
# predefined image from the 3 possible choosen images that are located 
# on the rotating bouy.  
class Wait2(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['wait2','touch2','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False

        def reset_callback(self,msg):
                self.reset = msg.data

        def execute(self, userdata):

                self.timer += 1
                if self.reset == True:
                        return 'reset'
                elif self.timer > 20000:
                        self.timer = 0
                        return 'wait2'
                else:
                        return 'touch2'

# Define State Touch2
# Purpose: The state uses a timer to move a certain distance to touch the bouy. 
# The time was set depending on the distance of the sub from the bouy. 
class Touch2(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['touch2','setyaw','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False

        def reset_callback(self,msg):
                self.reset = msg.data

        def execute(self, userdata):

                self.timer += 1
                if self.reset == True:
                        return 'reset'
                elif self.timer > 20000:
                        self.timer = 0
                        return 'touch2'
                else:
                        return 'setyaw'

# Define State SetYaw
# The state prepares the sub for the next competiton task.
# The sub has to be orientated to point in the direction of the 
# "stake through the heart" task. 
class SetYaw(smach.State): 
        def __init__(self):
                smach.State.__init__(self, outcomes=['setyaw','done','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False

        def reset_callback(self,msg):
                self.reset = msg.data

        def execute(self, userdata):

                self.timer += 1
                if self.reset == True:
                        return 'reset'
                elif self.timer > 20000:
                        self.timer = 0
                        return 'setyaw'
                else:
                        return 'done'

# Define State Done
# Purpose: The state sets sends a signal to the main state machine letting it know that the 
# the task is complete. 
class Done(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['done','init','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False

        def reset_callback(self,msg):
                self.reset = msg.data

        def execute(self, userdata):

                self.timer += 1
                if self.reset == True:
                        return 'reset'
                elif self.timer > 20000:
                        self.timer = 0
                        return 'done'
                else:
                        return 'init'

# Define State Reset
class reset(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['restart','init'])
		
		self.timer = 0
	
	def execute(self, userdata):
		self.timer +=1
		if self.timer > 20000:
			return 'restart'
		else: 
			return 'init'

##-------------------------- END STATE DEFINITIONS ------------------------------------##


def main():
	# Initialize node with desired node name - ideally task name
	rospy.init_node('my_task_statemachine')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['task_complete'])

	# Create and start introspection server - fancy way of saying view gui feature
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Open SMACH container
	with sm:
		# Add states to the container
		smach.StateMachine.add('INIT', init(), 
					transitions={'wait':'INIT','start':'A'})
		smach.StateMachine.add('A', A(),
					transitions={'nottime':'A','timeup':'B','reset':'RESET'})
		smach.StateMachine.add('B', B(),
					transitions={'tracking':'B','task_complete':'task_complete','reset':'RESET'})	
		smach.StateMachine.add('RESET',reset(),
					transitions ={'wait':'RESET','restart':'INIT'})
	# Execute State Machine
	outcome = sm.execute()
	
	# Spin node - fancy way of saying run code in a loop
	rospy.spin()
	sis.stop()
	

if __name__ == '__main__':
	main()
