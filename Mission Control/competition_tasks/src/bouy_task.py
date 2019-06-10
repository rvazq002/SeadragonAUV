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
		smach.State.__init__(self, outcomes=['start','wait'])
		
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
		if self.timer > 20000:
			self.timer = 0
			return 'start'
		else:
			return 'wait'

# Define State A
class A(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['timeup','nottime','reset'])
		
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
			return 'timeup'
		else:
			return 'nottime'			

# Define State B
class B(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['tracking','task_complete','reset'])

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
                        return 'task_complete'
                else:
                        return 'tracking'

# Define State Reset
class reset(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['restart','wait'])
		
		self.timer = 0
	
	def execute(self, userdata):
		self.timer +=1
		if self.timer > 20000:
			return 'restart'
		else: 
			return 'wait'

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
