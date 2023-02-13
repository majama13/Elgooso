import pandas as pd
import numpy as np
import time
import random
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity, SetEntityPose


class SARSALearner(Node):
	
	def __init__(self):
		super().__init__('sarsa_learner')
		
		self.world_name = 'world_demo'
		self.robot_name = '\"elgooso\"'
		self.model = '\"home/majama/ros2_ws/src/elgooso/urdf/elgooso.urdf\"'
		
		self.pub = self.create_publisher(Twist, '/model/elgooso/cmd_vel', 10)
		self.ear_sub = self.create_subscription(
			LaserScan, 
			'sonar', 
			self.record_state_callback, 
			10)
		self.odom_sub = self.create_subscription(
			Odometry, 
			'/model/elgooso/odometry', 
			self.record_position_callback, 
			10)
#		self.gz_delete_entity = self.create_client(
#			DeleteEntity, 
			#'/world/'+self.world_name+'/remove')
#			'/world/world_demo/remove')
#		while not self.gz_delete_entity.wait_for_service(timeout_sec=1.0):
#			self.get_logger().info('delete service not available, waiting again...')
#		self.req_del = DeleteEntity.Request()
#		self.gz_spawn_entity = self.create_client(
#			SpawnEntity, 
#			'/world/'+self.world_name+'/create')
#		while not self.gz_spawn_entity.wait_for_service(timeout_sec=1.0):
#			self.get_logger().info('spawn service not available, waiting again...')
#		self.req_spawn = SpawnEntity.Request()
#		self.gz_control_world = self.create_client(
#			ControlWorld, 
#			'/world/'+self.world_name+'/control')
#		while not self.gz_control_world.wait_for_service(timeout_sec=1.0):
#			self.get_logger().info('control service not available, waiting again...')
#		self.req_control = ControlWorld.Request()
			
		#NSTATES is the possible values of the lasers to the power of the number of laser pts. 
		#lets say we have 3 laser pts and each one reads from 0.0m - 0.5m.
		#this means we have 5 to the power of 3
		#number of laser pts 
		num_pts = 5
		#number of values for each laser point (0 to 5)
		val = 6	
		self.ACTIONS		= ['FL', 'F', 'FR', 'L', 'R', 'BL', 'B', 'BR']
		self.N_STATES 		= val**num_pts*11
		self.START			= -10.0
		self.TERMINAL		= 10.0
		self.EPSILON		= .9
		self.MAX_EPISODE	= 400 
		self.GAMMA			= .9
		self.ALPHA			= .01 #0.1
		self.FIRST			= True
		
		self.robot_vel = 1.0
		self.angular_vel = 5.0
		self.posx = 0
		self.oldx = 0
		self.q_table = self.build_q_table()	
		self.episode = 0
		self.step = 0
		self.end = False
		self.state = 0
		self.q_predict = 0
		self.last_state = self.START
		self.timer_period = 3.0  # seconds
		self.timer = self.create_timer(self.timer_period, self.learnSARSA)
		
	## Sensor Data records state and distance traveled
	def record_state_callback(self, msg):
		self.sensor_rdg = ''
		for i in msg.ranges:
			try:
				i = round(i)
			except:
				i = 5
			self.sensor_rdg += str(i)
		if self.posx < 0:
			self.sensor_rdg += str(round(0))
		else:
			self.sensor_rdg += str(round(self.posx))
		self.state = self.hex2dec(int(self.sensor_rdg))
		#self.get_logger().info(f'rdg --> state: {self.sensor_rdg} --> {self.state}')
		
		#the idea is to concatenate the rdgs to make a number. 
		#This number designates the state in the q table.
		#ex if we have [2, 5, 5] as rdgs, then our state is 255
		#thus, in a scan we have for n rdgs of x max range: x^n states
	
	
	
	def record_position_callback(self, msg):
		self.posx = msg.pose.pose.position.x
		
		#we want to record the x distance traveled

	def dec2hex(self, num):
		if num == 0:
			return 0
		ans = ""
		while num > 0:
			ans = str(num%6) + ans
			num /= 6
		return int(ans)

	def hex2dec(self, num):
		if num == 0:
			return 0
		num = str(num)
		ans = int(num[0])
		for i in num[1:]:
			ans *= 6
			ans += int(i)
		return ans

	## Q table gives a value to each state, action pair
	## Here, Q table and environment are reset
	def build_q_table(self):
		table = pd.DataFrame(
			np.zeros((self.N_STATES, len(self.ACTIONS))),
			columns=self.ACTIONS
		)
		return table
		
		
		
	def init_env(self):
		self.get_logger().info('')
		self.get_logger().info('initializing environment')
		
#		self.req_del.entity.name = 'elgooso'
#		self.req_del.entity.type = 2
#		self.future_del = self.gz_delete_entity.call_async(self.req_del)
		
#		self.req_spawn.entity_factory.name = 'elgooso'
#		self.req_spawn.entity_factory.sdf_filename = 'home/majama/ros2_ws/src/elgooso/urdf/elgooso.urdf'
#		self.req_spawn.entity_factory.pose.position.x = 0
#		self.req_spawn.entity_factory.pose.position.y = 0
#		self.req_spawn.entity_factory.pose.position.z = 0
#		self.req_spawn.entity_factory.pose.orientation.x = 0
#		self.req_spawn.entity_factory.pose.orientation.y = 0
#		self.req_spawn.entity_factory.pose.orientation.z = 0
#		self.req_spawn.entity_factory.pose.orientation.w = 1
#		self.future_spawn = self.gz_spawn_entity.call_async(self.req_spawn)
		
		
		
#		self.req_control.world_control.reset.all = True
#		self.future = self.gz_control_world.call_async(self.req_control)
#		rclpy.spin_until_future_complete(self, self.future)
#		self.get_logger().info(f'future: {self.future.result()}')
#		return self.future.result()
		
		#move model to start point
		subprocess.run(["gz service -s /world/"+self.world_name+"/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"elgooso\", position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}'"], shell = True)
		
		#delete and respawn model
		#subprocess.run(["gz service -s /world/"+self.world_name+"/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "elgooso", type: 2'"], shell = True)
		
		#ros2 command to reset world
		#subprocess.run(["ros2 service call /world/"+self.world_name+"/control ros_gz_interfaces/srv/ControlWorld '{world_control: {reset: {model_only: true}}}'"], shell = True)
		#gazebo command to reset world
		#subprocess.run(["gz service -s /world/"+self.world_name+"/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'reset: {model_only: true}'"], shell = True)
		#gazebo command to spawn model
		#subprocess.run(["gz service -s /world/"+self.world_name+"/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "+self.model+", name: "+self.robot_name+"'"], shell = True)

	## Choose action and get reward and end state
	def actor(self, observation, q_table):
	
		if np.random.uniform() < self.EPSILON:
			state_action = q_table.loc[observation, :]
			action = np.random.choice(state_action[state_action == np.max(state_action)].index)
		else:
			action = np.random.choice(self.ACTIONS)
		return action




	def get_feedback(self, state, action):
		#try a reward of 0. or of distance traveled between states
		posx = self.posx
		reward = (posx - self.oldx)/(self.robot_vel * self.timer_period)
		#self.get_logger().info(f'posx: {posx}')
		#self.get_logger().info(f'oldx: {self.oldx}')
		self.get_logger().info(f'delta_x: {posx - self.oldx}')
		self.oldx = posx
		self.end = False
		if state == self.TERMINAL:
			reward = 1.
			self.end = True
		elif self.posx < 0 :
			reward = -1.
			self.end = True
		return reward



	##Not sure if this one is needed...prob not
#	def playGame(self, q_table):
#	
#		while not end:
#		    act = actor(self.state, q_table)
#		    print("step::", i ," action ::", act)
#
#			if self.CRASH:
#				end = True
#			elif x > self.END:
#				end = True
#		print("==> Game Over <==")




	def learnSARSA(self):
		if self.episode < self.MAX_EPISODE:
			self.get_logger().info('')
			self.get_logger().info(f'Episode: {self.episode}')
			if not self.end:
				#get the time, use this time to access state and action at time t through tf
				state = self.state
				action = self.actor(state, self.q_table) 				#select the action corresponding to the current state
				reward = self.get_feedback(state, action)

				if state != self.TERMINAL:
					q_target = reward + self.GAMMA * self.q_table.loc[state, action]
				else:
					q_target = reward
					
				if self.last_state != self.START:
					self.q_table.loc[self.last_state, self.last_action] += self.ALPHA * (q_target - self.q_predict)
					#self.get_logger().info(f'q_val: {self.q_table.loc[self.last_state, self.last_action]}')

				self.q_predict = self.q_table.loc[state, action]		#q value of our state, action pair
				
				self.last_state = state
				self.last_action = action
				
				self.moveBot(action)
				self.get_logger().info(f'reward: {reward}')
				self.get_logger().info(f'state: {self.state}')

				self.step += 1
				if self.step > 30: # feel free to change this parameter
					#self.get_logger().info('END')
					self.end = True
			else:
				self.episode += 1
				self.last_state = self.START
				self.end = False
				self.goBot()
				self.init_env()
			
		else:
			self.get_logger().info(f'q_table: {self.q_table}')
			rclpy.shutdown()

	def goBot(self):
		msg = Twist()
		msg.linear.x = self.robot_vel
		msg.angular.z = 0.0
		self.pub.publish(msg)
	
	def stopBot(self):
		msg = Twist()
		msg.linear.x = 0.0
		msg.angular.z = 0.0
		self.pub.publish(msg)

	def moveBot(self, action):
		#self.get_logger().info('moving')
		msg = Twist()
		x = self.robot_vel
		z = self.angular_vel
		if action == 'FL':
			msg.linear.x = x
			msg.angular.z = z
		elif action == 'F':
			msg.linear.x = x
			msg.angular.z = 0.
		elif action == 'FR':
			msg.linear.x = x
			msg.angular.z = -z
		elif action == 'L':
			msg.linear.x = 0.
			msg.angular.z = z
		elif action == 'R':
			msg.linear.x = 0.
			msg.angular.z = -z
		elif action == 'BL':
			msg.linear.x = -x
			msg.angular.z = z
		elif action == 'B':
			msg.linear.x = -x
			msg.angular.z = 0.
		elif action == 'BR':
			msg.linear.x = -x
			msg.angular.z = -z
		self.pub.publish(msg)


def main(args=None):
	rclpy.init(args=args)

	node = SARSALearner()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
