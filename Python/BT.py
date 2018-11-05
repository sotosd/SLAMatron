import serial
import time
import math
import numpy as np
import sys

import scipy.io
import scipy.stats
import matplotlib.pyplot as plt
from tqdm import tqdm
pi=math.pi

class Map():
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize+2 # Add extra cells for the borders
        self.ysize = ysize+2
        self.grid_size = grid_size # save this off for future use
        self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero

        self.alpha = 10.0# The assumed thickness of obstacles
        self.beta = 12.674*pi / 180 #5*np.pi/180.0 #12.674 The assumed width of the laser beam
        self.z_max = 400.0 # The max reading from the laser

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize*self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
                                         np.tile(np.arange(0, self.ysize*self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])

        # Log-Probabilities to add or remove from the map 
        self.l_occ = np.log(0.85/0.15)
        self.l_free = np.log(0.15/0.85)

    def update_map(self, pose, z):

        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[0, :, :] -= pose[0]+50 # A matrix of all the x coordinates of the cell
        dx[1, :, :] -= pose[1]+50 # A matrix of all the y coordinates of the cell
        theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell

        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for z_i in z:
            r = z_i[0] # range measured
            b = z_i[1] # bearing measured

            # Calculate which cells are measured free or occupied, so we know which cells to update
            # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
            free_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (dist_to_grid < (r - self.alpha/2.0))
            occ_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (np.abs(dist_to_grid - r) <= self.alpha/2.0)

            # Adjust the cells appropriately
            self.log_prob_map[occ_mask] += self.l_occ
            self.log_prob_map[free_mask] += self.l_free


if __name__ == '__main__':
	pi=math.pi
	bearing_angles=[-1.5708, -1.3962, -1.2217, -1.0472, -0.8727, -0.6981, -0.5236, -0.3490, -0.1745, 0, 0.1745, 0.3490, 0.5236, 0.6981, 0.8727, 1.0472, 1.2217, 1.3962, 1.5708]
	meas_rec = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	state=np.array(np.zeros(3))
	prv_state=np.array(np.zeros(3))

	print("Start")
	print("initializing...")
	port="/dev/rfcomm0" #This will be different for various devices and on windows it will probably be a COM port.
	bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
	print("Connected")
	bluetooth.flushInput() #This gives the bluetooth a little kick


	grid_size = 1.0
	map = Map(int(1000/grid_size), int(1000/grid_size), grid_size)
	plt.ion() # enable real-time plotting
	plt.figure(1) # create a plot

	while(True):
		try:
			try:
				data = bluetooth.readline()
				# print data
				sep = data.split()
				# print sep
				if "end"==sep[0]: 
					# pass
					bluetooth.flushInput()
					print "hello"
					break
				else:
					my_float_list= [float(x) for x in sep[4:23]]
					my_float_list=[ y if y <= 400.0 else 400.0 for y in my_float_list]
					meas_to_send = np.array([my_float_list, bearing_angles])
					# meas_to_send = np.array([[x / 100 for x in my_float_list],bearing_angles])
					
					# print meas_to_send.T

					my_float_list= [float(x) for x in sep[1:3]]
					AHeading=my_float_list[0]*pi/180
					state[0]=(my_float_list[1]*math.sin(AHeading))+prv_state[0]#my_float_list[0]*pi/180) # A cell that contains the x coordinate of the robot
					state[1]=(my_float_list[1]*math.cos(AHeading))+prv_state[1]#my_float_list[0]*pi/180) # A cell that contains the y coordinate of the robot
					state[2]=AHeading+1.5708

					prv_state=state
					# print state

					bluetooth.flushInput()

					map.update_map(state, meas_to_send.T) # update the map

					# Real-Time Plotting 
					# (comment out these next lines to make it run super fast, matplotlib is painfully slow)
					plt.clf()
					pose = state
					circle = plt.Circle((pose[1]+50, pose[0]+50), radius=7.0, fc='y')
					plt.gca().add_patch(circle)
					arrow = pose[0:2] + np.array([3.5, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])], [-np.sin(pose[2]), np.cos(pose[2])]]))
					plt.plot([pose[1], arrow[1]], [pose[0], arrow[0]])
					plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys')
					plt.pause(0.005)
					time.sleep(0.1) #A pause between bursts
			except Exception, e:
					pass 
		except serial.SerialException:
			print "Communication Lost"
			print "Attempting regaining communication."
			try: 
				bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
				#print "Connection Regained"
			except Exception, e:
				print "error open serial port: " + str(e)
				
	bluetooth.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
	print("Done")
	# Final Plotting
	plt.ioff()
	plt.clf()
	plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys') # This is probability
	plt.imshow(map.log_prob_map, 'Greys') # log probabilities (looks really cool)
plt.show()
