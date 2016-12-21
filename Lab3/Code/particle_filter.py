#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap, get_polar_line

#===============================================================================
class ParticleFilter(object):
    '''
    Class to hold the whole particle filter.
    
    p_wei: weights of particles in array of shape (N,)
    p_ang: angle in radians of each particle with respect of world axis, shape (N,)
    p_xy : position in the world frame of the particles, shape (2,N)
    '''
    
    #===========================================================================
    def __init__(self, room_map, num, odom_lin_sigma, odom_ang_sigma, 
                 meas_rng_noise, meas_ang_noise,x_init,y_init,theta_init):
        '''
        Initializes the particle filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.map = room_map
        self.num = num
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        
        # Map
        map_xmin = np.min(self.map[:, 0])
        map_xmax = np.max(self.map[:, 0])
        map_ymin = np.min(self.map[:, 1])
        map_ymax = np.max(self.map[:, 1])
        
        # Particle initialization arround starting point
        self.p_wei = 1.0 / num * np.ones(num)
        self.p_ang =2 * np.pi * np.random.rand(num)
        self.p_xy  = np.vstack(( x_init+ 1*np.random.rand(num)-0.5,
                                 y_init+ 1*np.random.rand(num)-0.5 ))
        #Flags for resampling                         
        self.moving=False
        self.n_eff=0 #Initialize Efficent number as 0
    
    #===========================================================================
    def predict(self, odom):
        '''
        Moves particles with the given odometry.
        odom: incremental odometry [delta_x delta_y delta_yaw] in the vehicle frame
        '''
        #Check if we have moved from previous reading.
        if odom[0]==0 and odom[1]==0 and odom[2]==0:
            self.moving=False
        else:
            for i in range(self.num):
	        self.p_xy[0,i]+=(odom[0]+np.random.randn()*self.odom_lin_sigma)*np.cos(self.p_ang[i])-(odom[1]+np.random.randn()*self.odom_lin_sigma)*np.sin(self.p_ang[i])
		self.p_xy[1,i]+=(odom[0]+np.random.randn()*self.odom_lin_sigma)*np.sin(self.p_ang[i])+(odom[1]+np.random.randn()*self.odom_lin_sigma)*np.cos(self.p_ang[i])
	    
	    self.p_ang+=odom[2]*np.ones(self.num)+np.random.randn(self.num)*self.odom_ang_sigma         
	    self.p_ang=angle_wrap(self.p_ang)

            #Update flag for resampling
            self.moving=True     
    
    #===========================================================================
    def weight(self, lines):
        '''
        Look for the lines seen from the robot and compare them to the given map.
        Lines expressed as [x1 y1 x2 y2].
        '''
        # TODO: code here!!
        # Constant values for all weightings
        val_rng = 1.0 / (self.meas_rng_noise * np.sqrt(2 * np.pi))
        val_ang = 1.0 / (self.meas_ang_noise * np.sqrt(2 * np.pi))
        
        # Loop over particles
        for i in range(self.num):
            
            # Transform map lines to local frame and to [range theta]
            map_polar=np.zeros((self.map.shape[0],2))
	    d1=np.zeros(map_polar.shape[0])
            for j in range(self.map.shape[0]):
                map_polar[j,:]=get_polar_line( self.map[j,:],np.array([self.p_xy[0,i],self.p_xy[1,i],self.p_ang[i]]) )
		d1[j]=np.sqrt( (self.map[j,2]-self.map[j,0])**2+(self.map[j,3]-self.map[j,1])**2 )
		             
            # Transform measured lines to [range theta] and weight them
            lines_polar=np.zeros((lines.shape[0],2))
            for j in range(lines.shape[0]):
                # Weight them
		modify=np.ones(map_polar.shape[0])
		d2=np.sqrt( (lines[j,2]-lines[j,0])**2+(lines[j,3]-lines[j,1])**2 )
    	        lines_polar[j,:]=get_polar_line(lines[j,:])
		for c in range(self.map.shape[0]):
			if d1[c]<d2:
				modify[c]=0
		
		WR=val_rng*np.exp(-( (lines_polar[j,0]-map_polar[:,0])**2 )/( (2*self.meas_rng_noise)**2 ) ) 
		Wang=val_ang*np.exp(-( (lines_polar[j,1]-map_polar[:,1])**2 )/( (2*self.meas_ang_noise)**2 ) )	
		self.p_wei[i]*=np.max(Wang*WR*modify)

        # Normalize weights
        self.p_wei /= np.sum(self.p_wei)
        # TODO: Compute efficient number
        self.n_eff=1.0/np.sum(self.p_wei**2)
        
    #===========================================================================
    def resample(self):
        '''
        Systematic resampling of the particles.
        '''
        # TODO: code here!!
        # Look for particles to replicate    
	r=np.random.uniform(0.0,1.0)/self.num
	c=self.p_wei[0]
	j=0
	xy=np.zeros((2,self.num))
	ang=self.p_ang
        for i in range(self.num):
            u=r+i/(self.num*1.0)
	    while (u>c):
   	        j+=1
	        c=c+self.p_wei[j]

            xy[:,i]=self.p_xy[:,j]
	    ang[i]=self.p_ang[j]

        # Pick chosen particles
        self.p_xy=xy
        self.p_ang=ang
        self.p_wei = 1.0 / self.num * np.ones(self.num)
    #===========================================================================
    def get_mean_particle(self):
        '''
        Gets mean particle.
        '''
        # Weighted mean
        weig = np.vstack((self.p_wei, self.p_wei))
        mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
        
        ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
                          np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
                          
        return np.array([mean[0], mean[1], ang])
