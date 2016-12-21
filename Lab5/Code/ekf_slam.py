#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
import probabilistic_lib.functions as funcs
from probabilistic_lib.functions import angle_wrap, comp, state_inv, state_inv_jacobian,compInv
import scipy.linalg
import rospy

#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0,y0,theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = 0.08 # TODO chose your own value
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.xk = np.array([x0,y0,theta0]) # Position
        self.Pk = np.zeros((3,3)) # Uncertainty
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 5
    
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.xk.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range theta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk):
        
        '''
        Predicts the position of the robot according to the previous position and the odometry measurements. It also updates the uncertainty of the position
        '''
        #TODO: Program this function
        # - Update self.xk and self.Pk using uk and self.Qk
        
        # Compound robot with odometry
	a=funcs.comp(self.xk[0:3],uk)
	self.xk = np.append(a ,self.xk[3:])
		
	# Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        c1=-np.sin(self.xk[2])*uk[0]-np.cos(self.xk[2])*uk[1]
	c2=np.cos(self.xk[2])*uk[0]-np.sin(self.xk[2])*uk[1]
	Ak=np.array([ [1, 0, c1],[0, 1, c2],[0 ,0, 1]])
	a1=np.cos(self.xk[2])
	a2=np.sin(self.xk[2])
	Wk=np.array([ [a1, -a2, 0],[a2, a1, 0],[0 ,0, 1]])
	
        # Prepare the F_k and G_k matrix for the new uncertainty computation
	n=self.get_number_of_features_in_map()
	F_k=np.eye(3+2*n)
	G_k=np.zeros((3+2*n,3))
	F_k[0:3,0:3]=Ak
	G_k[0:3,0:3]=Wk
        # Compute uncertainty
        self.Pk=np.dot(np.dot(F_k,self.Pk),F_k.T)+np.dot(np.dot(G_k,self.Qk),G_k.T)
        # Update the class variables	
    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        '''
    	
        #TODO: Program this function											
        # fore each sensed line do:
        #   1- Transform the sened line to polar
        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance
        #   3- Data association
	Innovk_List   = list()
        H_k_List      = list()
        Rk_List       = list()
	idx_not_associated =np.array([])
        for i in range(lines.shape[0]):
		z = self.get_polar_line(lines[i,:],np.array([0, 0 ,0]))  # polar line in robot frame
		# Variables for finding minimum
		minD = 1e9
		minidx = -1
		for idx in range(self.get_number_of_features_in_map()):
			(D,v,h,H)=self.lineDist(z,idx)
 			if np.sqrt(D) < minD:
		            minidx = idx
		            minh = h
		            minH = H
		            minv = v
		            minD = D

           	# Minimum distance below threshold
           	if minD < self.chi_thres:
               	    # Append results
	   	    self.featureObservedN[minidx]+=1   # optional part
		    if self.featureObservedN[minidx]> self.min_observations:      # optional part
		        H_k_List.append(minH)
		        Innovk_List.append(minv)
		        Rk_List.append(self.Rk)
		else:
		    idx_not_associated =np.append(idx_not_associated,i)
			        
        return Innovk_List, H_k_List, Rk_List, idx_not_associated 
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, Rk_List) :
        '''
        Updates the position of the robot according to the given position
        and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        #TODO: Program this function
        if len(Innovk_List)<1: 
            return
            
	# Compose list of matrices as single matrices
        m = len(H_k_List)
	n=self.get_number_of_features_in_map()
        H = np.zeros((2*m, 2*n+3))
        v = np.zeros((2*m))
        R = np.zeros((2*m, 2*m))
        for i in range(m):
            H[2*i:2*i+2, :] = H_k_List[i]
            v[2*i:2*i+2] = Innovk_List[i]
            R[2*i:2*i+2, 2*i:2*i+2] = Rk_List[i]
	
	S=np.dot(H,np.dot(self.Pk,H.T))+R
        
	# Kalman Gain
        k =np.dot(np.dot(self.Pk,H.T),np.linalg.inv(S))

        # Update Position
        self.xk+=np.dot(k,v)

        # Update Uncertainty
        a=np.dot(k,H)
	I=np.eye(a.shape[0])
	b=I-a
	c=np.dot(b,np.dot(self.Pk,b.T))
	self.Pk=c+np.dot(k,np.dot(R,k.T))    
    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kineckt sensor and the
        indexes that have not been associated, augment the state vector to 
        introduce the new features
        '''
        # If no features to add to the map exit function
        if idx.size<1:
            return
	n=self.get_number_of_features_in_map()
	F_k=np.eye((3+n*2))
	G_k=np.zeros((3+n*2,2))
	for i in range(idx.shape[0]):
		z = self.get_polar_line(lines[idx[i],:],np.array([0, 0 ,0]))
		(z_m, H_tf, j2)=self.tfPolarLine(self.xk[0:3],z)
		self.xk=np.append(self.xk,z_m)
		self.featureObservedN=np.append(self.featureObservedN,np.array([0]))
		F_k=np.vstack(( F_k,np.hstack(( H_tf,np.zeros((2,2*n))  ))  ))
		G_k=np.vstack(( G_k,j2 ))
	self.Pk=np.dot(np.dot(F_k,self.Pk),F_k.T)+np.dot(np.dot(G_k,self.Rk),G_k.T)	

	
    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        # Auxiliar computations
        sqrt2 = x_x**2+x_y**2
        sqrt = np.sqrt(sqrt2)
        atan2 = np.arctan2(x_y,x_x)
        sin = np.sin(atan2 - phi)
        cos = np.cos(atan2 - phi)
        
        # Compute the new rho
        rho = line[0] + sqrt* cos  
        if rho <0:
            rho = -rho
            phi = angle_wrap(phi+pi)         
        
        # Allocate jacobians
        #H_tf = np.zeros((2,3))
        #H_line = np.eye(2)

        # TODO: Evaluate jacobian respect to transformation
	a=-x_x*np.sin(phi+x_ang)+x_y*np.cos(phi+x_ang)
	H_tf=np.array([[np.cos(phi+x_ang),np.sin(phi+x_ang),a],[0,0,1]])

        # TODO: Evaluate jacobian respect to line
	H_line=np.array([[1,-x_x*np.sin(phi+x_ang)+x_y*np.cos(phi+x_ang)],[0,1]])               
        return np.array([rho,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
        '''
        Given a line and an index of the state vector, it computes the
        distance between both lines
        '''        
        # TODO program this function
                
        # Transform the map line into robot frame and compute jacobians
	(state_inv,state_inv_jacobian)=compInv(self.xk[0:3])
	(h,H_position,H_line)=self.tfPolarLine(state_inv,self.xk[3+2*idx:3+2*idx+2])
	n=self.get_number_of_features_in_map()
        
        # Allocate overall jacobian
	        
	H = np.zeros((2,3+2*n))
        
        # Concatenate position jacobians and place them into the position
        hg1=np.dot(H_position,state_inv_jacobian)
	H[:,0:3]=hg1
	
        # Place the position of the jacobian with respec to the line in its
        # position
        H[:,3+2*idx:3+2*idx+2]=H_line
        # Calculate innovation
        v =z-h  
        
        # Calculate innovation uncertainty
        S = self.Rk+np.dot(np.dot(H,self.Pk),H.T)
  
        # Calculate mahalanobis distance
        D = np.dot(np.dot(v.T,np.linalg.inv(S)),v) 
        
        return D,v,h,H
