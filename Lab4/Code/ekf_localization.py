#!/usr/bin/python
# -*- coding: utf-8 -*-

"""EKF class that Implements prediction and update."""

import numpy as np
import probabilistic_lib.functions as funcs
# use: comp, get_polar_line, get_map


# ==============================================================================
class EKF(object):
    """Class to hold the whole Extended Kalman Filter (EKF)."""

    # ==========================================================================
    def __init__(self, xinit, odom_lin_sigma, odom_ang_sigma, meas_rng_noise,
                 meas_ang_noise):
        """
        Initialize the EKF filter.

        Input:
          room_map : a nx4 array of lines in the form [x1 y1 x2 y2]
          xinit    : initial position
          odom_lin_sigma: odometry linear noise
          odom_ang_sigma: odometry angular noise
          meas_rng_noise: measurement linear noise
          meas_ang_noise: measurement angular noise
        """
        # Map with initial displacement
        self.map = funcs.get_dataset3_map(xinit[0], xinit[1], xinit[2])

        # Prediction noise
        self.Qk = np.array([[odom_lin_sigma**2, 0, 0],
                            [0, odom_lin_sigma**2, 0],
                            [0, 0, odom_ang_sigma**2]])

        # Measurement noise
        self.Rk = np.array([[meas_rng_noise**2, 0],
                            [0, meas_ang_noise**2]])

        # Pose initialization
        self.xk = np.zeros(3)
        self.Pk = 0.2*0.2*np.eye(3)  # initial uncertainty of robot state

    # ==========================================================================
    def predict(self, uk):
        """
        Implement the prediction equations of the EKF.

        Saves the new robot state and uncertainty.

        Input:
          uk : numpy array [shape (3,) with (dx, dy, dtheta)]
        """
        # TODO: Program this function
        ################################################################
        # Check website for numpy help:
        #         http://wiki.scipy.org/NumPy_for_Matlab_Users
        # 1. Update self.xk and self.Pk using uk and self.Qk
        #        can use comp() from funtions.py
	
	self.xk=funcs.comp(self.xk,uk)
	c1=-np.sin(self.xk[2])*uk[0]-np.cos(self.xk[2])*uk[1]
	c2=np.cos(self.xk[2])*uk[0]-np.sin(self.xk[2])*uk[1]
	Ak=np.array([ [1, 0, c1],[0, 1, c2],[0 ,0, 1]])
	a1=np.cos(self.xk[2])
	a2=np.sin(self.xk[2])
	Wk=np.array([ [a1, -a2, 0],[a2, a1, 0],[0 ,0, 1]])
	self.Pk=np.dot(np.dot(Ak,self.Pk),Ak.T)+np.dot(np.dot(Wk,self.Qk),Wk.T)
    # ==========================================================================
    def data_association(self, lines):
        """
        Look for closest correspondences.

        The correspondences are between the provided measured lines and the map
        known a priori.

        Input:
          lines : nx4 matrix with a segment in each row as [x1 y1 x2 y2]
        Return:
          Hk_list : list of 2x3 matrices (jacobian)
          Yk_list : list of 2x1 matrices (innovation)
          Sk_list : list of 2x2 matrices (innovation uncertainty)
          Rk_list : list of 2x2 matrices (measurement uncertainty)
        """
        # TODO: Program this function
        ################################################################
        # 1. Map lines (self.map) to polar robot frame: get_polar_line
        # 2. Sensed lines (lines) to polar robot frame: get_polar_line
        # 3. Data association

        # Init variables
        chi_thres = 0.103  # chi square 2DoF 95% confidence
        associd = list()
        Hk_list = list()
        Vk_list = list()
        Sk_list = list()
        Rk_list = list()

        #return Hk_list, Vk_list, Sk_list, Rk_list  # TODO: delete this line

        # For each obseved line
        print('\n-------- Associations --------')
        for i in range(0, lines.shape[0]):

            # Get the polar line representation in robot frame
            z = funcs.get_polar_line(lines[i,:])

            # Variables for finding minimum
            minD = 1e9
            minj = -1

            # For each line in the known map
            for j in range(0, self.map.shape[0]):

                # Compute matrices
                h = funcs.get_polar_line(self.map[j,:],self.xk) #robot
		h2=funcs.get_polar_line(self.map[j,:]) #world frame
	        H = self.jacobianH(h2)
                v =z-h
                S =np.dot(np.dot(H,self.Pk),H.T)+self.Rk

                # Mahalanobis distance
                D = np.dot(np.dot(v.T,np.linalg.inv(S)),v)

                # Optional: Check if observed line is longer than map
                ########################################################
		map_dis=np.sqrt((self.map[j,2]-self.map[j,0])**2+(self.map[j,3]-self.map[j,1])**2)
		line_dis=np.sqrt((lines[i,2]-lines[i,0])**2+(lines[i,3]-lines[i,1])**2)		
		if map_dis<line_dis:                
		    islonger = True
		else:
	            islonger = False
		
                # Check if the observed line is the one with smallest
                # mahalanobis distance
                if np.sqrt(D) < minD and not islonger:
                    minj = j
                    minz = z
                    minh = h
                    minH = H
                    minv = v
                    minS = S
                    minD = D

            # Minimum distance below threshold
            if minD < chi_thres:
                print("\t{} -> {}".format(minz, minh))
                # Append results
                associd.append([i, minj])
                Hk_list.append(minH)
                Vk_list.append(minv)
                Sk_list.append(minS)
                Rk_list.append(self.Rk)
	
        return Hk_list, Vk_list, Sk_list, Rk_list

    # ==========================================================================
    def update_position(self, Hk_list, Vk_list, Sk_list, Rk_list):
        """
        Update the position of the robot according to the given matrices.

        The matrices contain the current position and the data association
        parameters. All input lists have the same lenght.

        Input:
          Hk_list : list of 2x3 matrices (jacobian)
          Yk_list : list of 2x1 matrices (innovation)
          Sk_list : list of 2x2 matrices (innovation uncertainty)
          Rk_list : list of 2x2 matrices (measurement uncertainty)
        """
        # Compose list of matrices as single matrices
        n = len(Hk_list)
        H = np.zeros((2*n, 3))
        v = np.zeros((2*n))
        S = np.zeros((2*n, 2*n))
        R = np.zeros((2*n, 2*n))
        for i in range(n):
            H[2*i:2*i+2, :] = Hk_list[i]
            v[2*i:2*i+2] = Vk_list[i]
            S[2*i:2*i+2, 2*i:2*i+2] = Sk_list[i]
            R[2*i:2*i+2, 2*i:2*i+2] = Rk_list[i]

        # There is data to update
        if not n > 0:
            return

        # TODO: Program this function
        ################################################################
        # Do the EKF update
        k =np.dot(np.dot(self.Pk,H.T),np.linalg.inv(S))
	self.xk+=np.dot(k,v)
	a=np.dot(k,H)
	I=np.eye(a.shape[0])
	b=I-a
	c=np.dot(b,np.dot(self.Pk,b.T))
	self.Pk=c+np.dot(k,np.dot(R,k.T))

    # ==========================================================================
    def jacobianH(self, lineworld):
        """
        Compute the jacobian of the get_polar_line function.

        It does it with respect to the robot state xk (done in pre-lab).
        """
        # TODO: Program this function
        ################################################################
        # Complete the Jacobian H from the pre-lab
        # Jacobian H
        sa=np.sin(np.arctan2(self.xk[1],self.xk[0])-lineworld[1])
	ca=np.cos(np.arctan2(self.xk[1],self.xk[0])-lineworld[1])
	sxy=np.sqrt(self.xk[0]**2+self.xk[1]**2)
	h1=-(self.xk[0]*ca+self.xk[1]*sa)/sxy
	h2=-(self.xk[1]*ca-self.xk[0]*sa)/sxy
        H =np.array([[h1,h2,0],[0,0,-1]])

        return H
