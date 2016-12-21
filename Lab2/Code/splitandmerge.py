#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import math
from probabilistic_lib.functions import angle_wrap

#===============================================================================
def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1]
    
    # TODO: CODE HERE!!!
    # Check minimum number of points
    #if ... < min_points:
        #return None
    if (points.shape[1]<min_points):
    	return None
    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    a=y1-y2
    b=x2-x1
    c=x1*y2-x2*y1
    #
    #

    # Distances of points to line (prelab question)
    #
    #
    #
    max_d=0
    max_i=0	
    D=0
    for i in range(first_pt,last_pt):
    	D=abs(a*points[0,i]+b*points[1,i]+c)/math.sqrt(math.pow(a,2)+math.pow(b,2))
	if D>max_d:
	    max_d=D
    	    max_i=i

    # Check split threshold
    #if ... > split_thres:
    if(max_d>split_thres):

        # Check sublines
        #
        #prev = split(points, split_thres, inter_thres, min_points, ...)
        #post = split(points, split_thres, inter_thres, min_points, ...)
   	prev = split(points, split_thres, inter_thres, min_points, first_pt,max_i)
	post = split(points, split_thres, inter_thres, min_points, max_i+1,last_pt)
        # Return results of sublines
        #if prev is not None and post is not None:
            #return np.vstack((prev, post))
	if prev !=None and post !=None:
	    return np.vstack((prev, post))
        #elif prev is not None:
            #return prev
	elif prev != None:
	    return prev
        #elif post is not None:
            #return post
	elif post!=None:
	    return post
        #else:
            #return None
	else: 
	    return None
    # Do not need to split furthermore
    #else:
        # Optional check interpoint distance
        #for i in range(first_pt, last_pt):
            #
            #
            #
            #
            # Check interpoint distance threshold
            #if ... > inter_thres:
                #Split line
                #prev = split(points, split_thres, inter_thres, min_points, ...)
                #post = split(points, split_thres, inter_thres, min_points, ...)
               
                # Return results of sublines
                #if prev is not None and post is not None:
                    #return np.vstack((prev, post))
                #elif prev is not None:
                    #return prev
                #elif post is not None:
                    #return post
                #else:
                    #return None
        
        # It is a good line
        #return np.array([[x1, y1, x2, y2]])
        
    # Dummy answer --> delete it
    else:
        for j in range(first_pt, last_pt-1):
	    a=math.sqrt(math.pow(points[0,j]-points[0,j+1],2)+math.pow(points[1,j]-points[1,j+1],2))
	    if a > inter_thres:
		prev = split(points, split_thres, inter_thres, min_points,first_pt,j)
		post = split(points, split_thres, inter_thres, min_points, j+1,last_pt)
		if prev!= None and post != None:
  		    return np.vstack((prev, post))
		elif prev != None:
		    return prev
		elif post != None:
		    return post
		else:
		    return None
		    
    return np.array([[x1, y1, x2, y2]])

#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    # No data received
    if lines is None:
        return np.array([])
        
    # Check and merge similar consecutive lines
    i = 0
    #iteration=lines.shape[0]-1
    while i in range(0,lines.shape[0]-2):
        
        # Line angles
        ang1 = math.atan2(lines[i,3]-lines[i,1],lines[i,2]-lines[i,0])
        ang2 = math.atan2(lines[i+1,3]-lines[i+1,1],lines[i+1,2]-lines[i+1,0])
        
        # Below thresholds?
        angdiff =abs(ang2-ang1) 
        disdiff = math.sqrt(math.pow(lines[i+1,0]-lines[i,2],2)+math.pow(lines[i+1,1]-lines[i,3],2))
        if angdiff < ang_thres and disdiff < dist_thres:
            
            # Joined line
            lines[i,:] = [lines[i,0],lines[i,1],lines[i+1,2],lines[i+1,3]]
            # Delete unnecessary line
            lines = np.delete(lines, i+1,0)
            
        # Nothing to merge
        else:
            i += 1
            
    return lines
