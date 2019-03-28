#!/usr/bin/env python
from __future__ import division
import csv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import os
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import math
from tf.transformations import *
from scipy.spatial.distance import cdist
import xlwt
import curvature

plt.rcParams.update({'font.size': 18})
PATH = os.getcwd()+'/'

class mocap_object:
    def __init__(self, name):
        self.name = name
        self.tf = '/vicon/'+name+'/'+name
        self.pose = np.array([])
        self.orient = np.array([])
        self.time = np.array([])
        self.rostime = np.array([])
        self.path_length = 0

class Patterns:
    def __init__(self):
    	self.names = np.array([])
    	self.rostime = np.array([])
    	self.patterns_times = np.array([])


class Data:
    def __init__(self):
    	self.name = ''
    	self.folder_to_save = '~/Desktop/'
    	self.glove_status = ''
        self.S_min_er = 0
        self.S_std_er = 0
        self.S_max_er = 0

        self.vel_mean = 0
        self.acc_mean = 0
        self.jerk_mean = 0

        self.centroid_path_length = 0
        self.min_dist_to_wall_centr = 0
        self.mean_dist_to_wall_centr = 0
        self.max_dist_to_wall_centr = 0

        self.drone1_path_length = 0
        self.drone1_min_wall_dist = 0
        self.drone1_mean_wall_dist = 0
        self.drone1_max_wall_dist = 0

        self.drone2_path_length = 0
        self.drone2_min_wall_dist = 0
        self.drone2_mean_wall_dist = 0
        self.drone2_max_wall_dist = 0

        self.drone3_path_length = 0
        self.drone3_min_wall_dist = 0
        self.drone3_mean_wall_dist = 0
        self.drone3_max_wall_dist = 0

        self.labirint_path_length = 0
        self.labirint_t_sec = 0


def area(drone1,drone2,drone3, simulation=False):
	area_array = np.array([])
	if simulation:
		drone1_time = drone1.timesp; drone1_pose = drone1.sp; drone2_pose = drone2.sp; drone3_pose = drone3.sp;
	else:
		drone1_time = drone1.time; drone1_pose = drone1.pose; drone2_pose = drone2.pose; drone3_pose = drone3.pose;
	for i in range(min(len(drone1_pose),len(drone2_pose),len(drone3_pose))):
		x = np.array([drone1_pose[i][0], drone2_pose[i][0], drone3_pose[i][0]])
		y = np.array([drone1_pose[i][1], drone2_pose[i][1], drone3_pose[i][1]])
		area = 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
		if len(area_array)==0:
			area_array = np.array([drone1_time[i][0], area])
		else:
			area_array = np.vstack((area_array, np.array([drone1_time[i][0], area]) ))
	return area_array

def centroid(drone1,drone2,drone3, simulation=False):
	centroid_array = np.array([])
	if simulation:
		drone1_time = drone1.timesp; drone1_pose = drone1.sp; drone2_pose = drone2.sp; drone3_pose = drone3.sp;
	else:
		drone1_time = drone1.time; drone1_pose = drone1.pose; drone2_pose = drone2.pose; drone3_pose = drone3.pose;
	for i in range(min(len(drone1_pose),len(drone2_pose),len(drone3_pose))):
		x_aver = np.array([drone1_pose[i][0], drone2_pose[i][0], drone3_pose[i][0]])
		y_aver = np.array([drone1_pose[i][1], drone2_pose[i][1], drone3_pose[i][1]])
		z_aver = np.array([drone1_pose[i][2], drone2_pose[i][2], drone3_pose[i][2]])
		centr = np.array([drone1_time[i][0], np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
		if len(centroid_array)==0:
			centroid_array = centr
		else:
			centroid_array = np.vstack((centroid_array, centr ))
	return centroid_array

def path_length(drone_pose):
	length = 0
	for i in range( 1,len(drone_pose) ):
		length += np.linalg.norm(drone_pose[i,:]-drone_pose[i-1,:])
	return length

def polyarea(drone1, drone2, drone3, time_index):
	t = time_index
	if t>min( len(drone1.pose), len(drone2.pose), len(drone3.pose) ):
		t = min( (drone1.pose).shape[0], (drone2.pose).shape[0], (drone3.pose).shape[0] ) - 1
	x = np.array([drone1.pose[t,0], drone2.pose[t,0], drone3.pose[t,0]])
	y = np.array([drone1.pose[t,1], drone2.pose[t,1], drone3.pose[t,1]])
	return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))


def plot(data, style='-'):
	plt.plot(data[:,1], -data[:,0], style, linewidth=2.0)


def deviation_evaluator(centr_action, pattern_name):
	if pattern_name=='contracted_right':
		gain = (centr_action[:,1][-1] - centr_action[:,1][0] > 0)
	elif pattern_name=='contracted_left':
		gain = (centr_action[:,1][-1] - centr_action[:,1][0] < 0)
	elif 'extended' in pattern_name:
		gain = (centr_action[:,0][-1] - centr_action[:,0][0] < 0)
	else:
		gain = -1
	return gain

def areachange_evaluator(area_change, pattern_name):
	if (area_change>0.00693):
		gain = ('contracted' in pattern_name)
	elif (area_change<-0.00693):
		gain = ('extended' in pattern_name)
	else:
		gain = 0.5
	return gain

def nsamples_from_pattern_duration(pattern_name, reaction_time=0):
	if 'extended' in pattern_name:
		ms = 400+reaction_time
	elif 'contracted' in pattern_name:
		ms = 1200+reaction_time
	rate = 60
	n = rate*ms / 1000
	return int(n)

def trajeval(centroid_array, drone1, drone2, drone3, title, obstacles=None, patterns=None):
	fig = plt.figure()
	ax = fig.gca()
	centroid_path = centroid_array[:,1:3]
	t_index = -1
	plot(centroid_path[:t_index,:])
	plot(drone1.pose[:t_index,:], '--')
	plot(drone2.pose[:t_index,:], '--')
	plot(drone3.pose[:t_index,:], '--')
	legend_list = ['centroid','drone1', 'drone2', 'drone3']
	plt.plot(drone1.pose[-1,1], -drone1.pose[-1,0], 'ro', color='blue')
	plt.plot(drone2.pose[-1,1], -drone2.pose[-1,0], 'ro', color='blue')
	plt.plot(drone3.pose[-1,1], -drone3.pose[-1,0], 'ro', color='blue')


	if obstacles is not None:
		for obstacle in obstacles:
			circle = plt.Circle((obstacle.pose[1], -obstacle.pose[0]),0.27, color='yellow')
			ax.add_artist(circle)
			plt.plot(obstacle.pose[1], -obstacle.pose[0],'ro')
			legend_list.append(obstacle.name)
	#plt.legend(legend_list)
	gain_dev = 0
	gain_area = 0
	for  i in range(len(patterns.names)):
		reaction_time = 1500 # [ms]
		pattern_start = (np.abs(drone1.rostime - patterns.patterns_times[i]).argmin())
		pattern_end = pattern_start+nsamples_from_pattern_duration(patterns.names[i])
		pattern_and_react_time = pattern_start+nsamples_from_pattern_duration(patterns.names[i], reaction_time)
		centr_deviation = centroid_path[pattern_start : pattern_and_react_time]
		gain_dev += deviation_evaluator(centr_deviation, patterns.names[i])
		# print 'dx=',centr_action[:,0][-1] - centr_action[:,0][0]
		# print 'dy=',centr_action[:,1][-1] - centr_action[:,1][0]
		# print patterns.names[i]
		# print '\n'
		area_change = polyarea(drone1, drone2, drone3, pattern_and_react_time) -\
					  polyarea(drone1, drone2, drone3, pattern_start) 
		gain_area += areachange_evaluator(area_change, patterns.names[i])
		# print areachange_evaluator(area_change, patterns.names[i])
		# print 'area change=', area_change
		# print patterns.names[i]
		# print '\n'
		plot(centr_deviation, 'x')
		plt.plot( centroid_path[pattern_start,1], -centroid_path[pattern_start,0], 'o', markersize=6 )
	#print 'centroid dev gain[%]', round( gain_dev / len(patterns.patterns_times), 3 ) * 100
	print 'area change gain[%]', round( gain_area / len(patterns.patterns_times), 3 ) * 100
	plt.xlabel('Y, meters')
	plt.ylabel('X, meters')
	plt.grid()
	ax.set_aspect('equal')
	plt.title(title)


def savedata(data):
	#style0 = xlwt.easyxf('font: name Times New Roman, color-index red, bold on', num_format_str='#,##0.00')
	#style1 = xlwt.easyxf(num_format_str='D-MMM-YY')

	wb = xlwt.Workbook()
	ws = wb.add_sheet(data.glove_status)

	ws.write(0, 0, 'S_min_er'); ws.write(0, 1, data.S_min_er)
	ws.write(1, 0, 'S_std_er'); ws.write(1, 1, data.S_std_er)
	ws.write(2, 0, 'S_max_er'); ws.write(2, 1, data.S_max_er)

	ws.write(3, 0, 'vel_mean'); ws.write(3, 1, data.vel_mean)
	ws.write(4, 0, 'acc_mean'); ws.write(4, 1, data.acc_mean)
	ws.write(5, 0, 'jerk_mean'); ws.write(5, 1, data.jerk_mean)

	ws.write(6, 0, 'drone1_path_length'); 		ws.write(6, 1, data.drone1_path_length)
	ws.write(7, 0, 'drone1_min_dist_to_wall'); 	ws.write(7, 1, data.drone1_min_wall_dist)
	ws.write(8, 0, 'drone1_mean_dist_to_wall'); ws.write(8, 1, data.drone1_mean_wall_dist)	
	ws.write(9, 0, 'drone1_max_dist_to_wall'); 	ws.write(9, 1, data.drone1_max_wall_dist)	

	ws.write(6, 3, 'drone2_path_length'); 		ws.write(6, 4, data.drone2_path_length)
	ws.write(7, 3, 'drone2_min_dist_to_wall'); 	ws.write(7, 4, data.drone2_min_wall_dist)
	ws.write(8, 3, 'drone2_mean_dist_to_wall'); ws.write(8, 4, data.drone2_mean_wall_dist)	
	ws.write(9, 3, 'drone2_max_dist_to_wall'); 	ws.write(9, 4, data.drone2_max_wall_dist)	

	ws.write(6, 6, 'drone3_path_length'); 		ws.write(6, 7, data.drone3_path_length)
	ws.write(7, 6, 'drone3_min_dist_to_wall'); 	ws.write(7, 7, data.drone3_min_wall_dist)
	ws.write(8, 6, 'drone3_mean_dist_to_wall'); ws.write(8, 7, data.drone3_mean_wall_dist)	
	ws.write(9, 6, 'drone3_max_dist_to_wall'); 	ws.write(9, 7, data.drone3_max_wall_dist)

	ws.write(10, 0, 'centroid_path_length'); 	ws.write(10, 1, data.centroid_path_length)
	ws.write(11, 0, 'min_dist_to_wall_centr'); 	ws.write(11, 1, data.min_dist_to_wall_centr)
	ws.write(12, 0, 'mean_dist_to_wall_centr'); ws.write(12, 1, data.mean_dist_to_wall_centr)	
	ws.write(13, 0, 'max_dist_to_wall_centr'); 	ws.write(13, 1, data.max_dist_to_wall_centr)

	ws.write(14, 0, 'labirint_path_length'); 	ws.write(14, 1, data.labirint_path_length)	
	ws.write(15, 0, 'labirint_t_sec'); 			ws.write(15, 1, data.labirint_t_sec)

	wb.save(data.folder_to_save+'output.xls')


subject_name_list = [
					 'Grisha',
					 # 'Evgeny',
					 # 'Ruslan',
					 # 'Tamash',
					 # 'Vladimir'
					]
visualize = 1

default_area = 0.0693
area_array_with_glove_for_all = []
area_array_without_glove_for_all = []

data_with_glove = Data()



for name in subject_name_list:

	print "\n\nName:", name
	data_with_glove.name = name
	directory_with_glove = PATH+name+"/with_glove/"

	experiment_list = sorted([x[0] for x in os.walk(directory_with_glove)])
	#print 'experiment_list', experiment_list
	experiment_list.pop(0)

	for experiment_location in experiment_list:
		# print "experiment_location", experiment_location
		data_with_glove.folder_to_save = experiment_location

		drone1 = mocap_object('cf1')
		drone2 = mocap_object('cf2')
		drone3 = mocap_object('cf3')
		drone_object_list = [drone1, drone2, drone3]

		# DRONES
		for drone in drone_object_list:
			csv_file_name = experiment_location + '/_slash_vicon_slash_'+drone.name+'_slash_'+drone.name+'.csv'
			with open(csv_file_name) as csvfile:
				reader = csv.reader(csvfile)
				for row in reader:
					if row[10] != "x": # skip the first line
						if len(drone.pose)==0:
							init_time = float(row[0][:10])
							init_time = init_time + (float(row[0]) / 1000000000) - init_time
							drone.pose = np.array([float(row[10]), float(row[11]), float(row[12])])
							drone.time = np.array([ (float(row[0]) / 1000000000) - init_time ])
						else:
							drone.pose = np.vstack((drone.pose, np.array([float(row[10]), float(row[11]), float(row[12])])))
							drone.time = np.vstack((drone.time,   (float(row[0]) / 1000000000) - init_time  ))
						drone.rostime = np.append(drone.rostime, int(row[0]))
		print drone.pose.shape, drone.time.shape
		# OBSTACLES
		obstacle_names   = np.array([
						 'obstacle0', 'obstacle1',
						 'obstacle2', 'obstacle3',
						 'obstacle4', 'obstacle5',
						 'obstacle6', 'obstacle7',
						 'obstacle8', 'obstacle9',
						 'obstacle10', 'obstacle11',
						 'obstacle12', 'obstacle13',
						 'obstacle14', 'obstacle15',
						 'obstacle16', 'obstacle17',
						 'obstacle18', 'obstacle19',
						 'obstacle20', 'obstacle21',
						 'obstacle22', 'obstacle23',
						 'obstacle24', 'obstacle25'
						 ])
		obstacles = np.array([])
		for i in range(len(obstacle_names)):
			obstacles = np.append(obstacles, mocap_object( obstacle_names[i]))
		for obstacle in obstacles:
			csv_file_name = experiment_location + '/_slash_vicon_slash_'+obstacle.name+'_slash_'+obstacle.name+'.csv'
			with open(csv_file_name) as csvfile:
				reader = csv.reader(csvfile)
				for row in reader:
					if row[10] != "x": # skip the first line
						obstacle.pose = np.array([float(row[10]), float(row[11]), float(row[12])])
			
		# PATTERNS
		patterns = Patterns()
		csv_file_name = experiment_location + '/_slash_pattern_topic.csv'
		with open(csv_file_name) as csvfile:
			reader = csv.reader(csvfile)
			r = 0
			for row in reader:
				if row[1] != "data": # skip 1st raw
					r += 1
					patterns.rostime = np.append(patterns.rostime, int(row[0]))
					if row[1] != "''":
						patterns.names = np.append(patterns.names,row[1][1:-1])
						patterns.patterns_times = np.append(patterns.patterns_times, int(row[0]))


		data_with_glove.glove_status = 'Tactile'

		#velocity
		centroid_array = centroid(drone1,drone2,drone3)
		vel_x = np.diff(centroid_array[:,1]) / np.diff(centroid_array[:,0])
		acc_x = np.diff(vel_x) / np.diff(centroid_array[:-1][:,0])
		jerk_x = np.diff(acc_x) / np.diff(centroid_array[:-2][:,0])

		data_with_glove.vel_mean = np.mean(abs(vel_x))
		data_with_glove.acc_mean = np.mean(abs(acc_x))
		data_with_glove.jerk_mean = np.mean(abs(jerk_x))

		data_with_glove.drone1_path_length = round( path_length(drone1.pose),2 )
		data_with_glove.drone2_path_length = round( path_length(drone2.pose),2 )
		data_with_glove.drone3_path_length = round( path_length(drone3.pose),2 )
		data_with_glove.centroid_path_length = round( path_length(centroid_array[:,1:]), 2)

		centroid_path = centroid_array[:,1:3]
		trajeval(centroid_array, drone1, drone2, drone3, data_with_glove.name+" Tactile", obstacles, patterns)

		savedata(data_with_glove)

if visualize:
	plt.draw()
	plt.pause(0.1)
	raw_input('Hit Enter to close')
	plt.close('all')
