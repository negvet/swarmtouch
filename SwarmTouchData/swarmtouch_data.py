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



class mocap_object:
    def __init__(self, name):
        self.name = name
        self.tf = '/vicon/'+name+'/'+name
        self.pose = np.array([])
        self.orient = np.array([])
        self.time = np.array([])
        self.min_wall_dist = 0
        self.mean_wall_dist = 0
        self.max_wall_dist = 0
        self.path_length = 0

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


def area(drone1,drone2,drone3):
	area_array = np.array([])
	for i in range(min(len(drone1.pose),len(drone2.pose),len(drone3.pose))):
		x = np.array([drone1.pose[i][0], drone2.pose[i][0], drone3.pose[i][0]])
		y = np.array([drone1.pose[i][1], drone2.pose[i][1], drone3.pose[i][1]])
		area = 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
		if len(area_array)==0:
			area_array = np.array([drone1.time[i][0], area])
		else:
			area_array = np.vstack((area_array, np.array([drone1.time[i][0], area]) ))
	return area_array

def centroid(drone1,drone2,drone3):
	centroid_array = np.array([])
	for i in range(min(len(drone1.pose),len(drone2.pose),len(drone3.pose))):
		x_aver = np.array([drone1.pose[i][0], drone2.pose[i][0], drone3.pose[i][0]])
		y_aver = np.array([drone1.pose[i][1], drone2.pose[i][1], drone3.pose[i][1]])
		z_aver = np.array([drone1.pose[i][2], drone2.pose[i][2], drone3.pose[i][2]])
		centr = np.array([drone1.time[i][0], np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
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

def plot(data, style='-'):
	plt.plot(data[:,1], -data[:,0], style)


def trajplot(centroid_array, drone1, drone2, drone3, title, wall_coords, obstacles=None):
	plt.figure()
	centroid_path = centroid_array[:,1:3]
	plot(centroid_path)
	plot(drone1.pose, '--')
	plot(drone2.pose, '--')
	plot(drone3.pose, '--')
	plot(wall_coords)
	legend_list = ['centroid','drone1', 'drone2', 'drone3', 'wall']
	# time_sample = 1000
	# plt.plot(drone1.pose[time_sample,1], -drone1.pose[time_sample,0], '*')
	# plt.plot(drone2.pose[time_sample,1], -drone2.pose[time_sample,0], '*')
	# plt.plot(drone3.pose[time_sample,1], -drone3.pose[time_sample,0], '*')
	# plt.plot(centroid_path[time_sample,1], -centroid_path[time_sample,0], 'x')
	if obstacles is not None:
		for obstacle in obstacles:
			plt.plot(obstacle.pose[1], -obstacle.pose[0],'o')
			legend_list.append(obstacle.name)
	plt.legend(legend_list)
	plt.xlabel('Y[m]')
	plt.ylabel('X[m]')
	plt.title(title)


def dist2wall(path, obstacles):
	wall_obstacles = [obstacles[7], obstacles[6], obstacles[3], obstacles[0], obstacles[1]]
	wall_coords = np.array([0,0])
	wall_coords = np.vstack((wall_coords,wall_obstacles[0].pose[:2]))
	if obstacles is not None:
		for obstacle in wall_obstacles:
			wall_coords = np.vstack( (wall_coords, np.vstack( (np.linspace(wall_coords[-1,0], obstacle.pose[:2][0], 10), np.linspace(wall_coords[-1,1], obstacle.pose[:2][1], 10)) ).T ) )
	wall_coords = wall_coords[1:]
	test = path[ path[:,0]>obstacles[7].pose[0] ]
	near_wall_path = test[ ~np.logical_and( np.logical_and( np.logical_and(test[:,0]<obstacles[6].pose[0] , test[:,0]>obstacles[7].pose[0]), test[:,1]<obstacles[0].pose[1]), test[:,1]>obstacles[6].pose[1]) ]
	
	min_wall_dist = np.min(cdist(near_wall_path,wall_coords))
	mean_wall_dist = 0
	max_wall_dist = mean_wall_dist
	current_dist = np.zeros(len(near_wall_path))
	for i in range(len(near_wall_path)):
		current_dist[i] = min(cdist(near_wall_path,wall_coords)[i,:])
		mean_wall_dist += current_dist[i] / len(near_wall_path)
		max_wall_dist = max(max_wall_dist,current_dist[i])

	#print 'min dist to wall index', np.where(current_dist==min_wall_dist)[0]
	#print 'max dist to wall index', np.where(current_dist==max_wall_dist)[0]

	return round(min_wall_dist,2), round(mean_wall_dist,2), round(max_wall_dist,2), wall_coords

def labirint(centroid_path, obstacles):
	labirint_path = centroid_path[np.logical_and( np.logical_and(centroid_path[:,1]<obstacles[2].pose[1], centroid_path[:,1]>obstacles[8].pose[1]), centroid_path[:,0]<obstacles[3].pose[0] )]
	labirint_path_length = path_length(labirint_path)
	t1_index = int(np.where(centroid_path[:,1]==labirint_path[0,1])[0][0])
	t2_index = int(np.where(centroid_path[:,1]==labirint_path[-1,1])[0][0])
	labirint_t_sec = abs(time[t1_index]-time[t2_index])

	return labirint_path_length, labirint_t_sec




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
					 #'Evgeny',
					 'Ruslan'
					 #'Tamash'
					]

default_area = 0.0693
area_array_with_glove_for_all = []
area_array_without_glove_for_all = []
palm_orient_without_glove_for_all = []
palm_orient_with_glove_for_all = []

data_with_glove = Data()
data_without_glove = Data()
global time
time = np.array([])



for name in subject_name_list:

	print "\n\nName:", name ,'______________________________________________________'
	data_without_glove.name = name
	data_with_glove.name = name
	directory_without_glove = "/home/ruslan/Desktop/SwarmTouchData/"+name+"/without_glove/"
	directory_with_glove = "/home/ruslan/Desktop/SwarmTouchData/"+name+"/with_glove/"

	list_of_directories_with_experiments = [directory_without_glove, directory_with_glove]
	# print 'list_of_directories_with_experiments', list_of_directories_with_experiments

	for exper_type in list_of_directories_with_experiments: # With and without glove
		# print 'exper_type', exper_type
		experiment_list = sorted([x[0] for x in os.walk(exper_type)])
		experiment_list.pop(0)
		# print 'experiment_list', experiment_list

		for experiment_location in experiment_list:
			# print "experiment_location", experiment_location
			data_with_glove.folder_to_save = experiment_location
			data_without_glove.folder_to_save = experiment_location

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
							time = np.append(time, int(row[4]))
							if len(drone.pose)==0:
								init_time = float(row[0][:10])
								init_time = init_time + (float(row[0]) / 1000000000) - init_time
								drone.pose = np.array([float(row[10]), float(row[11]), float(row[12])])
								drone.time = np.array([ (float(row[0]) / 1000000000) - init_time ])
							else:
								drone.pose = np.vstack((drone.pose, np.array([float(row[10]), float(row[11]), float(row[12])])))
								drone.time = np.vstack((drone.time,   (float(row[0]) / 1000000000) - init_time  ))

			# OBSTACLES
			obstacle_names   = np.array(['obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8', 'obstacle9'])
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

			# GLOVE
			glove = mocap_object('glove')
			csv_file_name = experiment_location + '/_slash_vicon_slash_glove_slash_glove.csv'
			with open(csv_file_name) as csvfile:
				reader = csv.reader(csvfile)
				for row in reader:
					if row[10] != "x": # skip the first line
						if len(glove.pose)==0:
							init_time = float(row[0][:10])
							init_time = init_time + (float(row[0]) / 1000000000) - init_time
							glove.pose = np.array([float(row[10]), float(row[11]), float(row[12])])
							glove.orient = euler_from_quaternion( np.array([float(row[14]), float(row[15]), float(row[16]), float(row[17])]))[2]*180/math.pi
							glove.time = np.array([ (float(row[0]) / 1000000000) - init_time ])
						else:
							glove.pose = np.vstack((glove.pose, np.array([float(row[10]), float(row[11]), float(row[12])])))
							glove.orient = np.vstack( (glove.orient, euler_from_quaternion( np.array([float(row[14]), float(row[15]), float(row[16]), float(row[17])]))[2]*180/math.pi ) )
							glove.time = np.vstack((glove.time,   (float(row[0]) / 1000000000) - init_time  ))

			if 'without_glove' in exper_type:
				area_array_without_glove = area(drone1,drone2,drone3)
				area_array_without_glove_for_all.append(area_array_without_glove)
				area_array_without_glove_error = abs(area_array_without_glove[:,1] - default_area)
				palm_orient_without_glove_for_all.append(glove.orient)

				data_without_glove.glove_status = 'Without_glove'
				data_without_glove.S_min_er = round( np.mean(area_array_without_glove_error),4 )
				data_without_glove.S_std_er = round( np.std(area_array_without_glove_error),4 )
				data_without_glove.S_max_er = round( np.amax(area_array_without_glove_error), 4)

				#velocity
				centroid_array = centroid(drone1,drone2,drone3)
				vel_x = np.diff(centroid_array[:,1]) / np.diff(centroid_array[:,0])
				acc_x = np.diff(vel_x) / np.diff(centroid_array[:-1][:,0])
				jerk_x = np.diff(acc_x) / np.diff(centroid_array[:-2][:,0])

				data_without_glove.vel_mean = np.mean(abs(vel_x))
				data_without_glove.acc_mean = np.mean(abs(acc_x))
				data_without_glove.jerk_mean = np.mean(abs(jerk_x))

				data_without_glove.drone1_path_length = round( path_length(drone1.pose),2 )
				data_without_glove.drone2_path_length = round( path_length(drone2.pose),2 )
				data_without_glove.drone3_path_length = round( path_length(drone3.pose),2 )
				data_without_glove.centroid_path_length = round( path_length(centroid_array[:,1:]), 2)

				centroid_path = centroid_array[:,1:3]
				data_without_glove.labirint_path_length, data_without_glove.labirint_t_sec = labirint(centroid_path, obstacles)
				wall_coords = dist2wall(drone1.pose[:,:2],obstacles)[-1]
				#trajplot(centroid_array, drone1, drone2, drone3, data_without_glove.name+" Without glove", wall_coords, obstacles)

				data_without_glove.drone1_min_wall_dist, data_without_glove.drone1_mean_wall_dist, data_without_glove.drone1_max_wall_dist, _ = dist2wall(drone1.pose[:,:2],obstacles)
				data_without_glove.drone2_min_wall_dist, data_without_glove.drone2_mean_wall_dist, data_without_glove.drone2_max_wall_dist, _ = dist2wall(drone2.pose[:,:2],obstacles)
				data_without_glove.drone3_min_wall_dist, data_without_glove.drone3_mean_wall_dist, data_without_glove.drone3_max_wall_dist, _ = dist2wall(drone3.pose[:,:2],obstacles)
				data_without_glove.min_dist_to_wall_centr, data_without_glove.mean_dist_to_wall_centr, data_without_glove.max_dist_to_wall_centr, _ = dist2wall(centroid_path,obstacles)

				savedata(data_without_glove)
				
				

			if 'with_glove' in exper_type:
				area_array_with_glove = area(drone1,drone2,drone3)
				area_array_with_glove_for_all.append(area_array_with_glove)
				palm_orient_with_glove_for_all.append(glove.orient)
				area_array_with_glove_error = abs(area_array_with_glove[:,1] - default_area)

				data_with_glove.glove_status = 'With_glove'
				data_with_glove.S_min_er = round( np.mean(area_array_with_glove_error),4 )
				data_with_glove.S_std_er = round( np.std(area_array_with_glove_error), 4)
				data_with_glove.S_max_er = round( np.amax(area_array_with_glove_error),4 )

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
				data_with_glove.labirint_path_length, data_with_glove.labirint_t_sec = labirint(centroid_path, obstacles)
				wall_coords = dist2wall(drone1.pose[:,:2],obstacles)[-1]
				#trajplot(centroid_array, drone1, drone2, drone3, data_without_glove.name+" With glove", wall_coords, obstacles)

				data_with_glove.drone1_min_wall_dist, data_with_glove.drone1_mean_wall_dist, data_with_glove.drone1_max_wall_dist,_ = dist2wall(drone1.pose[:,:2],obstacles)
				data_with_glove.drone2_min_wall_dist, data_with_glove.drone2_mean_wall_dist, data_with_glove.drone2_max_wall_dist,_ = dist2wall(drone2.pose[:,:2],obstacles)
				data_with_glove.drone3_min_wall_dist, data_with_glove.drone3_mean_wall_dist, data_with_glove.drone3_max_wall_dist,_ = dist2wall(drone3.pose[:,:2],obstacles)
				data_with_glove.min_dist_to_wall_centr, data_with_glove.mean_dist_to_wall_centr, data_with_glove.max_dist_to_wall_centr,_ = dist2wall(centroid_path,obstacles)

				savedata(data_with_glove)

fig = plt.figure()
delimeter = 60 # take pts each 0.5 sec if delimeter=30 (1.0 sec for delimeter=60)
centroid_path_sampled = centroid_path[::delimeter,:]
fig.add_subplot(121)
plot(centroid_path_sampled, '--')

x = centroid_path_sampled[:,0]
y = centroid_path_sampled[:,1]

n_circle_pts = 5
curv_array = np.array([])
for i in range(len(x)-n_circle_pts):
	x_pts = x[i:i+n_circle_pts]
	y_pts = y[i:i+n_circle_pts]
	curv = curvature.plotter(x_pts,y_pts)
	curv_array = np.append(curv_array, curv)
	#plt.show()
plt.title('Mean rarius: '+str(round( np.mean(1./curv_array), 2 )) )

fig.add_subplot(122)
plt.title('Curvature, mean='+str(round( np.mean(curv_array), 2 )))
plt.plot(curv_array)

# Figures
# f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
#AREA
# for i in range(len(area_array_without_glove_for_all)):
# 	ax1.plot(area_array_without_glove_for_all[i][:,0], area_array_without_glove_for_all[i][:,1])
# for i in range(len(area_array_with_glove_for_all)):
# 	ax2.plot(area_array_with_glove_for_all[i][:,0], area_array_with_glove_for_all[i][:,1])


# ax3.plot(palm_orient_without_glove_for_all[0])
# ax3.plot(palm_orient_with_glove_for_all[0])

# ax4.plot(palm_orient_without_glove_for_all[1])
# ax4.plot(palm_orient_with_glove_for_all[1])



# ax1.set_title('Area Without glove')
# ax2.set_title('Area With glove')
# ax1.axis([0, 60, 0.00, 0.1])
# ax2.axis([0, 60, 0.00, 0.1])

# ax1.axhline(y=default_area, color='k')
# ax2.axhline(y=default_area, color='k')

# ax3.set_title('Yaw '+subject_name_list[0])
# ax4.set_title('Yaw '+subject_name_list[1])


plt.show()
