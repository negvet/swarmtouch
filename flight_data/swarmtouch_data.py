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
from low_pass_filter import butter_lowpass_filter


class mocap_object:
    def __init__(self, name):
        self.name = name
        self.tf = '/vicon/'+name+'/'+name
        self.pose = np.array([])
        self.orient = np.array([])
        self.time = np.array([])
        # self.obstacle_update_status = [False, None]

def area(drone1,drone2,drone3):
	area_array = np.array([])
	for i in range(len(drone1.pose)-100):
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
	for i in range(min(len(drone1.pose), len(drone2.pose), len(drone3.pose))):
		x_aver = np.array([drone1.pose[i][0], drone2.pose[i][0], drone3.pose[i][0]])
		y_aver = np.array([drone1.pose[i][1], drone2.pose[i][1], drone3.pose[i][1]])
		z_aver = np.array([drone1.pose[i][2], drone2.pose[i][2], drone3.pose[i][2]])
		centr = np.array([drone1.time[i][0], np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
		if len(centroid_array)==0:
			centroid_array = centr
		else:
			centroid_array = np.vstack((centroid_array, centr ))
	return centroid_array

def derivatives(title):
	centroid_array = centroid(drone1,drone2,drone3)
	# raw velocities
	vel_x = np.diff(centroid_array[:,1]) / np.diff(centroid_array[:,0])
	vel_y = np.diff(centroid_array[:,2]) / np.diff(centroid_array[:,0])
	vel_z = np.diff(centroid_array[:,3]) / np.diff(centroid_array[:,0])
	vel_x = vel_x[10:]; vel_y = vel_y[10:]; vel_z = vel_z[10:]
	# filtered velocities
	vel_x = butter_lowpass_filter(vel_x, cutoff=4, fs=60)
	vel_y = butter_lowpass_filter(vel_y, cutoff=4, fs=60)
	vel_z = butter_lowpass_filter(vel_z, cutoff=4, fs=60)
	# raw acc
	acc_x = np.diff(vel_x) / np.diff(centroid_array[:-1][10:,0])
	acc_y = np.diff(vel_y) / np.diff(centroid_array[:-1][10:,0])
	acc_z = np.diff(vel_z) / np.diff(centroid_array[:-1][10:,0])
	# filtered acc
	acc_x = butter_lowpass_filter(acc_x, cutoff=4, fs=60)
	acc_y = butter_lowpass_filter(acc_y, cutoff=4, fs=60)
	acc_z = butter_lowpass_filter(acc_z, cutoff=4, fs=60)
	# raw jerk
	jerk_x = np.diff(acc_x) / np.diff(centroid_array[:-2][10:,0])
	jerk_y = np.diff(acc_y) / np.diff(centroid_array[:-2][10:,0])
	jerk_z = np.diff(acc_z) / np.diff(centroid_array[:-2][10:,0])
	# filtered jerk
	jerk_x = butter_lowpass_filter(jerk_x, cutoff=4, fs=60)
	jerk_y = butter_lowpass_filter(jerk_y, cutoff=4, fs=60)
	jerk_z = butter_lowpass_filter(jerk_z, cutoff=4, fs=60)

	if visualize:
		plot_vel_acc_jerk(vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, jerk_x, jerk_y, jerk_z, title)

	vel = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
	acc = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
	jerk = np.sqrt(jerk_x**2 + jerk_y**2 + jerk_z**2)

	return vel, acc, jerk


def plot_vel_acc_jerk(vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, jerk_x, jerk_y, jerk_z, title):
	plt.figure()
	plt.subplot(3, 1, 1)
	plt.title(title)
	plt.plot(vel_x, label='$V_x$')
	plt.plot(vel_y, label='$V_y$')
	plt.plot(vel_z, label='$V_z$')
	plt.legend()

	plt.subplot(3, 1, 2)
	plt.plot(acc_x, label='$a_x$')
	plt.plot(acc_y, label='$a_y$')
	plt.plot(acc_z, label='$a_z$')
	plt.legend()

	plt.subplot(3, 1, 3)
	plt.plot(jerk_x, label='$j_x$')
	plt.plot(jerk_y, label='$j_y$')
	plt.plot(jerk_z, label='$j_z$')
	plt.legend()


# area_user
# area_all_users


visualize = 0
PATH = os.getcwd()+"/"
subject_name_list = [
					 'Grisha',
					 'Evgeny',
					 'Ruslan',
					 'Tamash',
					 'Vladimir'
					]
default_area = 0.0693
area_array_with_glove_for_all = []
area_array_without_glove_for_all = []

for name in subject_name_list:

	print "\n\nName:", name ,'______________________________________________________'
	directory_without_glove = PATH+name+"/without_glove/"
	directory_with_glove = PATH+name+"/with_glove/"

	list_of_directories_with_experiments = [directory_without_glove, directory_with_glove]
	# print 'list_of_directories_with_experiments', list_of_directories_with_experiments

	for exper_type in list_of_directories_with_experiments: # With and without glove
		# print 'exper_type', exper_type
		experiment_list = sorted([x[0] for x in os.walk(exper_type)])
		experiment_list.pop(0)
		experiment_list.pop(-1)
		# print 'experiment_list', experiment_list

		for experiment_location in experiment_list:
			# print "experiment_location", experiment_location
			
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

			if 'without_glove' in exper_type:
				area_array_without_glove = area(drone1,drone2,drone3)
				area_array_without_glove_for_all.append(area_array_without_glove)
				area_array_without_glove_error = abs(area_array_without_glove[:,1] - default_area)
				print 'without_glove AREA error:'
				print 'np.mean  ', np.mean(area_array_without_glove_error)
				print 'np.std   ', np.std(area_array_without_glove_error)
				print 'np.amax  ', np.amax(area_array_without_glove_error)

				#velocity
				print 'without_glove VELOCITY'
				vel, acc, jerk = derivatives(title=name+' without glove')
				print 'vel mean', np.mean(vel)
				print 'acc mean', np.mean(acc)
				print 'jerk mean', np.mean(jerk)


			if 'with_glove' in exper_type:
				area_array_with_glove = area(drone1,drone2,drone3)
				area_array_with_glove_for_all.append(area_array_with_glove)

				area_array_with_glove_error = abs(area_array_with_glove[:,1] - default_area)
				print 'with_glove AREA error:'
				print 'np.mean  ', np.mean(area_array_with_glove_error)
				print 'np.std   ', np.std(area_array_with_glove_error)
				print 'np.amax  ', np.amax(area_array_with_glove_error)

				#velocity
				print 'with_glove VELOCITY'
				vel, acc, jerk = derivatives(title=name+' with glove')
				print 'vel mean', np.mean(vel)
				print 'acc mean', np.mean(acc)
				print 'jerk mean', np.mean(jerk)



# f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)

#AREA
# for i in range(len(area_array_without_glove_for_all)):
# 	ax1.plot(area_array_without_glove_for_all[i][:,0], area_array_without_glove_for_all[i][:,1])
# for i in range(len(area_array_with_glove_for_all)):
# 	ax2.plot(area_array_with_glove_for_all[i][:,0], area_array_with_glove_for_all[i][:,1])

# ax1.set_title('Area Without glove')
# ax2.set_title('Area With glove')
# ax1.axis([0, 60, 0.00, 0.1])
# ax2.axis([0, 60, 0.00, 0.1])

# ax1.axhline(y=default_area, color='k')
# ax2.axhline(y=default_area, color='k')

if visualize:
	plt.draw()
	plt.pause(0.1)
	raw_input('Hit Enter to continue')
	plt.close('all')
