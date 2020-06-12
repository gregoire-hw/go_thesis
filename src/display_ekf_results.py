#! /usr/bin/env python

import sys
import math
import matplotlib.pyplot as plt
import csv
import os

dvl_time = []
dvl_x = [] 
dvl_y = []
dvl_z = []
gt_time = []
gt_x = []
gt_y = []
gt_z = []
ekf_time = []
ekf_x = []
ekf_y = []
ekf_z = []
cov = []
innov = []

my_path = os.path.abspath(os.path.dirname(__file__))
path = os.path.join(my_path, "data/DR.csv")
with open(path) as csv_file:
	csv_reader = csv.DictReader(csv_file, delimiter=',')
	for row in csv_reader:
		dvl_time.append(float(row['time']))
		dvl_x.append(float(row['X']))
		dvl_y.append(float(row['Y']))
		dvl_z.append(float(row['Z']))

my_path = os.path.abspath(os.path.dirname(__file__))
path = os.path.join(my_path, "data/ekf.csv")
with open(path) as csv_file:
	csv_reader = csv.DictReader(csv_file, delimiter=',')
	for row in csv_reader:
		ekf_time.append(float(row['time']))
		ekf_x.append(float(row['X']))
		ekf_y.append(float(row['Y']))
		ekf_z.append(float(row['Z']))

my_path = os.path.abspath(os.path.dirname(__file__))
path = os.path.join(my_path, "data/gt.csv")
with open(path) as csv_file:
	csv_reader = csv.DictReader(csv_file, delimiter=',')
	for row in csv_reader:
		gt_time.append(float(row['time']))
		gt_x.append(float(row['X']))
		gt_y.append(float(row['Y']))
		gt_z.append(float(row['Z']))

plt.figure(0)
plt.title('paths')
plt.plot(dvl_x,dvl_y,'b')
plt.plot(ekf_x,ekf_y,'r')
plt.plot(gt_x,gt_y,'g')
