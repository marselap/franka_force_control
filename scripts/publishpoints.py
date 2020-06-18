#!/usr/bin/env python

import rospy

from os import listdir
from os.path import isfile, join

import numpy as np
import pandas as pd 

import scipy, scipy.optimize, scipy.signal
from scipy.spatial.transform import Rotation as R

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import itertools

import csv

import re


if __name__ == '__main__':
    rospy.init_node('read_csv')

    pub = rospy.Publisher('/path_ref', JointTrajectory, queue_size=1)    


    data_1 = np.genfromtxt('/home/franka/exploration_pc/otee.csv',delimiter=',')

    # ref = np.genfromtxt('/home/franka/exploration_pc/expl_dir.csv',delimiter=',')
    # ref = pd.read_csv('/home/franka/exploration_pc/expl_dir.csv')

    # print ref

    data_1 = data_1[1:]

    matrices = []
    for d in data_1:
        mtrx = []
        for i in xrange(4):
            row = []
            for j in xrange(4):
                row.append(d[j*4+i+1])
            mtrx.append(row)
        matrices.append(mtrx)


    pos = []
    rot = []
    # pos_ee = []
    for t, trans in zip(data_1, matrices):
        # pos_ee_ = [trans[0,3], trans[0,3], trans[0,3]], 
        pos_ = np.dot(np.matrix(trans), np.array([0., 0., 0.04, 1.]))
        pos_ = pos_.tolist()[0]
        pos.append([t[0], pos_[0], pos_[1], pos_[2]])
        # pos_ee.append([t[0], pos_ee_[0], pos_ee_[1], pos_ee_[2]])
        rot_ = []
        for i,row in enumerate(trans):
            if i<3:
                rot_.append(row[0:3])
        rot.append(np.matrix(rot_))


    starttime = 1568540833072666373
    endtime = 1568540846073661037

    # print ((pos[:,0] > starttime) and (pos[:,0] < endtime) )


    pos_filt = [row for row in pos if row[0] > starttime and row[0]< endtime]
    rot_filt = [mtrx for row,mtrx in zip(pos,rot) if row[0] > starttime and row[0]< endtime]


    rot = []
    for r in rot_filt:
        rot_ = R.from_dcm(r)
        rot_ = rot_.as_quat()
        rot.append(rot_)

    rot = np.transpose(rot)
    qx = rot[0]
    qy = rot[1]
    qz = rot[2]
    qw = rot[3]
    print np.shape(rot)
    print np.shape(rot[0])
    print np.shape(pos_filt)
    pos_filt = np.matrix(pos_filt)

    pos_filt[:,0] = pos_filt[:,0] - pos_filt[0,0]
    pos_filt[:,0] /= pos_filt[-1,0]

    pos = pos_filt


    len_ = len(pos)

    t = np.random.random(len_)
    x = np.random.random(len_)
    y = np.random.random(len_)
    z = np.random.random(len_)
    for i in range(len_):
        t[i] = pos[i,0]
        x[i] = pos[i,1]
        y[i] = pos[i,2]
        z[i] = pos[i,3]

    deg = 15
    c_x_ = np.polyfit(t, x, deg)
    c_y_ = np.polyfit(t, y, deg)
    c_z_ = np.polyfit(t, z, deg)

    d_c_x_ = []
    d_c_y_ = []
    d_c_z_ = []


    for j in range(deg - 1):
        i = deg-j
        print j
        print i
        d_c_x_.append(i * c_x_[i])
        d_c_y_.append(i * c_y_[i])
        d_c_z_.append(i* c_z_[i])


    p_x_ = np.poly1d(c_x_)
    p_y_ = np.poly1d(c_y_)
    p_z_ = np.poly1d(c_z_)

    d_p_x_ = np.poly1d(d_c_x_)
    d_p_y_ = np.poly1d(d_c_y_)
    d_p_z_ = np.poly1d(d_c_z_)


    t_ = np.linspace(0, 1, 1000)
    x_ = p_x_(t_)
    y_ = p_y_(t_)
    z_ = p_z_(t_)

    d_x_ = d_p_x_(t_)
    d_y_ = d_p_y_(t_)
    d_z_ = d_p_z_(t_)

    norm = []
    for i in range(len(d_x_)):
        norm.append(np.sqrt(pow(d_x_[i],2) + pow(d_y_[i],2) + pow(d_z_[i],2)))
        d_x_[i] /= norm[i]
        d_y_[i] /= norm[i]
        d_z_[i] /= norm[i]

    pos_poly = []
    pos_poly.append(x_)
    pos_poly.append(y_)
    pos_poly.append(z_)
    pos_poly = np.transpose(pos_poly)


    trajectory = JointTrajectory()

    allpoints = []
    for i in range(len(x_)):
        point = JointTrajectoryPoint()
        ref = [x_[i], y_[i], z_[i], d_x_[i], d_y_[i], d_z_[i]]
        point.positions = ref
        allpoints.append(point)

    trajectory.points = allpoints

    rate = rospy.Rate(0.2) # 10hz
    while not rospy.is_shutdown():
        pub.publish(trajectory)
        rate.sleep()