#!/usr/bin/env python

from os import listdir
from os.path import isfile, join

import numpy as np
import pandas as pd 

import scipy, scipy.optimize, scipy.signal
from scipy.spatial.transform import Rotation as R


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import itertools

import csv

def main():

    folderpath = '/home/franka/exploration_pc/'

    onlyfiles = [(f, join(folderpath, f)) for f in listdir(folderpath) if isfile(join(folderpath, f))]



    # data = {}
    # for tuple_ in onlyfiles:
    #     data_1 = pd.read_csv(tuple_[1])
    #     data[tuple_[0]] = data_1
    # print data.keys()

    # print data['otee.csv'].iloc[1]['field10']

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
    for t, trans in zip(data_1, matrices):
        pos_ = np.dot(np.matrix(trans), np.array([0., 0., 0.04, 1.]))
        pos_ = pos_.tolist()[0]
        pos.append([t[0], pos_[0], pos_[1], pos_[2]])
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

    print np.shape(rot)
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
        t[i] = 1. - pos[i,0]
        x[i] = pos[i,1]
        y[i] = pos[i,2]
        z[i] = pos[i,3]

    deg = 11
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



    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x, y, z)
    ax.scatter(x_, y_, z_, c='red')
    plt.axis('equal')
    plt.show()

    dir_poly = []
    dir_poly.append(d_x_)
    dir_poly.append(d_y_)
    dir_poly.append(d_z_)
    dir_poly = np.transpose(dir_poly)

    # pos_poly = []
    # pos_poly.append(x_)
    # pos_poly.append(y_)
    # pos_poly.append(z_)
    # pos_poly = np.transpose(pos_poly)

    # my_dict = {}
    # my_dict['t'] = pos_filt[:,0].tolist()
    # my_dict['x'] = pos_filt[:,1].tolist()
    # my_dict['y'] = pos_filt[:,2].tolist()
    # my_dict['z'] = pos_filt[:,3].tolist()
    # df = pd.DataFrame.from_dict(my_dict, orient="index")
    # df.to_csv("pos_pipe_dict.csv")

    # df = []
    # df = pd.read_csv("pos_pipe_dict.csv", index_col=0)
    # d = df.to_dict("split")
    # d = dict(zip(d["index"], d["data"]))

    # rot = np.transpose(rot)
    # print np.shape(rot[0,:])

    # my_dict = {}
    # my_dict['qx'] = rot[0,:].tolist()
    # my_dict['qy'] = rot[1,:].tolist()
    # my_dict['qz'] = rot[2,:].tolist()
    # my_dict['qw'] = rot[3,:].tolist()
    # df = []
    # df = pd.DataFrame.from_dict(my_dict, orient="index")
    # df.to_csv("ori_pipe_dict.csv")


    # df = []
    # df = pd.read_csv("ori_pipe_dict.csv", index_col=0)
    # d = df.to_dict("split")
    # d = dict(zip(d["index"], d["data"]))
    # print d.keys()

    # my_dict = {}
    # my_dict['x'] = pos_poly[:,0].tolist()
    # my_dict['y'] = pos_poly[:,1].tolist()
    # my_dict['z'] = pos_poly[:,2].tolist()
    # df = pd.DataFrame.from_dict(my_dict, orient="index")
    # df.to_csv("pos_poly_dict.csv")

    my_dict = {}
    my_dict['dx'] = dir_poly[:,0].tolist()
    my_dict['dy'] = dir_poly[:,1].tolist()
    my_dict['dz'] = dir_poly[:,2].tolist()
    df = pd.DataFrame.from_dict(my_dict, orient="index")
    df.to_csv("dir_poly_dict.csv")



    # writer = csv.writer(writeFile)
        # writer.writerows(pos_filt)

    # with open('ori_pipe.csv', 'w') as writeFile:
    #     writer = csv.writer(writeFile)
    #     writer.writerows(rot)


    # with open('pos_poly.csv', 'w') as writeFile:
    #     writer = csv.writer(writeFile)
    #     writer.writerows(pos_poly)

if __name__ == '__main__':

    main()