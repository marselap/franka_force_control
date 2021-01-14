#!/usr/bin/env python

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseArray, Point, Pose
from franka_msgs.msg import FrankaState

from sklearn.cluster import AgglomerativeClustering
import numpy as np
import math


class MoveGroupPythonIntefacelistener(object):
  def __init__(self):

    rospy.init_node('franka_cluster_points',
                    anonymous=True)

    subs1 = rospy.Subscriber('exploration_direction', Point, self.exploration_callback)
    subs2 = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.franka_state_callback)
    subs3 = rospy.Subscriber('cluster', Bool, self.waypoints_request_cb)

    subs = rospy.Subscriber('trajectory_success', Bool, self.success_callback)

    self.waypoints_pub = rospy.Publisher('waypoints', PoseArray, queue_size=1)

    self.save_points = False

    self.positions = []

    self.experiments = []

    self.alldata = {}
    self.indices = 0

    self.waypoints_request = False

    self.xdir = 1.

  def success_callback(self, data):

    self.experiments = []

  def exploration_callback(self, data):

    ref = [data.x, data.y, data.y]
    if any(not r == 0 for r in ref):
      self.save_points = True    
    else:
      self.save_points = False
    if data.x < 0.:
      self.xdir = -1.
    elif data.x > 0.:
      self.xdir = 1.


  def franka_state_callback(self, data):

    if self.save_points:
      self.positions.append(data.O_T_EE)

  def waypoints_request_cb(self, data):
    self.waypoints_request = True

  def cluster(self):

    data = self.experiments

    lens = 0
    for exp in self.experiments:
      lens += len(exp)
    
    data = np.zeros((lens, 3))
    
    lens_spent = 0

    for i,exp in enumerate(self.experiments):
      data[lens_spent:lens_spent+len(exp), :] = exp
      lens_spent += len(exp)
    
    data = np.asarray(data)

    print np.shape(data)

    ranges = []
    for i in range(3):
      ranges.append(max(data[:,i])-min(data[:,i]))

    
    print data[1,:]
    print ranges

    n_clusters = int(np.linalg.norm(ranges)/0.02)

    print("n clusters: ", n_clusters)

    model = AgglomerativeClustering(linkage="complete", n_clusters = n_clusters)

    model.fit(data)

    lset = set(model.labels_)
    centres = {}
    for label in lset:
        indices = [i for i, x in enumerate(model.labels_) if x == label]
        cluster_members = np.asarray([data[i,:] for i in indices])
        center = find_mean_member(cluster_members)
        centres[label] = center

    order_centres = get_ordered_centres(data, model.labels_)

    return centres, order_centres




  def get_orientations(self, waypoints, order):


    # q_8toee = [ 0.92387769054, -0.38268051139, 0.0, 0.0]
    q_8toee = [0.92387769054, 0., 0., -0.38268051139]
    link8toee = quaternion_matrix(q_8toee)
    link8toee = np.asarray(link8toee)
    
    link8toee[0,3] = 0.0
    link8toee[1,3] = 0.0
    link8toee[2,3] = 0.10339999944


    link8toee = np.linalg.inv(link8toee)


    orientations = {}
    ori_q = {}
    for i in range(len(order)-1):
      p1 = waypoints[order[i]]
      p2 = waypoints[order[i+1]]

      x = p2 - p1
      x /= np.linalg.norm(x)

      x = self.xdir * x

      z_glob = np.asarray([0,0,1])
      y = np.cross(x, z_glob)

      y = y / np.linalg.norm(y)
      
      z = np.cross(x, y)

      z = z / np.linalg.norm(z)

      m = np.zeros((3,3))
      m[0,:] += x
      m[1,:] += y
      m[2,:] += z
      m = np.transpose(m)

      t = np.eye(4)
      t[0:3,0:3] = m
      t[0:3,3] = p1

      
      # new_t = np.dot(linkeeto8, t)
      new_t = np.dot(t, link8toee)



      m = new_t[0:3,0:3]
      waypoints[order[i]] = new_t[0:3,3]

      q = quaternion_from_matrix(m)

      print"_________"
      print x
      print m
      print q

      orientations[order[i]] = m
      ori_q[order[i]] = q



    # orientations[order[-1]] = orientations[order[-2]]

    ori_q[order[-1]] = ori_q[order[-2]]


    return ori_q



def find_mean_member(cluster_members):

    cm = cluster_members
    return np.asarray([np.mean(cm[:,0]), np.mean(cm[:,1]), np.mean(cm[:,2])])


def get_ordered_centres(all_points, labels):
    lset = set(labels)
    lsetsize = len(lset)

    ordered_centres = []

    i = 0
    while len(ordered_centres) < lsetsize:
        if not labels[i] in ordered_centres:
            ordered_centres.append(labels[i])
        i += 1

    return ordered_centres 


def quaternion_matrix(quaternion):

    _EPS = np.finfo(float).eps * 4.0

    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def quaternion_from_matrix(matrix, isprecise=False):
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q

def make_msg(waypoints, ori, order):

  wps = PoseArray()


  print order

  order2 = order[:-1]
  for i in order2:
    pose = Pose()

    pose.position.x = waypoints[i][0]
    pose.position.y = waypoints[i][1]
    pose.position.z = waypoints[i][2]

    pose.orientation.w = ori[i][0]
    pose.orientation.x = ori[i][1]
    pose.orientation.y = ori[i][2]
    pose.orientation.z = ori[i][3]

    wps.poses.append(pose)

  return wps

def main():
  try:


    listener = MoveGroupPythonIntefacelistener()

    r = rospy.Rate(1000)

    while not rospy.is_shutdown():

        try:
            if listener.save_points == False and len(listener.positions) > 0:
              # ako je gotovo gibanje i postoje snimljene neke tocke
              # spremi tocke i radi clustere po potrebi

              nppos = np.asarray(listener.positions)
              pos = nppos[:,12:15]
              listener.experiments.append(pos)

              listener.alldata[listener.indices] = listener.positions

              listener.indices += 1
              listener.positions = []

            if listener.waypoints_request:

              waypoints, order = listener.cluster()

              orientations = listener.get_orientations(waypoints, order)

              waypoints_msg = make_msg(waypoints, orientations, order)

              listener.waypoints_pub.publish(waypoints_msg)

              listener.waypoints_request = False


            r.sleep()
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

