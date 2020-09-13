#!/usr/bin/env python

import numpy as np
import rospy
import csv

from pycrazyswarm import *
import uav_trajectory


def executeExternal(file,file2, allcfs, rate = 100):
    rate = rospy.Rate(rate)
    start_time = rospy.Time.now()
    with open(file) as csvfile:
        with open(file2) as csvfile2:
            spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
            spamreader2 = csv.reader(csvfile2, delimiter=' ', quotechar='|')
            for row in spamreader:
                t = row[0].split('\t')
                row1 = next(spamreader2)
                row2 = next(spamreader2)
                row3 = next(spamreader2)
                row4 = next(spamreader2)
                t_gain1 = row1[0].split('\t')
                t_gain2 = row2[0].split('\t')
                t_gain3 = row3[0].split('\t')
                t_gain4 = row4[0].split('\t')
                for cf in allcfs.crazyflies:
                    cf.cmdFullState(
                        [float(t[0]),float(t[1]),float(t[2])],
                        [float(t[6]),float(t[7]),float(t[8])],
                        [float(t[12]),float(t[13]),float(t[14])],
                        float(t[5]),
                        [float(t[9]),float(t[10]),float(t[11])],
                        float(t[4]),
                        float(t[3]))
                    cf.cmdGains(
                        [float(x) for x in t_gain1],0x01)
                    cf.cmdGains(
                        [float(x) for x in t_gain2],0x0A)
                    cf.cmdGains(
                        [float(x) for x in t_gain3],0x0C)
                    cf.cmdGains(
                        [float(x) for x in t_gain4],0x0E)
                rate.sleep()

def executeTrajectory(file, reverse = False, rate = 100, offset=np.array([0,0,0])):
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(file)
    rate = rospy.Rate(rate)

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start_time).to_sec()
        print(t)
        if t > traj.duration:
            break

        if reverse:
            e = traj.eval(traj.duration - t)
        else:
            e = traj.eval(t)
        for cf in allcfs.crazyflies:
            cf.cmdFullState(
                e.pos + np.array(cf.initialPosition) + offset,
                e.vel,
                e.acc,
                e.yaw,
                e.omega)

        rate.sleep()

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    rate = 100
    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)
    
    for cf in allcfs.crazyflies:
        cf.goTo(goal=[0,0,1],yaw=0, duration=2.0)
    timeHelper.sleep(2.5)
    #executeTrajectory("figure8.csv", False, rate, np.array([0,0,0.5]))

    executeExternal('traj.csv','Gains.csv',allcfs,rate)
    #executeTrajectory("figure8.csv", False, rate, np.array([0,0,0.5]))
    
    timeHelper.sleep(3)

    for cf in allcfs.crazyflies:
        cf.goTo(goal=[0,0,0.1],yaw=0, duration=2.0)
    timeHelper.sleep(2.5)

    allcfs.land(targetHeight=0, duration=2.0)
    timeHelper.sleep(2.5)

    for i in range(0, 100):
        for cf in allcfs.crazyflies:
            cf.cmdStop()
