#!/usr/bin/env python
import controler_v1 as controller
import rospy
from time import sleep
import numpy as np
from scipy import interpolate
import pid
import trajectory_v1 as traj


def avg_dist(points):
    i = points.shape[1]
    dr = points[:,1:] - points[:,:i-1]
    ds = np.sqrt(np.sum(dr*dr, axis=0))
    #n_max = np.argmax(ds)
    #n_min = np.argmin(ds)
    avg = np.sum(ds)/(i-1)
    return avg

def i_finder(path, i = 100):
    points = path.points(i)
    avg = avg_dist(points)
    if (avg < 0.03 or avg > 0.09):
        i = int(i*(avg/0.05))
        return i_finder(path, i)
    else:
        print(i)
        return i

'''def velocity(tck,i):
    vmax = 15
    vmin = 4
    vnet = 10
    d1 = np.array(interpolate.splev(np.linspace(0,1,i), tck, der = 1))
    d2 = np.array(interpolate.splev(np.linspace(0,1,i), tck, der = 2))
    d1_norm = d1/np.sqrt(np.sum(d1*d1, axis = 0))
    d2m = np.sqrt(np.sum(d2*d2, axis = 0))
    d2m_p = d2m/(np.sum(d2m)/d2m.size)
    v = vnet/d2m_p
    v[v>vmax] = vmax
    v[v<vmin] = vmin
    v_out = d1_norm*v
    return v_out'''

class trajectory_follower:
    def __init__(self):
        self.trajectory_covered = False
        self.vmax = 2
        self.vmin = 0.3
        self.vnet = 1
        self.target_point_index = None
        self.error = None
        self.current_waypoints = None
        self.current_points = None
        self.current_tck = None
        self.current_derivative = None
        self.closest_point_index = 0
        self.number_of_points = None
        self.current_ds = None
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.angular_v = [0.0, 0.0, 0.0]
        self.derror = 5
        self.ic = controller.Flight_controller()
        self.linear_pid = pid.PID()
        self.angular_pid = pid.PID()
        self.linear_pid_factor = 0.5
        self.correction_of_path = False
        



    def data(self,waypoints,):
        waypoints = np.array(waypoints)
        self.current_waypoints = waypoints
        
        self.ms = traj.min_snap(waypoints[0], waypoints[1], waypoints[2], self.vnet)
        self.number_of_points = i_finder(self.ms)
        print('number of points')
        print(self.number_of_points)
        self.current_points = self.ms.points(self.number_of_points)
        print('current points shape')
        print(self.current_points.shape)
        self.number_of_total_points = self.current_points.shape[1]
        

    def closest_point(self):
        i = self.closest_point_index - 5
        j = self.closest_point_index + 5
        if i < 0:
            i = 0
        if j > (self.number_of_points - 1):
            j = self.number_of_points - 1
        dr = self.current_points[:, i:j] - np.reshape(np.array(self.current_pose), (3,1))
        dr_2 = dr*dr
        ds = np.sum(dr_2, axis = 0)
        index = np.argmin(ds)
        self.closest_point_index = i + index
        self.error = np.sqrt(ds[index])
        if self.closest_point_index < (self.number_of_total_points -3):
            self.target_point_index = self.closest_point_index + 1
        else:
            self.trajectory_covered = True

    def ds_finder(self):
        if self.error < 0.5:
            self.correction_of_path = False
        if(self.error < self.derror and not self.correction_of_path):
            i = 1
            ds = self.current_points[:, self.closest_point_index + i] -self.current_points[:, self.closest_point_index]
            temp_ds = np.sqrt(np.sum(ds*ds))
            
            while (temp_ds*20 < self.vmin):
                ds = self.current_points[:, self.closest_point_index + i] -self.current_points[:, self.closest_point_index]
                temp_ds = np.sqrt(np.sum(ds*ds))
                i += 1
            if(temp_ds*20 > self.vmax):
                new_derivative = (ds/temp_ds)*self.vmax/20
            else:
                new_derivative = ds
        else:
            if self.correction_of_path == False:
                self.correction_of_path = True
            temp = self.current_points[:, self.closest_point_index]
            dr = temp - np.array(self.current_pose)
            ds = np.sqrt(np.sum(dr*dr))
            if (ds*20 > self.vmax):
                new_derivative = (dr/ds)*self.vmax/20.0
            else:
                new_derivative = dr
        return new_derivative

    '''def send_velocity(self):
        if(self.error < self.derror):
            ds = self.current_ds[self.closest_point_index]
        else:
            dr = np.array(self.current_pose) - self.current_points[:,(self.closest_point_index + 1)]
            ds = np.sqrt(np.sum(dr*dr))
        v = ds*20
        self.pid_output = self.linear_pid.update(self.current_points[:,self.closest_point_index], self.current_pose)
        if( v > self.vmax):
            n = int(ds*20/self.vmax)
            if(self.error < self.derror):
                v_out = self.vmax*self.current_derivative[:,self.closest_point_index] - self.linear_pid_factor*self.pid_output
            else:
                direction = self.current_points[:,self.closest_point_index + 1] - self.current_pose
                v_out = self.vmax*(direction/np.sqrt(np.sum(direction*direction)))
                self.linear_pid.reset()
            for _ in range(0,n):
                print(v_out)
                print(self.pid_output)
                self.ic.update_target_vel(v_out, self.angular_v)
                return
        if(v < self.vmin):
            new_derivative = self.ds_finder()
            v_out = self.vmin*new_derivative - self.linear_pid_factor*self.pid_output/5
            self.ic.update_target_vel(v_out, self.angular_v)
            return
        else:
            if (self.error < self.derror):
                v_out = v*self.current_derivative[:,self.closest_point_index] - self.linear_pid_factor*self.pid_output
            else:
                direction = self.current_points[:,self.closest_point_index + 1] - self.current_pose
                v_out = v*(direction/np.sqrt(np.sum(direction*direction)))
                self.linear_pid.reset()
            self.ic.update_target_vel(v_out, self.angular_v)
            return'''

    def send_velocity(self):
        self.current_pose = np.array(self.ic.current_position)
        self.closest_point()
        dr = self.ds_finder()
        v_out = dr*20
        self.pid_output = self.linear_pid.update(self.current_points[:,self.closest_point_index], self.current_pose)
        pid_factor = self.linear_pid_factor*self.pid_output
        v_out -= pid_factor
        self.ic.update_target_vel(v_out, self.angular_v)

    def start_the_flight(self):
        self.current_pose = np.array(self.ic.current_position)
        self.closest_point()
        while not self.trajectory_covered:
            self.send_velocity()
            self.current_pose = np.array(self.ic.current_position)
            self.closest_point()
            print(self.error)
            print(-self.closest_point_index + self.number_of_total_points)






if __name__ == '__main__':
    try:
        tf = trajectory_follower()
    except:
        rospy.loginfo('Fail')
    x = [0,-3,-8,-9,-9,-8,-6]
    y = [0,3,0,-5,-7,-12,-15]
    z = [5,5,5,5,5,5,5]
    alt = 5.0
    tf.ic.toggle_arm(True)
    tf.ic.takeoff(alt)
    #sleep(5)
    #tf.ic.toggle_arm(True)
    tf.ic.set_offboard_mode()
    pose = tf.ic.current_position
    x[0] = pose[0]
    y[0] = pose[1]
    z[0] = pose[2]
    tf.data([x,y,z])
    tf.start_the_flight()

    
    