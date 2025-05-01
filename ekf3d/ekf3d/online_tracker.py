#!/usr/bin/env python3
from .ros_helpers import *

import threading
import sys
import time
import rospkg

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from .config import cfg, cfg_from_yaml_file
from .tracker import Tracker3D
from .trajectory import bbox_from_state
from .box_op import bbs_to_corners, corners_to_bbs

##################################################################
def q2r(q):
    """
    method to convert quaternion into rotation matrix
    @input: q .. quaternion (qx,qy,qz,qw)
    @output: R .. rotation matrix
    """
    q = q[[3, 0, 1, 2]]
    R = [[q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2, 2 * (q[1] * q[2] - q[0] * q[3]),
          2 * (q[1] * q[3] + q[0] * q[2])],
         [2 * (q[1] * q[2] + q[0] * q[3]), q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2,
          2 * (q[2] * q[3] - q[0] * q[1])],
         [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]),
          q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2]]
    return np.array(R)

##################################################################

class Entry:

    def __init__(self, ts, val):
        self.ts = ts
        self.val = val

class TrackerNode(NodeHelper):

    def __init__(self, name):
        super().__init__(name)
        config_path = self._getparam("cfg_path", "NONE.yaml")  
        self.config =  cfg_from_yaml_file(config_path,cfg)
        self.tracker = Tracker3D(box_type="Waymo", config = self.config)
        self.tracker_lock = threading.Lock()
        self.frame_no = -1

        self.q_lidar_pcd = []
        self.q_yolo_pcd = []
        self.frame_id = "map"
        self.ts = -1
        self.last_update_ts = -1

    def track(self, det_scores, objects, centroid, pose):    

        self.frame_no += 1

        mask = (det_scores > self.config.input_score)[:,0]
        objects = objects[mask,...]
        det_scores = det_scores[mask,...]

        self.tracker_lock.acquire()
        bbs_current, ids_current = self.tracker.tracking(
                objects[:,:7],
                centroid[:, :4],
                timestamp=self.frame_no
                )
        self.tracker_lock.release()


        ids_active, bbs_active = self.report(0.)
        return ids_active, bbs_active
        #return ids_current, bbs_current

    def report(self, ts_offset):    

        self.tracker_lock.acquire()
        self._loginfo("reporting trajectoris at last update + %lf [s]" % ts_offset) 
        self._loginfo("active trajectories: %d" % len(self.tracker.active_trajectories)) 
        
        bbs_active = []
        ids_active = []
        for id,trajectory in self.tracker.active_trajectories.items():
            ids_active.append(id)
            last_frame = max(trajectory.trajectory.keys())
            self._loginfo ("active trajectory %d at frame %d vs global %d" % (id,last_frame,self.frame_no))
            bbs = bbox_from_state(trajectory.trajectory[last_frame].predicted_state, ts_offset=ts_offset)
            bbs_active.append(bbs)
 
        self.tracker_lock.release()


        ids_active = np.array(ids_active)
        #print (bbs_active)
        if len(bbs_active):
            bbs_active = np.stack(bbs_active,axis=0)
        else:
            bbs_active = np.zeros((0,7))

        #print(bbs_active.shape,ids_active.shape)

        with np.printoptions(precision=2, suppress=True):
            print("report:",ids_active.transpose())
            print (bbs_active.shape,bbs_active)

        return ids_active, bbs_active
        #return ids_current, bbs_current

    def get_entry(self, ts, q):

        last_diff = np.inf
        best_i = -1

        for i in range(len(q)):
            diff = np.abs(q[i].ts - ts)
            if diff > last_diff:
                break
            else:
                best_i = i
                last_diff = diff

        if last_diff < self.queue_tolerance:
            ent = q[best_i]
            self._loginfo("val for tracker with %lf diff vs %lf" % (last_diff,self.queue_tolerance))
            q = q[:best_i+1]        
        else:
            ent = None
            self._loginfo("no valid val for tracker, best diff %lf vs %lf" % (last_diff,self.queue_tolerance))

        return ent 

    def callback_lidar_bbx(self, msg_pc2):

        start = self._now()

        self._loginfo("processing lidar detected bbx with %lf delay" % (start - ros_to_sec(msg_pc2.header.stamp))) 

        ts = ros_to_sec(msg_pc2.header.stamp)
        pcd_np = np.array([[p[0],p[1],p[2],p[3]] for p in pc2.read_points(msg_pc2, field_names=["x","y","z","score"], skip_nans=True)])
        n = pcd_np.shape[0] // 8
        pcd_np = pcd_np.reshape((n,8,4))

        self.q_lidar_pcd.append(Entry(ts, pcd_np))
        #self.ts = ts
        #self.real_ts = self._now()
        #self.frame_id = msg_pc2.header.frame_id

        if self.update_from_lidar:
            self._loginfo ("update from lidar")
            self.update_at_ts(ts)

    def callback_yolo_centroids(self, msg_pc2):

        start = self._now()

        self._loginfo("processing yolo detection centroids with %lf delay" % (start - ros_to_sec(msg_pc2.header.stamp))) 

        ts = ros_to_sec(msg_pc2.header.stamp)
        pcd_np = np.array([[p[0],p[1],p[2],0] for p in pc2.read_points(msg_pc2, field_names=["x","y","z"], skip_nans=True)])
        n = pcd_np.shape[0]
        pcd_np = pcd_np.reshape((n,4))

        yaw = 0
        for i in range(len(self.q_yolo_pcd)-1,0,-1):
            if ts - self.q_yolo_pcd[i].ts > self.yolo_yaw_window:
                break
            elif self.q_yolo_pcd[i].val.size:
                for j in range(n):
                    oj = min(j,self.q_yolo_pcd[i].val.shape[0]-1) # yolo should keep track
                    dx = (pcd_np[j,0] - self.q_yolo_pcd[i].val[oj,0]) 
                    dy = (pcd_np[j,1] - self.q_yolo_pcd[i].val[oj,1])
                    dd = np.sqrt(dx*dx+dy*dy)
                    if dd / (ts - self.q_yolo_pcd[i].ts) > self.yolo_yaw_minv:
                        yaw = np.arctan2(dy,dx)
                    else:
                        yaw = self.q_yolo_pcd[i].val[oj,3]
        pcd_np[:,3] = yaw            

        self.q_yolo_pcd.append(Entry(ts, pcd_np))
        
        if self.update_from_yolo:
            self._loginfo ("update from yolo")
            self.update_at_ts(ts)

    def callback_tick_update(self, e):
        ts = self._now() - self.update_delay
        if self.update_from_tick and ts > 0:
            self._loginfo ("update from tick")
            self.update_at_ts(ts)
        elif self.update_from_tick:
            self._loginfo ("update from tick skipped")

    def update_at_ts(self, ts):

        start = time.time()

        self._loginfo("updating tracker at %lf" % ts)

        self._loginfo("->q size %d" % (len(self.q_lidar_pcd)))
        pcd_entry = self.get_entry(ts, self.q_lidar_pcd)
        yolo_pcd_entry = self.get_entry(ts, self.q_yolo_pcd)
        self._loginfo("<-q size %d %d" % (len(self.q_lidar_pcd),len(self.q_yolo_pcd)))

        if pcd_entry is not None:
            pcd_np = pcd_entry.val.reshape((-1,8,4))
            self._loginfo("-- found bbox n=%d at %lf (delta %lf)" % (pcd_np.shape[0], pcd_entry.ts,ts - pcd_entry.ts))
        else:
            pcd_np = np.zeros((0,8,4))


        if yolo_pcd_entry is not None:
            yolo_pcd_np = yolo_pcd_entry.val[:1,:].reshape((-1,4))
            self._loginfo("-- found yolo centroid n=%d at %lf (delta %lf)" % (yolo_pcd_np.shape[0], yolo_pcd_entry.ts, ts - yolo_pcd_entry.ts))
        else:
            yolo_pcd_np = np.zeros((0,4))

        pose = np.eye(4)

        scores = pcd_np[...,3].reshape((-1,8,1))[:,0,:]
        corners = pcd_np[...,:3].reshape((-1,8,3))

        bbs = corners_to_bbs(corners)

        track_idx, track_bbs = self.track(scores, bbs, yolo_pcd_np, pose) # bbs reported regirested based on pose
        self.last_update_ts = ts

        self._loginfo("processed detected objects in %lf s" % (time.time() - start)) 

        if not self.independent_report:
            #print ("track_bbs",track_bbs.shape)
            track_corners = bbs_to_corners(track_bbs)

            id_pts = np.stack([track_idx for _ in range(8)],axis=1).reshape((-1,1))
            corner_pts = track_corners.reshape((-1,3))
            vals = np.concatenate([corner_pts,id_pts],axis=1).astype('float32')

            field_x = PointField()
            field_x.name = 'x'
            field_x.offset = 0
            field_x.count = 1
            field_x.datatype = PointField.FLOAT32

            field_y = PointField()
            field_y.name = 'y'
            field_y.offset = 4
            field_y.count = 1
            field_y.datatype = PointField.FLOAT32

            field_z = PointField()
            field_z.name = 'z'
            field_z.offset = 8
            field_z.count = 1
            field_z.datatype = PointField.FLOAT32

            field_id = PointField()
            field_id.name = 'id'
            field_id.offset = 12
            field_id.count = 1
            field_id.datatype = PointField.FLOAT32

            fields = [
                    field_x,
                    field_y,
                    field_z,
                    field_id,
                    ]

            header = Header()
            header.stamp = rosstamp(ts)
            header.frame_id = self.frame_id
            msg_pred = pc2.create_cloud(header,fields,vals)
            self.pub_pred.publish(msg_pred)

            self._loginfo("report done in %lf s" % (time.time() - start))

    def callback_tick_publish(self, e):

        if self.independent_report:
            ts = self._now()
            ts_offset = (ts - self.update_delay) - self.last_update_ts

            pose = np.eye(4)

            track_idx, track_bbs = self.report(ts_offset) # bbs reported

            #print ("track_bbs",track_bbs.shape)
            track_corners = self.bbs_to_corners(track_bbs)

            id_pts = np.stack([track_idx for _ in range(8)],axis=1).reshape((-1,1))
            corner_pts = track_corners.reshape((-1,3))
            vals = np.concatenate([corner_pts,id_pts],axis=1).astype('float32')
            
            field_x = PointField()
            field_x.name = 'x'
            field_x.offset = 0
            field_x.count = 1
            field_x.datatype = PointField.FLOAT32

            field_y = PointField()
            field_y.name = 'y'
            field_y.offset = 4
            field_y.count = 1
            field_y.datatype = PointField.FLOAT32

            field_z = PointField()
            field_z.name = 'z'
            field_z.offset = 8
            field_z.count = 1
            field_z.datatype = PointField.FLOAT32

            field_id = PointField()
            field_id.name = 'id'
            field_id.offset = 12
            field_id.count = 1
            field_id.datatype = PointField.FLOAT32

            fields = [
                    field_x,
                    field_y,
                    field_z,
                    field_id,
                    ]

            header = Header()
            if self.ts > 0:
                header.stamp = rosstamp(self.ts+ts_offset)
            else:
                header.stamp = rosstamp(self._now())
            header.frame_id = self.frame_id
            msg_pred = pc2.create_cloud(header,fields,vals)
            self.pub_pred.publish(msg_pred)

            self._loginfo("report done")




def main():
    
    rosinit(name='tracker_wrapper', anonymous=True, args=sys.argv)

    capture_node = TrackerNode(name='tracker_wrapper')

    publish_freq = capture_node._getparam("report_freq", 1.0)  
    update_freq = capture_node._getparam("update_freq", 1.0)  

    capture_node.frame_id = capture_node._getparam("frame_id", "map")  
    capture_node.independent_report = capture_node._getparam("independent_report", False)  
    capture_node.pub_pred = capture_node._create_publisher("tracked_objects", PointCloud2, queue_size=1)
    capture_node.update_delay = capture_node._getparam("update_delay", 2.0)  
    capture_node.queue_tolerance = capture_node._getparam("queue_tolerance", 0.5/(update_freq))  
    capture_node.yolo_yaw_window = capture_node._getparam("yolo_yaw_window", 10.0)  
    capture_node.yolo_yaw_minv = capture_node._getparam("yolo_yaw_minv", 0.3)  
    capture_node.update_from_lidar = capture_node._getparam("update_from_lidar", False)  
    capture_node.update_from_yolo = capture_node._getparam("update_from_yolo", False)  
    capture_node.update_from_tick = capture_node._getparam("update_from_tick", False)  

    capture_node._loginfo("building subscribers")

    callback_lidar_bbx = lambda m : capture_node.callback_lidar_bbx(m)
    capture_node._create_subscriber("detected_bbx", PointCloud2, callback_lidar_bbx, queue_size=10)

    callback_yolo_centroids = lambda m : capture_node.callback_yolo_centroids(m)
    capture_node._create_subscriber("detection_centroids", PointCloud2, callback_yolo_centroids, queue_size=10)

    callback_tick_publish = lambda e : capture_node.callback_tick_publish(e)
    capture_node._create_timer(rosstamp(1/publish_freq), callback_tick_publish)

    callback_tick_update = lambda e : capture_node.callback_tick_update(e)
    capture_node._create_timer(rosstamp(1/update_freq), callback_tick_update)

    capture_node._loginfo("starting spin")
    capture_node._spin()


if __name__ == "__main__":
    main()
