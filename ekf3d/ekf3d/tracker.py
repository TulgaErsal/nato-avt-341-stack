from .trajectory import Trajectory
from .box_op import *
import numpy as np


class Tracker3D:
    def __init__(self,
                    box_type='Kitti',
                    config = None):
        """
        initialize the the 3D tracker
        """
        self.config = config
        self.current_timestamp = None
        self.current_bbs = None
        self.current_centroid = None
        self.box_type = box_type

        self.label_seed = 0

        self.active_trajectories = {}
        self.dead_trajectories = {}

    def tracking(self,
                 bbs_3D = None,
                 centroid = None,
                 timestamp = None
                 ):
        self.current_bbs = bbs_3D
        self.current_centroid = centroid
        self.current_timestamp = timestamp

        self.trajectores_prediction()
 
        assert self.current_bbs is not None
        assert self.current_centroid is not None

        if self.current_bbs.size == 0 and self.current_centroid.size == 0:
            return np.zeros(shape=(0,7)),np.zeros(shape=(0))
        else:
            print ("updating")
            self.current_bbs = convert_bbs_type(self.current_bbs,self.box_type)
            bbx_ids,centroid_ids = self.association()
            bbs,ids = self.trajectories_update(bbx_ids,centroid_ids)

            return np.array(bbs),np.array(ids)



    def trajectores_prediction(self):
        """
        predict the possible state of each active trajectories, if the trajectory is not updated for a while,
        it will be deleted from the active trajectories set, and moved to dead trajectories set
        Returns:

        """
        if len(self.active_trajectories) == 0 :
            return
        else:
            dead_track_id = []

            for key in self.active_trajectories.keys():
                print("traj",key,"missed",self.active_trajectories[key].consecutive_missed_num,"vs",self.config.max_prediction_num)
                if self.active_trajectories[key].consecutive_missed_num>=self.config.max_prediction_num:
                    dead_track_id.append(key)
                    continue
                if len(self.active_trajectories[key])-self.active_trajectories[key].consecutive_missed_num == 1 \
                    and len(self.active_trajectories[key])>= self.config.max_prediction_num_for_new_object :
                    dead_track_id.append(key)
                self.active_trajectories[key].state_prediction(self.current_timestamp)

            for id in dead_track_id:
                tra = self.active_trajectories.pop(id)
                self.dead_trajectories[id]=tra

    def compute_bbx_cost_map(self):
        """
        compute the cost map between detections and predictions
        Returns:
              cost, array(N,M), where N is the number of detections, M is the number of active trajectories
              all_ids, list(M,), the corresponding IDs of active trajectories
        """

        all_ids = []

        all_predictions = []
        all_prediction_covs = []
        all_detections = []

        for key in self.active_trajectories.keys():
            all_ids.append(key)
            state = np.array(self.active_trajectories[key].trajectory[self.current_timestamp].predicted_state)
            state = state.reshape(-1)

            pd = state.shape[0]
            cov = np.array(self.active_trajectories[key].trajectory[self.current_timestamp].predicted_covariance)
            cov = cov.reshape((pd,pd))

            all_predictions.append(state)
            all_prediction_covs.append(cov)

        for i in range(len(self.current_bbs)):
            box = self.current_bbs[i]
            label=1
            new_tra = Trajectory(init_bb=box,
                                 init_timestamp=self.current_timestamp,
                                 label=label,
                                 config = self.config)

            state = new_tra.trajectory[self.current_timestamp].predicted_state
            state = state.reshape(-1)
            all_detections.append(state)

        all_detections = np.array(all_detections)
        all_predictions = np.array(all_predictions)
        all_prediction_covs = np.array(all_prediction_covs)

        print ("predictions:\n",all_predictions)    
        print ("detections:\n",all_detections)    

        det_len = len(all_detections)
        pred_len = len(all_predictions)

        print ("bbx cost map grid size",det_len,pred_len)
        if pred_len*det_len == 0:
            costs = np.zeros((pred_len,det_len))
        else:
            costs = np.ones((pred_len,det_len))*1e6

            all_detections = all_detections.reshape((det_len,-1))
            all_predictions = all_predictions.reshape((pred_len,-1))

            pd = all_predictions.shape[-1]
            all_prediction_covs = all_prediction_covs.reshape((pred_len,pd,pd))

            for ipred in range(pred_len):
                for jdet in range(det_len):

                    mu2x = (all_detections[jdet,0:2]-all_predictions[ipred,0:2]).reshape((-1,1))
                    S = all_prediction_covs[ipred,0:2,0:2]
                    dis = np.sqrt(mu2x.T @ np.linalg.inv(S) @ mu2x)
                    print ("mu2x:\n",mu2x,"\nS:\n",S,"\ndis:",dis)

                    costs[ipred,jdet] = dis

        print ("COSTS BBX\n",costs)            
 
        return costs,all_ids

    def compute_centroid_cost_map(self):

        print ("computing centroid cost map for",self.current_centroid.shape)

        all_ids = []

        all_predictions = []
        all_prediction_covs = []
        all_detections = []

        for key in self.active_trajectories.keys():
            all_ids.append(key)
            state = np.array(self.active_trajectories[key].trajectory[self.current_timestamp].predicted_state)
            state = state.reshape(-1)

            pd = state.shape[0]
            cov = np.array(self.active_trajectories[key].trajectory[self.current_timestamp].predicted_covariance)
            cov = cov.reshape((pd,pd))

            all_predictions.append(state)
            all_prediction_covs.append(cov)

        if self.current_centroid.shape[0] > 0:
              
            pose = self.current_centroid
            box = None
            label=1
            new_tra = Trajectory(init_bb=box,
                                 init_centroid=pose,
                                 init_timestamp=self.current_timestamp,
                                 label=label,
                                 config = self.config)

            state = new_tra.trajectory[self.current_timestamp].predicted_state
            state = state.reshape(-1)
            all_detections.append(state)

        all_detections = np.array(all_detections)
        all_predictions = np.array(all_predictions)
        all_prediction_covs = np.array(all_prediction_covs)

        print ("predictions:\n",all_predictions)    
        print ("detections:\n",all_detections)    

        det_len = len(all_detections)
        pred_len = len(all_predictions)

        print ("centroid cost map grid size",det_len,pred_len)
        if pred_len*det_len == 0:
            costs = np.zeros((pred_len,det_len))
        else:
            costs = np.ones((pred_len,det_len))*1e6

            all_detections = all_detections.reshape((det_len,-1))
            all_predictions = all_predictions.reshape((pred_len,-1))

            pd = all_predictions.shape[-1]
            all_prediction_covs = all_prediction_covs.reshape((pred_len,pd,pd))

            for ipred in range(pred_len):
                for jdet in range(det_len):

                    mu2x = (all_detections[jdet,0:2]-all_predictions[ipred,0:2]).reshape((-1,1))
                    S = all_prediction_covs[ipred,0:2,0:2]
                    dis = np.sqrt(mu2x.T @ np.linalg.inv(S) @ mu2x)
                    print ("mu2x:\n",mu2x,"\nS:\n",S,"\ndis:",dis)

                    costs[ipred,jdet] = dis

        print ("COSTS UWB\n",costs)            
 
        return costs,all_ids

    def association(self):
        """
        greedy assign the IDs for detected state based on the cost map
        Returns:
            ids, list(N,), assigned IDs for boxes, where N is the input boxes number
        """
        bbx_cost_map, all_ids = self.compute_bbx_cost_map()
        bbx_assignments = [None for i in range(len(self.current_bbs))]
        for i in range(len(self.active_trajectories.keys())):
            
            if bbx_cost_map.size:
                minval = np.min(bbx_cost_map[i,:])
                arg_min = np.argmin(bbx_cost_map[i,:])
            else:
                minval = 1e6
                arg_min = None

            print("traj",i,"bbx cost min",minval)
            if minval < self.config.bind_thr and minval == np.min(bbx_cost_map[:,arg_min]):
                print("traj",i,"bind to bbx",arg_min)
                bbx_assignments[arg_min] = all_ids[i]
                bbx_cost_map[:,arg_min] = 1e6
            elif minval < self.config.bind_thr:
                print("traj",i,"is not dominant for bbx",arg_min)
            else:
                print("traj",i,"no bbx match")


        centroid_cost_map, all_ids = self.compute_centroid_cost_map()
        centroid_assignments = []
        if self.current_centroid.shape[0] > 0:
            i = 0

            if centroid_cost_map.size:
                minval = np.min(centroid_cost_map[i])
                arg_min = np.argmin(centroid_cost_map[i])
            else:
                minval = 1e6

            print(i,"centroid cost min",minval)
            if minval < self.config.bind_thr:
                print(i,"centroid bind to",arg_min)
                centroid_assignments.append(all_ids[arg_min])
                centroid_cost_map[:,arg_min] = 1e6
            else:
                print(i,"centroid no match")
                centroid_assignments.append(None)

        return bbx_assignments, centroid_assignments


    def trajectories_update(self,bbx_ids,centroid_ids):
        """
        update a exiting trajectories based on the association results, or init a new trajectory
        Args:
            ids: list or array(N), the assigned ids for boxes
        """
        assert len(bbx_ids) == len(self.current_bbs)
        assert len(centroid_ids) == int(self.current_centroid.shape[0])

        used_bbx_ix = []
        used_centroid_jx = []

        print ("active trajectories:",self.active_trajectories.keys())
        print ("bbx idx:",bbx_ids)
        print ("centroid idx:",centroid_ids)
  
        # update active trajectories
        for label in self.active_trajectories.keys():

            bbx_i = None
            for ii in range(len(self.current_bbs)):
                if label == bbx_ids[ii]:
                    bbx_i = ii
                    break
            centroid_j = None
            if self.current_centroid.shape[0] > 0:
                jj = 0
                if label == centroid_ids[jj]:
                    centroid_j = jj
            
            if bbx_i is None and centroid_j is None:
                print("unpaired traj %d" % (label))
                continue
            
            box = None
            centroid = None
            if bbx_i is not None:
                print("bbox %d paired to traj %d" % (bbx_i,label))
                used_bbx_ix.append(bbx_i)
                box = self.current_bbs[bbx_i]
            if centroid_j is not None:
                print("centroid %d paired to traj %d" % (centroid_j,label))
                used_centroid_jx.append(centroid_j)
                centroid = self.current_centroid.reshape((-1))

            track = self.active_trajectories[label]
            track.state_update(
                 bb=box,
                 centroid=centroid,
                 timestamp=self.current_timestamp)

        # process bbx with no assigned trajectory
        for i in range(len(self.current_bbs)):
            if len(self.active_trajectories.keys()) >= self.config.max_trajectories:
                print ("ignore off active trajectory detection")
                continue
            if i in used_bbx_ix:
                print ("bbox",i,"already assigned")
                continue
            assert bbx_ids[i] is None
            
            label = self.label_seed
            self.label_seed+=1
            print("new bbox %d over label %d" % (i,label))

            box = self.current_bbs[i]
            centroid = np.zeros((0,4))    

            new_tra = Trajectory(init_bb=box,
                                     init_centroid=centroid,
                                     init_timestamp=self.current_timestamp,
                                     label=label,
                                     config = self.config)
            self.active_trajectories[label] = new_tra


        if self.current_centroid.shape[0] > 0:
            j = 0
            if len(self.active_trajectories.keys()) >= self.config.max_trajectories:
                print ("ignore off active trajectory detection")
            if j in used_centroid_jx:
                print ("centroid",j,"already assigned")
                pass
            else:
                label = self.label_seed
                self.label_seed+=1
                print("new yolo over label %d" % (label))

                box = None
                centroid = self.current_centroid   

                new_tra = Trajectory(init_bb=box,
                                     init_centroid=centroid,
                                     init_timestamp=self.current_timestamp,
                                     label=label,
                                     config = self.config)
                self.active_trajectories[label] = new_tra

        #if len(valid_bbs)==0:
        #    return np.zeros(shape=(0,7)),np.zeros(shape=(0))
        #else:
        #    return np.array(valid_bbs),np.array(valid_ids)
        return np.zeros(shape=(0,7)),np.zeros(shape=(0))





