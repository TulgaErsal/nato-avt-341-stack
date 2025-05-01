import numpy as np
from .object import Object

def build_detected_state(bb=None,centroid=None):

    if bb is None:
        bb = np.zeros(7)
    else:
        bb = np.copy(bb)
    if centroid is None:
        centroid = np.zeros(4)
    else:
        centroid = np.copy(centroid)
    bb = bb.reshape((1,-1))    
    centroid = centroid.reshape((1,-1))    

    detected_state_template = np.zeros(shape=(Trajectory.Z_DIM)) 
    if bb is not None and np.size(bb):
        detected_state_template[Trajectory.Z_ID_LIDAR_X:Trajectory.Z_ID_LIDAR_Z+1] = bb[0,:3] #init x,y,z
        detected_state_template[Trajectory.Z_ID_LIDAR_YAW] = bb[0,6] #yaw
    if centroid is not None and np.size(centroid):
        detected_state_template[Trajectory.Z_ID_CAMERA_X:Trajectory.Z_ID_CAMERA_Z+1] = centroid[0,:3] #init x,y,z
        detected_state_template[Trajectory.Z_ID_CAMERA_FAKEYAW] = centroid[0,3] #init x,y,z
    detected_state_template = np.mat(detected_state_template).T
    return detected_state_template

def bbox_from_state(state, ts_offset=0):
    x = state[Trajectory.X_ID_X].A1[0]
    y = state[Trajectory.X_ID_Y].A1[0]
    z = state[Trajectory.X_ID_Z].A1[0]
    v_fwd = state[Trajectory.X_ID_V_FWD].A1[0]
    yaw = state[Trajectory.X_ID_YAW].A1[0]

    bb = np.array([x,y,z,1,1,1,yaw])
    bb_dot = np.array([np.sin(np.pi/2-yaw)*v_fwd,np.sin(yaw)*v_fwd,0])

    bb[:3] += ts_offset*bb_dot

    return bb

class Trajectory:

    # internal state dims
    X_DIM = 6
    X_ID_X = 0
    X_ID_Y = 1
    X_ID_Z = 2
    X_ID_YAW = 3
    X_ID_V_FWD = 4
    X_ID_V_ANG = 5

    # observation dims
    Z_DIM = 8
    Z_ID_LIDAR_X = 0
    Z_ID_LIDAR_Y = 1
    Z_ID_LIDAR_Z = 2 
    Z_ID_LIDAR_YAW = 3
    Z_ID_CAMERA_X = 4
    Z_ID_CAMERA_Y = 5
    Z_ID_CAMERA_Z = 6
    Z_ID_CAMERA_FAKEYAW = 7

    def __init__(self,
                 init_bb=None,
                 init_centroid=None,
                 init_timestamp=None,
                 label=None,
                 config = None
                 ):
        """

        Args:
            init_bb: array(7) or array(7*k), 3d box or tracklet
            init_timestamp: int, init timestamp
            label: int, unique ID for this trajectory
        """


        self.init_bb = init_bb
        if self.init_bb is None or self.init_bb.size == 0:
            self.init_bb = np.zeros((0,7))
        else:
            self.init_bb = self.init_bb.reshape((1,7))
        
        self.init_centroid = init_centroid
        if self.init_centroid is None or self.init_centroid.size == 0:
            self.init_centroid = np.zeros((0,4))
        else:
            self.init_centroid = self.init_centroid.reshape((1,4))

        self.init_timestamp = init_timestamp
        self.label = label
        self.tracking_bb_size = True

        self.config = config

        self.build_A = self.build_A_carlike

        self.scanning_interval = 1./self.config.LiDAR_scanning_frequency

        self.trajectory = {}

        self.track_dim = self.compute_track_dim() # 9+4+bb_features.shape

        self.init_parameters()
        self.init_trajectory()


        self.consecutive_missed_num = 0
        self.first_updated_timestamp = init_timestamp
        self.last_updated_timestamp = init_timestamp

    def __len__(self):
        return len(self.trajectory)

    def compute_track_dim(self):
        """
        compute tracking dimension
        :return:
        """
        track_dim = Trajectory.X_DIM #x,y,z,v_fwd,v_ang,yaw

        return track_dim

    def init_trajectory(self):
        """
        first initialize the object state with the input boxes info,
        then initialize the trajectory with the initialized object.
        :return:
        """

        detected_state_template = build_detected_state(self.init_bb, self.init_centroid)
        update_covariance_template = np.eye(Trajectory.X_DIM)*0.01
        update_covariance_template = np.mat(update_covariance_template).T

        self.build_H(
                self.init_bb is not None and np.size(self.init_bb),
                self.init_centroid is not None and np.size(self.init_centroid)
                )

        update_state_template = self.Hbck * detected_state_template
        
        object = Object()

        object.updated_state = update_state_template
        object.predicted_state = update_state_template
        object.detected_state = detected_state_template
        object.updated_covariance =update_covariance_template
        object.predicted_covariance = update_covariance_template

        self.trajectory[self.init_timestamp] = object

    def init_parameters(self):
        """
        initialize KF tracking parameters
        :return:
        """
        self.build_Q()
        self.build_P()
        self.build_A()

    def build_Q(self):    
        self.Q = np.mat(np.eye(Trajectory.X_DIM))
        self.Q *= self.config.state_func_covariance

    def build_P(self):
        self.P = np.mat(np.eye(Trajectory.Z_DIM))
        self.P[Trajectory.Z_ID_LIDAR_X:Trajectory.Z_ID_LIDAR_Z+1,Trajectory.Z_ID_LIDAR_X:Trajectory.Z_ID_LIDAR_Z+1] *= self.config.measure_func_covariance_lidar_xyz
        self.P[Trajectory.Z_ID_LIDAR_YAW,Trajectory.Z_ID_LIDAR_YAW] *= self.config.measure_func_covariance_lidar_yaw
        self.P[Trajectory.Z_ID_CAMERA_X:Trajectory.Z_ID_CAMERA_Z+1,Trajectory.Z_ID_CAMERA_X:Trajectory.Z_ID_CAMERA_Z+1] *= self.config.measure_func_covariance_camera_xyz
        self.P[Trajectory.Z_ID_CAMERA_FAKEYAW,Trajectory.Z_ID_CAMERA_FAKEYAW] *= self.config.measure_func_covariance_camera_yaw

    def build_A_carlike(self, state = None):
        # update transition matrix A wrt simple car model

        if state is None:
            v_fwd = 0
            theta = 0
        else:
            v_fwd = state[Trajectory.X_ID_V_FWD]
            theta = state[Trajectory.X_ID_YAW]

        self.A = np.mat(np.eye(Trajectory.X_DIM))

        self.A[Trajectory.X_ID_X,Trajectory.X_ID_V_FWD] = np.sin(np.pi/2-theta) * self.scanning_interval
        self.A[Trajectory.X_ID_Y,Trajectory.X_ID_V_FWD] = np.sin(theta)  * self.scanning_interval
        self.A[Trajectory.X_ID_YAW,Trajectory.X_ID_V_ANG] = 1 * self.scanning_interval
        self.A[Trajectory.X_ID_V_FWD,Trajectory.X_ID_V_FWD] = 0.99
        self.A[Trajectory.X_ID_V_ANG,Trajectory.X_ID_V_ANG] = 0.99

        self.K = np.mat(np.zeros(shape=(Trajectory.X_DIM,Trajectory.X_DIM)))
        self.K[Trajectory.X_ID_V_FWD, Trajectory.X_ID_X] = self.A[Trajectory.X_ID_X, Trajectory.X_ID_V_FWD]
        self.K[Trajectory.X_ID_V_FWD, Trajectory.X_ID_Y] = self.A[Trajectory.X_ID_Y, Trajectory.X_ID_V_FWD]
        self.K[Trajectory.X_ID_V_ANG, Trajectory.X_ID_YAW] = self.A[Trajectory.X_ID_YAW, Trajectory.X_ID_V_ANG]


    def build_H(self, has_bbox = False, has_centroid = False):
        # update observation jacobians
        self.build_H_fwd(has_bbox, has_centroid)
        self.build_H_bck(has_bbox, has_centroid)

        with np.printoptions(precision=2, suppress=True):
            print ("UPDATE C & H")
            print (self.H.shape,"C:\n",self.H)
            print (self.Hbck.shape,"H:\n",self.Hbck)

    def build_H_fwd(self, has_bbox, has_centroid):
        # update observation jacobian
        w_xyz_bbox = 1 if has_bbox else 0
        w_xyz_centroid = 1 if has_centroid else 0 #and not has_bbox else 0
        w_ang_bbox = 1 if has_bbox else 0
        w_ang_centroid = 1 if has_centroid else 0 #and not has_bbox else 0

        self.H = np.mat(np.zeros(shape=(Trajectory.Z_DIM,Trajectory.X_DIM)))
        self.H[Trajectory.Z_ID_LIDAR_X:Trajectory.Z_ID_LIDAR_Z+1, Trajectory.X_ID_X:Trajectory.X_ID_Z+1] = np.eye(3)*w_xyz_bbox
        self.H[Trajectory.Z_ID_LIDAR_YAW,Trajectory.X_ID_YAW] = 1*w_ang_bbox
        self.H[Trajectory.Z_ID_CAMERA_X:Trajectory.Z_ID_CAMERA_Z+1, :Trajectory.X_ID_Z+1] = np.eye(3)*w_xyz_centroid
        self.H[Trajectory.Z_ID_CAMERA_FAKEYAW,Trajectory.X_ID_YAW] = 1*w_ang_centroid
        

    def build_H_bck(self, has_bbox, has_centroid):
        # update inverted observation jacobian
        w_bbox = 1.0 * (has_bbox)
        w_centroid = 1.0 * (has_centroid)
        
        w_xyz_total = w_bbox + w_centroid
        w_xyz_bbox = w_bbox / w_xyz_total if w_xyz_total > 0 else 0
        w_xyz_centroid = w_centroid / w_xyz_total if w_xyz_total > 0 else 0


        w_ang_total = w_bbox + w_centroid 
        w_ang_bbox = w_bbox / w_ang_total if w_ang_total > 0 else 0
        w_ang_centroid = w_centroid / w_ang_total if w_ang_total > 0 else 0

        self.Hbck = np.mat(np.zeros(shape=(Trajectory.Z_DIM,Trajectory.X_DIM)))
        self.Hbck[Trajectory.Z_ID_LIDAR_X:Trajectory.Z_ID_LIDAR_Z+1, Trajectory.X_ID_X:Trajectory.X_ID_Z+1] = np.eye(3)*w_xyz_bbox
        self.Hbck[Trajectory.Z_ID_LIDAR_YAW,Trajectory.X_ID_YAW] = 1*w_ang_bbox
        self.Hbck[Trajectory.Z_ID_CAMERA_X:Trajectory.Z_ID_CAMERA_Z+1, :Trajectory.X_ID_Z+1] = np.eye(3)*w_xyz_centroid
        self.Hbck[Trajectory.Z_ID_CAMERA_FAKEYAW,Trajectory.X_ID_YAW] = 1*w_ang_centroid
        self.Hbck = self.Hbck.T

    def state_prediction(self,timestamp):
        """
        predict the object state at the given timestamp
        """

        previous_timestamp = timestamp-1

        assert previous_timestamp in self.trajectory.keys()

        previous_object = self.trajectory[previous_timestamp]

        if previous_object.updated_state is not None:
            previous_state = previous_object.updated_state
            previous_covariance = previous_object.updated_covariance
        else:
            previous_state = previous_object.predicted_state
            previous_covariance = previous_object.predicted_covariance


        self.build_A(previous_state)
        with np.printoptions(precision=2, suppress=True):
            print ("UPDATE A")
            print (self.A.shape,"A:\n",self.A)
            print (self.H.shape,"H:\n",self.H)
            print (self.Hbck.shape,"Hbck:\n",self.Hbck)
            print (self.P.shape,"P:\n",self.P)  
            print (self.K.shape,"K:\n",self.K)


        current_predicted_state = self.A*previous_state
        current_predicted_state[Trajectory.X_ID_YAW,0] = (current_predicted_state[Trajectory.X_ID_YAW,0]+np.pi) % (np.pi*2) - np.pi
        current_predicted_covariance = self.A*previous_covariance*self.A.T + self.Q

        new_ob = Object()

        new_ob.predicted_state = current_predicted_state
        new_ob.predicted_covariance = current_predicted_covariance

        self.trajectory[timestamp] = new_ob
        self.consecutive_missed_num += 1

    def sigmoid(self,x):
        return 1.0/(1+np.exp(-float(x)))

    def state_update(self,
                     bb=None,
                     centroid=None,
                     timestamp=None,
                     ):
        """
        update the trajectory
        Args:
            bb: array(7) or array(7*k), 3D box or tracklet
            timestamp:
        """
        assert timestamp in self.trajectory.keys()

        if bb is None:
            bb = np.zeros((0,4))
        if centroid is None:
            centroid = np.zeros((0,3))

        detected_state_template = build_detected_state(bb, centroid)
        self.build_H(np.size(bb), np.size(centroid))

        current_ob = self.trajectory[timestamp]

        predicted_state = current_ob.predicted_state
        predicted_covariance = current_ob.predicted_covariance

        temp = self.H*predicted_covariance*self.H.T+self.P

        KF_gain = predicted_covariance*self.H.T*temp.I

        predicted_detection = self.H*predicted_state
        prediction_error = detected_state_template-predicted_detection
        prediction_error[Trajectory.Z_ID_LIDAR_YAW] = ((prediction_error[Trajectory.Z_ID_LIDAR_YAW] + np.pi) % (2*np.pi)) - np.pi
        prediction_error[Trajectory.Z_ID_CAMERA_FAKEYAW] = ((prediction_error[Trajectory.Z_ID_CAMERA_FAKEYAW] + np.pi) % (2*np.pi)) - np.pi

        updated_state = predicted_state+KF_gain*prediction_error
        updated_covariance = (np.mat(np.eye(Trajectory.X_DIM)) - KF_gain*self.H)*predicted_covariance

        with np.printoptions(precision=2, suppress=True):
            #print ("KF_gain",KF_gain.shape,"\n",KF_gain)
            print ("detected_state ",detected_state_template.T)
            print ("predicted_state",predicted_state.T)
            print ("predicted_det  ",predicted_detection.T)
            print ("prediction_err ",prediction_error.T)
            print ("KF gain:\n",KF_gain)
            print ("predicted_cov\n",predicted_covariance)
            print ("updated state  ",updated_state.T)

        if len(self.trajectory)==2:

            updated_state = self.Hbck*detected_state_template+\
                            self.K*(self.Hbck*detected_state_template-self.trajectory[timestamp-1].updated_state)


        current_ob.updated_state = updated_state
        current_ob.updated_covariance = updated_covariance
        current_ob.detected_state = detected_state_template

        self.consecutive_missed_num = 0
        self.last_updated_timestamp = timestamp

