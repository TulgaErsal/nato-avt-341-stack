import numpy as np

def bbs_to_corners(bbs):
    # <- x,y,z,l,w,h,yaw

    x = bbs[:,0]
    y = bbs[:,1]
    z = bbs[:,2]
    l = bbs[:,3]
    w = bbs[:,4]
    h = bbs[:,5]
    yaw = bbs[:,6]

    yaw = yaw

    fwd_x = l * np.cos(np.pi*2-yaw)
    fwd_y = -1 * l * np.sin(np.pi*2-yaw)

    side_x = -1 * w * np.sin(yaw)
    side_y =  w * np.cos(yaw)

    t = z + h/2
    b = z - h/2

    fl_x = x + fwd_x/2 + side_x/2
    fl_y = y + fwd_y/2 + side_y/2
        
    fr_x = x + fwd_x/2 - side_x/2
    fr_y = y + fwd_y/2 - side_y/2
        
    nl_x = x - fwd_x/2 + side_x/2
    nl_y = y - fwd_y/2 + side_y/2
        
    nr_x = x - fwd_x/2 - side_x/2
    nr_y = y - fwd_y/2 - side_y/2

    fbl = np.stack([fl_x,fl_y,b],axis=1)
    ftl = np.stack([fl_x,fl_y,t],axis=1)

    fbr = np.stack([fr_x,fr_y,b],axis=1)
    ftr = np.stack([fr_x,fr_y,t],axis=1)

    nbl = np.stack([nl_x,nl_y,b],axis=1)
    ntl = np.stack([nl_x,nl_y,t],axis=1)

    nbr = np.stack([nr_x,nr_y,b],axis=1)
    ntr = np.stack([nr_x,nr_y,t],axis=1)

    corners = np.stack([fbl,fbr,nbr,nbl,ftl,ftr,ntr,ntl],axis=1)

    return corners


def corners_to_bbs(corners):
    # -> x,y,z,l,w,h,yaw

    center = np.mean(corners, axis=1)

    fbl = corners[:,0,:] # farbottomleft
    fbr = corners[:,1,:]
    nbr = corners[:,2,:]
    nbl = corners[:,3,:]
    ftl = corners[:,4,:] 
    ftr = corners[:,5,:]
    ntr = corners[:,6,:]
    ntl = corners[:,7,:]

    l = np.sqrt(np.power(fbl-nbl,2).sum(axis=1))
    w = np.sqrt(np.power(fbl-fbr,2).sum(axis=1))
    h = np.sqrt(np.power(fbl-ftl,2).sum(axis=1))

    #yaw = 2*np.pi - np.arctan2(fbl[:,1]-nbl[:,1],fbl[:,0]-nbl[:,0])
    yaw = np.arctan2(fbl[:,1]-nbl[:,1],fbl[:,0]-nbl[:,0])

    yaw = (yaw + np.pi) % (2*np.pi) - np.pi

    bbs = np.concatenate([center,np.expand_dims(l,1),np.expand_dims(w,1),np.expand_dims(h,1),np.expand_dims(yaw,1)],axis=1) 
    return bbs.reshape((-1,7))

def convert_bbs_type(boxes, input_box_type):
    boxes = np.array(boxes)

    assert input_box_type in ["Kitti", "OpenPCDet", "Waymo"], 'unsupported input box type!'

    if input_box_type in ["OpenPCDet", "Waymo"]:
        return boxes

    if input_box_type == "Kitti":  # (h,w,l,x,y,z,yaw) -> (x,y,z,l,w,h,yaw)

        t_id = boxes.shape[1] // 7
        new_boxes = np.zeros(shape=boxes.shape)
        new_boxes[:, :] = boxes[:, :]
        for i in range(t_id):
            b_id = i * 7
            new_boxes[:, b_id + 0:b_id + 3] = boxes[:, b_id + 3:b_id + 6]
            new_boxes[:, b_id + 3] = boxes[:, b_id + 2]
            new_boxes[:, b_id + 4] = boxes[:, b_id + 1]
            new_boxes[:, b_id + 5] = boxes[:, b_id + 0]
            new_boxes[:, b_id + 6] = (np.pi - boxes[:, b_id + 6]) + np.pi / 2
            new_boxes[:, b_id + 2] += boxes[:, b_id + 0] / 2
        return new_boxes

def get_registration_angle(mat):
    cos_theta = mat[0, 0]
    sin_theta = mat[1, 0]

    if cos_theta < -1:
        cos_theta = -1
    if cos_theta > 1:
        cos_theta = 1

    theta_cos = np.arccos(cos_theta)

    if sin_theta >= 0:
        return theta_cos
    else:
        return 2 * np.pi - theta_cos


def register_bbs(boxes, pose):

    if pose is None:
        return boxes

    ang = get_registration_angle(pose)

    t_id = boxes.shape[1] // 7

    ones = np.ones(shape=(boxes.shape[0], 1))
    for i in range(t_id):
        b_id = i * 7
        box_xyz = boxes[:, b_id:b_id + 3]
        box_xyz1 = np.concatenate([box_xyz, ones], -1)

        box_world = np.matmul(box_xyz1, pose.T)

        boxes[:, b_id:b_id + 3] = box_world[:, 0:3]
        boxes[:, b_id + 6] += ang
    return boxes

def corners3d_to_img_boxes(P2, corners3d):
    """
    :param corners3d: (N, 8, 3) corners in rect coordinate
    :return: boxes: (None, 4) [x1, y1, x2, y2] in rgb coordinate
    :return: boxes_corner: (None, 8) [xi, yi] in rgb coordinate
    """
    sample_num = corners3d.shape[0]
    corners3d_hom = np.concatenate((corners3d, np.ones((sample_num, 8, 1))), axis=2)  # (N, 8, 4)

    img_pts = np.matmul(corners3d_hom, P2.T)  # (N, 8, 3)

    x, y = img_pts[:, :, 0] / img_pts[:, :, 2], img_pts[:, :, 1] / img_pts[:, :, 2]
    x1, y1 = np.min(x, axis=1), np.min(y, axis=1)
    x2, y2 = np.max(x, axis=1), np.max(y, axis=1)

    img_boxes = np.concatenate((x1.reshape(-1, 1), y1.reshape(-1, 1), x2.reshape(-1, 1), y2.reshape(-1, 1)), axis=1)
    boxes_corner = np.concatenate((x.reshape(-1, 8, 1), y.reshape(-1, 8, 1)), axis=2)

    img_boxes[:, 0] = np.clip(img_boxes[:, 0], 0, 1242 - 1)
    img_boxes[:, 1] = np.clip(img_boxes[:, 1], 0, 375 - 1)
    img_boxes[:, 2] = np.clip(img_boxes[:, 2], 0, 1242 - 1)
    img_boxes[:, 3] = np.clip(img_boxes[:, 3], 0, 375 - 1)

    return img_boxes, boxes_corner

def bb3d_2_bb2d(bb3d,P2):

    x,y,z,l,w,h,yaw = bb3d[0],bb3d[1],bb3d[2],bb3d[3],bb3d[4],bb3d[5],bb3d[6]

    pt1 = [l / 2, 0, w / 2, 1]
    pt2 = [l / 2, 0, - w / 2, 1]
    pt3 = [- l / 2, 0, w / 2, 1]
    pt4 = [- l / 2, 0, - w / 2, 1]
    pt5 = [l / 2, - h, w / 2, 1]
    pt6 = [l / 2, - h, - w / 2, 1]
    pt7 = [- l / 2, - h, w / 2, 1]
    pt8 = [- l / 2, - h, - w / 2, 1]
    pts = np.array([[pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8]])
    transpose = np.array([[np.cos(np.pi - yaw), 0, -np.sin(np.pi - yaw), x],
                          [0, 1, 0, y],
                          [np.sin(np.pi - yaw), 0, np.cos(np.pi - yaw), z],
                          [0, 0, 0, 1]])
    pts = np.matmul(pts, transpose.T)
    box, _ = corners3d_to_img_boxes(P2, pts[:, :, 0:3])

    return box
