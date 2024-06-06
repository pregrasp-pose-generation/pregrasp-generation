"""
Last modified date: 2023.02.23
Author: Jialiang Zhang
Description: energy functions
"""
import torch
import numpy as np

def penalty_term(normalized_vector, target_direction_list, batch_size_each):
    """
    Calculate E_rotation

    Parameters
    ----------
    normalized_vector: (M*B, 3) torch.Tensor
        current hand direction vector
    target_direction_list: (M, 3) torch.Tensor
        target direction vector
    batch_size_each: batch_size
    

    Return
    ----------
    penalty_list: (1,M*B) torch.Tensor
        E_rotation
    
    """
    penalty_list=[]
    for i in range(len(target_direction_list)):
        normalized_vector_i = normalized_vector[i*batch_size_each:(i+1)*batch_size_each]
        target_direction = target_direction_list[i]
        cosine_similarity = torch.matmul(normalized_vector_i, target_direction)
        penalty = 1 - cosine_similarity
        penalty = penalty.view(-1, 1)
        penalty_list.append(penalty.squeeze())
    penalty_list=torch.stack(penalty_list)
    penalty_list = penalty_list.reshape(1,-1).squeeze()
    return penalty_list


def cal_energy(hand_model, object_model, contact_point_set_list, desired_hand_direction_list,  w_fc=1.0, w_dis=100.0, w_pen=400.0, w_spen=12.0, w_joints=1.0, w_tpen=40.0, w_function=200.0, w_rotation=1.0, verbose=False):
    batch_size, n_contact, _ = hand_model.contact_points.shape
    
    # E_dis
    device = object_model.device
    distance, contact_normal = object_model.cal_distance(hand_model.contact_points)
    E_dis = torch.sum(distance.abs(), dim=-1, dtype=torch.float).to(device)

    # E_fc
    contact_normal = contact_normal.reshape(batch_size, 1, 3 * n_contact)
    transformation_matrix = torch.tensor([[0, 0, 0, 0, 0, -1, 0, 1, 0],
                                          [0, 0, 1, 0, 0, 0, -1, 0, 0],
                                          [0, -1, 0, 1, 0, 0, 0, 0, 0]],
                                         dtype=torch.float, device=device)
    g = torch.cat([torch.eye(3, dtype=torch.float, device=device).expand(batch_size, n_contact, 3, 3).reshape(batch_size, 3 * n_contact, 3),
                   (hand_model.contact_points @ transformation_matrix).view(batch_size, 3 * n_contact, 3)], 
                  dim=2).float().to(device)
    norm = torch.norm(contact_normal @ g, dim=[1, 2])
    E_fc = norm * norm
    
    # E_joints
    E_joints = torch.sum((hand_model.hand_pose[:, 9:] > hand_model.joints_upper) * (hand_model.hand_pose[:, 9:] - hand_model.joints_upper), dim=-1) + \
        torch.sum((hand_model.hand_pose[:, 9:] < hand_model.joints_lower) * (hand_model.joints_lower - hand_model.hand_pose[:, 9:]), dim=-1)

    # E_pen
    object_scale = object_model.object_scale_tensor.flatten().unsqueeze(1).unsqueeze(2)
    object_surface_points = object_model.surface_points_tensor * object_scale
    distances = hand_model.cal_distance(object_surface_points)
    distances[distances <= 0] = 0
    E_pen = distances.sum(-1)

    # E_spen
    E_spen = hand_model.self_penetration()

    # E_tpen
    plane_distances = hand_model.cal_dis_plane(object_model.plane_parameters)  # [B, n_links]
    plane_distances[plane_distances > 0] = 0
    E_tpen = -plane_distances.sum(-1)

    # E_function
    special_point_list = [(torch.Tensor(contact_point_set_list[i]) * 0.1).to(device) for i in range(len(contact_point_set_list))]
    E_function = object_model.cal_distance_to_specific_point(hand_model.get_surface_points_verts(), special_point_list)

    # E_rotation
    normalized_vector_list = []
    for i in range(len(desired_hand_direction_list)):
        hand_begin=torch.Tensor(desired_hand_direction_list[i][0]).to(device)
        hand_end=torch.Tensor(desired_hand_direction_list[i][1]).to(device)
        target_vector = hand_end - hand_begin
        norms = torch.norm(target_vector, p=2, dim=0)
        norms = norms.view(-1, 1)
        normalized_vector = target_vector / norms
        normalized_vector_list.append(normalized_vector.T)
    E_rotation = penalty_term(hand_model.get_hand_pose_direction(), normalized_vector_list,object_model.batch_size_each)
    z_direction = abs(100 * hand_model.get_hand_pose_direction()[:, 2])# Reduce the angle between the hand and the table
    if verbose:
        return w_fc * E_fc + w_dis * E_dis + w_pen * E_pen + w_spen * E_spen + w_joints * E_joints + w_tpen * E_tpen + w_function * E_function + w_rotation * E_rotation + z_direction, E_fc, E_dis, E_pen, E_spen, E_joints, E_tpen, E_function, E_rotation
    else:
        return w_fc * E_fc + w_dis * E_dis + w_pen * E_pen + w_spen * E_spen + w_joints * E_joints + w_tpen * E_tpen + w_function * E_function + w_rotation * E_rotation + z_direction
    
