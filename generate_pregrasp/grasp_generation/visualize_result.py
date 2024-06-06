"""
Last modified date: 2023.02.23
Author: Jialiang Zhang
Description: visualize grasp result in world frame using plotly.graph_objects
"""

import os
import sys
import json

import argparse
import torch
import numpy as np
import transforms3d
import plotly.graph_objects as go

from utils.hand_model import HandModel
from utils.object_model import ObjectModel

translation_names = ['WRJTx', 'WRJTy', 'WRJTz']
rot_names = ['WRJRx', 'WRJRy', 'WRJRz']
joint_names = [
    'robot0:FFJ3', 'robot0:FFJ2', 'robot0:FFJ1', 'robot0:FFJ0',
    'robot0:MFJ3', 'robot0:MFJ2', 'robot0:MFJ1', 'robot0:MFJ0',
    'robot0:RFJ3', 'robot0:RFJ2', 'robot0:RFJ1', 'robot0:RFJ0',
    'robot0:LFJ4', 'robot0:LFJ3', 'robot0:LFJ2', 'robot0:LFJ1', 'robot0:LFJ0',
    'robot0:THJ4', 'robot0:THJ3', 'robot0:THJ2', 'robot0:THJ1', 'robot0:THJ0'
]

def plane2pose(plane_parameters):
    r3 = plane_parameters[:3]
    r2 = torch.zeros_like(r3)
    r2[0], r2[1], r2[2] = (-r3[1], r3[0], 0) if r3[2] * r3[2] <= 0.5 else (-r3[2], 0, r3[0])
    r1 = torch.cross(r2, r3)
    pose = torch.zeros([4, 4], dtype=torch.float, device=plane_parameters.device)
    pose[0, :3] = r1
    pose[1, :3] = r2
    pose[2, :3] = r3
    pose[2, 3] = plane_parameters[3]
    pose[3, 3] = 1
    return pose

def numerical_screening(result_path, object_code, batch_size,object_center,dmax,object_contact_center):
    """
    Numerical screening

    Parameters
    ----------
    result_path: str
        result npy file path
    object_code: str
        object name
    batch_size: int
        batch_size of the npy file
    """
    device = 'cpu'
    correct_group = {}
    correct_group_path = 'numerical_screening/'+object_code + '_correct.json'

    for i in range(batch_size):
        ns_data_dict = np.load(os.path.join(result_path, object_code + '.npy'), allow_pickle=True)[i]
        m_plane = torch.tensor(data_dict['plane'], dtype=torch.float, device=device)  # plane parameters in object reference frame: (A, B, C, D), Ax + By + Cz + D >= 0, A^2 + B^2 + C^2 = 1
        m_pose = plane2pose(m_plane)  # 4x4 homogeneous transformation matrix from object frame to world frame
        m_qpos = ns_data_dict['qpos']
        m_rot = np.array(transforms3d.euler.euler2mat(*[m_qpos[name] for name in rot_names]))
        m_rot = m_rot[:, :2].T.ravel().tolist()
        m_hand_pose = torch.tensor([m_qpos[name] for name in translation_names] + m_rot + [m_qpos[name] for name in joint_names], dtype=torch.float, device=device)
        m_hand_model = HandModel(
            mjcf_path='mjcf/shadow_hand_vis.xml',
            mesh_path='mjcf/meshes',
        )
        m_hand_model.set_parameters(m_hand_pose.unsqueeze(0))
        _, m_flag = m_hand_model.get_plotly_data(i=0, opacity=0.5, color='lightblue', with_contact_points=False, pose=m_pose)
        # Set thresholds for each energy function term for numerical screening
        if    (ns_data_dict['E_joints'] < 0.04) & (ns_data_dict['E_pen'] < 0.01)& (ns_data_dict['E_rotation'] < 0.25)  & (ns_data_dict['E_function'] < 0.1) & m_flag:#  
            print(i)
            correct_group[i] = m_qpos
    correct_group_data = {'index': correct_group}
    correct_group_data['object_dmax'] = dmax
    correct_group_data['object_center'] = object_center
    correct_group_data['object_contact_center'] = object_contact_center.tolist()
    with open(correct_group_path, 'w') as file:
        json.dump(correct_group_data, file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--object_code', type=str, default='hammer')  
    parser.add_argument('--num', type=int, default=1)
    parser.add_argument('--result_path', type=str, default='../data/experiments/demo/results')
    parser.add_argument('--batch_size', type=int, default=128)

    args = parser.parse_args()

    device = 'cpu'

    with open('object_json/'+args.object_code + '.json', 'r') as file:
        desired_data = json.load(file)
    k = list(desired_data.keys())
    value_k = ['dmax','object center','palm direction vector(in the normalized object coordinate system)', 'contact_set(in the normalized object coordinate system)']
    contact_point_set_list = [desired_data[k[-2]][value_k[-1]]]
    desired_hand_direction_list = [desired_data[k[-2]][value_k[-2]]]
    object_center = desired_data[k[-2]][value_k[-3]]
    dmax = desired_data[k[-2]][value_k[-4]]

    # load results
    data_dict = np.load(os.path.join(args.result_path, args.object_code + '.npy'), allow_pickle=True)[args.num]
    
    # hand model
    hand_model = HandModel(
        mjcf_path='mjcf/shadow_hand_vis.xml',
        mesh_path='mjcf/meshes',
    )

    # object model
    object_model = ObjectModel(
        data_root_path='../data/meshdata',
        batch_size_each=128,
        num_samples=2000, 
        device=device
    )

    object_model.initialize(args.object_code, contact_point_set_list)
    object_model.object_scale_tensor = torch.tensor(data_dict['scale'], dtype=torch.float, device=device).reshape(1, 1)

    group_path = 'numerical_screening/'+args.object_code + '_correct.json'
    file_exists = os.path.exists(group_path)
    
    if file_exists:
        pass
    else:
        numerical_screening(args.result_path, args.object_code, args.batch_size,object_center,dmax,object_model.center_point_list[0])
    with open(group_path, 'r') as file:
        read_data = json.load(file)
    print("correct group index1: " + str(read_data['index'].keys()))
    plane = torch.tensor(data_dict['plane'], dtype=torch.float, device=device)  # plane parameters in object reference frame: (A, B, C, D), Ax + By + Cz + D >= 0, A^2 + B^2 + C^2 = 1
    pose = plane2pose(plane)  # 4x4 homogeneous transformation matrix from object frame to world frame
    qpos = data_dict['qpos']
    print(qpos)
    rot = np.array(transforms3d.euler.euler2mat(*[qpos[name] for name in rot_names]))
    rot = rot[:, :2].T.ravel().tolist()
    hand_pose = torch.tensor([qpos[name] for name in translation_names] + rot + [qpos[name] for name in joint_names], dtype=torch.float, device=device)
    if 'qpos_st' in data_dict:
        qpos_st = data_dict['qpos_st']
        rot = np.array(transforms3d.euler.euler2mat(*[qpos_st[name] for name in rot_names]))
        rot = rot[:, :2].T.ravel().tolist()
        hand_pose_st = torch.tensor([qpos_st[name] for name in translation_names] + rot + [qpos_st[name] for name in joint_names], dtype=torch.float, device=device)

   
    
    # visualize
    if 'qpos_st' in data_dict:
        hand_model.set_parameters(hand_pose_st.unsqueeze(0))
        hand_st_plotly, _ = hand_model.get_plotly_data(i=0, opacity=0.5, color='lightblue', with_contact_points=False, pose=pose)
    else:
        hand_st_plotly = []
    hand_model.set_parameters(hand_pose.unsqueeze(0))
    hand_en_plotly, _ = hand_model.get_plotly_data(i=0, opacity=1, color='lightblue', with_contact_points=False, pose=pose)
    object_plotly = object_model.get_plotly_data(i=0, color='#82bda2', opacity=1, pose=pose, enlargement_visible=True, contact_points_visible=True)

    data = hand_st_plotly + hand_en_plotly + object_plotly

    fig = go.Figure(data)

    if 'energy' in data_dict:
        energy = data_dict['energy']
        E_fc = round(data_dict['E_fc'], 3)
        E_dis = round(data_dict['E_dis'], 5)
        E_pen = round(data_dict['E_pen'], 5)
        E_spen = round(data_dict['E_spen'], 5)
        E_joints = round(data_dict['E_joints'], 5)
        E_tpen = round(data_dict['E_tpen'], 4)
        E_function = round(data_dict['E_function'], 5)
        E_rotation = round(data_dict['E_rotation'], 4)
        result = f'Index {args.num}  E_fc {E_fc}  E_dis {E_dis}  E_pen {E_pen}  E_spen {E_spen} E_joints {E_joints} E_tpen {E_tpen} E_function {E_function} E_rotation {E_rotation}'
        fig.add_annotation(text=result, x=0.5, y=0.1, xref='paper', yref='paper')
    
    print("E_fc: " + str(E_fc))
    print("E_dis: " + str(E_dis))
    print("E_pen: " + str(E_pen))
    print("E_spen: " + str(E_spen))
    print("E_joints: " + str(E_joints))
    print("E_tpen: " + str(E_pen))
    print("E_function: " + str(E_function))
    print("E_rot: " + str(E_rotation))
    print("energy: " + str(energy))
    fig.update_layout(xaxis_visible=False, yaxis_visible=False)

    fig.show()
