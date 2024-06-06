"""
Last modified date: 2022.03.11
Author: mzhmxzh
Description: Entry of the program
"""

import os

# os.chdir(os.path.dirname(__file__))
import json
import argparse
import shutil
import numpy as np
import torch
from tqdm import tqdm
import math
import transforms3d

import trimesh as tm
import pytorch3d.structures
import pytorch3d.ops
from torchsdf import index_vertices_by_faces


from utils.hand_model import HandModel
from utils.object_model import ObjectModel
from utils.initializations import initialize_table_top
from utils.energy import cal_energy
from utils.optimizer import Annealing
from utils.logger import Logger
from utils.rot6d import robust_compute_rotation_matrix_from_ortho6d


# prepare arguments
print(torch.cuda.is_available())
parser = argparse.ArgumentParser()
# experiment settings
parser.add_argument('--seed', default=1, type=int)
parser.add_argument('--gpu', default="1", type=str)
parser.add_argument('--object_code_list', nargs='+', default=
    ['hammer'])
parser.add_argument('--name', default='demo', type=str)
parser.add_argument('--n_contact', default=4, type=int)
parser.add_argument('--batch_size', default=128, type=int)
parser.add_argument('--n_iter', default=5000, type=int)
parser.add_argument('--poses', default='../data/poses', type=str)
# hyper parameters
parser.add_argument('--switch_possibility', default=0.5, type=float)
parser.add_argument('--mu', default=0.98, type=float)
parser.add_argument('--eps', default=1e-6, type=float)
parser.add_argument('--noise_size', default=0.005, type=float)
parser.add_argument('--stepsize_period', default=50, type=int)
parser.add_argument('--starting_temperature', default=18, type=float)
parser.add_argument('--annealing_period', default=30, type=int)
parser.add_argument('--temperature_decay', default=0.95, type=float)
parser.add_argument('--w_fc', default=1.0, type=float)
parser.add_argument('--w_dis', default=100.0, type=float)
parser.add_argument('--w_pen', default=200.0, type=float)
parser.add_argument('--w_spen', default=12.0, type=float)
parser.add_argument('--w_joints', default=2.0, type=float)
parser.add_argument('--w_tpen', default=40.0, type=float)
parser.add_argument('--w_function', default=200.0, type=float)
parser.add_argument('--w_rotation', default=1.0, type=float)

# initialization settings
parser.add_argument('--jitter_strength', default=0.1, type=float)
parser.add_argument('--distance_lower', default=0.2, type=float)
parser.add_argument('--distance_upper', default=0.3, type=float)
parser.add_argument('--theta_lower', default=-math.pi / 6, type=float)
parser.add_argument('--theta_upper', default=math.pi / 6, type=float)
parser.add_argument('--angle_upper', default=math.pi / 4, type=float)
# energy thresholds
parser.add_argument('--thres_fc', default=0.3, type=float)
parser.add_argument('--thres_dis', default=0.005, type=float)
parser.add_argument('--thres_pen', default=0.001, type=float)

args = parser.parse_args()

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

np.seterr(all='raise')
np.random.seed(args.seed)
torch.manual_seed(args.seed)


# prepare models
total_batch_size = len(args.object_code_list) * args.batch_size
os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

hand_model = HandModel(
    mjcf_path='mjcf/shadow_hand_wrist_free.xml',
    mesh_path='mjcf/meshes',
    contact_points_path='mjcf/contact_points.json',
    penetration_points_path='mjcf/penetration_points.json',
    device=device
)

object_model = ObjectModel(
    data_root_path='../data/meshdata',
    batch_size_each=args.batch_size,
    num_samples=2000, 
    device=device
)

# acquire desired grasp data from the json file
contact_point_set_list=[]
desired_hand_direction_list=[]
for i in range(len(args.object_code_list)):
    with open('object_json/'+args.object_code_list[i] +'.json', 'r') as file:
        desired_data = json.load(file)
    k = list(desired_data.keys())
    value_k = ['dmax','object center','palm direction vector(in the normalized object coordinate system)', 'contact_set(in the normalized object coordinate system)']
    contact_point_set = desired_data[k[-2]][value_k[-1]]
    desired_hand_direction = desired_data[k[-2]][value_k[-2]]
    contact_point_set_list.append(contact_point_set)
    desired_hand_direction_list.append(desired_hand_direction)

# initialize params of models
object_model.initialize(args.object_code_list, contact_point_set_list)

# scale up the object
n_objects = len(object_model.object_mesh_list)
object_model.object_face_verts_list = []
object_model.surface_points_tensor = []
for i in range(n_objects):
    object_model.object_mesh_list[i] = object_model.object_mesh_list[i].copy().apply_scale(1.5)# enlargement factor is 1.5
    object_model.object_scale_tensor_old.append(object_model.object_scale_tensor[i].clone())
    object_model.object_scale_tensor[i] = object_model.object_scale_tensor[i] * 1.5
    object_verts = torch.Tensor(object_model.object_mesh_list[i].vertices).to(object_model.device) 
    object_faces = torch.Tensor(object_model.object_mesh_list[i].faces).long().to(object_model.device)
    object_model.object_face_verts_list.append(index_vertices_by_faces(object_verts, object_faces))
    object_model.contact_point_set_list = [object_model.contact_point_set_list[i]*1.5 for i in range(len(object_model.contact_point_set_list))]
    if object_model.num_samples != 0:
        vertices = torch.tensor(object_model.object_mesh_list[i].vertices, dtype=torch.float,
                                device=object_model.device) 
        faces = torch.tensor(object_model.object_mesh_list[i].faces, dtype=torch.float, device=object_model.device)
        mesh = pytorch3d.structures.Meshes(vertices.unsqueeze(0), faces.unsqueeze(0))
        dense_point_cloud = pytorch3d.ops.sample_points_from_meshes(mesh,
                                                                    num_samples=100 * object_model.num_samples)
        surface_points = pytorch3d.ops.sample_farthest_points(dense_point_cloud, K=object_model.num_samples)[0][0]
        surface_points.to(dtype=float, device=object_model.device)
        object_model.surface_points_tensor.append(surface_points)
if object_model.num_samples != 0:
    object_model.surface_points_tensor = torch.stack(object_model.surface_points_tensor, dim=0).repeat_interleave(
        object_model.batch_size_each, dim=0)
        
initialize_table_top(hand_model, object_model, args)

hand_pose_st = hand_model.hand_pose.detach()

optim_config = {
    'switch_possibility': args.switch_possibility,
    'starting_temperature': args.starting_temperature,
    'temperature_decay': args.temperature_decay,
    'annealing_period': args.annealing_period,
    'noise_size': args.noise_size,
    'stepsize_period': args.stepsize_period,
    'mu': args.mu,
    'device': device
}
optimizer = Annealing(hand_model, **optim_config)

try:
    shutil.rmtree(os.path.join('../data/experiments', args.name, 'logs'))
except FileNotFoundError:
    pass
os.makedirs(os.path.join('../data/experiments', args.name, 'logs'), exist_ok=True)
logger_config = {
    'thres_fc': args.thres_fc,
    'thres_dis': args.thres_dis,
    'thres_pen': args.thres_pen
}
logger = Logger(log_dir=os.path.join('../data/experiments', args.name, 'logs'), **logger_config)


# optimize
weight_dict = dict(
    w_fc=args.w_fc,
    w_dis=args.w_dis,
    w_pen=args.w_pen,
    w_spen=args.w_spen,
    w_joints=args.w_joints,
    w_tpen=args.w_tpen,
    w_function=args.w_function,
    w_rotation=args.w_rotation
)
energy, E_fc, E_dis, E_pen, E_spen, E_joints, E_tpen, E_function, E_rotation = cal_energy(hand_model, object_model, contact_point_set_list=object_model.contact_point_set_list, desired_hand_direction_list=desired_hand_direction_list, verbose=True, **weight_dict)

energy.sum().backward(retain_graph=True)
logger.log(energy, E_fc, E_dis, E_pen, E_spen, E_joints, E_tpen, E_function, E_rotation, 0, show=False)

for step in tqdm(range(1, args.n_iter + 1), desc='optimizing'):
    s = optimizer.try_step()

    optimizer.zero_grad()
    new_energy, new_E_fc, new_E_dis, new_E_pen, new_E_spen, new_E_joints, new_E_tpen, new_E_function, new_E_rotation = cal_energy(hand_model, object_model, contact_point_set_list=object_model.contact_point_set_list, desired_hand_direction_list=desired_hand_direction_list, verbose=True, **weight_dict)

    new_energy.sum().backward(retain_graph=True)

    with torch.no_grad():
        accept, t = optimizer.accept_step(energy, new_energy)

        energy[accept] = new_energy[accept]
        E_dis[accept] = new_E_dis[accept]
        E_fc[accept] = new_E_fc[accept]
        E_pen[accept] = new_E_pen[accept]
        E_spen[accept] = new_E_spen[accept]
        E_joints[accept] = new_E_joints[accept]
        E_tpen[accept] = new_E_tpen[accept]
        E_function[accept] = new_E_function[accept]
        E_rotation[accept] = new_E_rotation[accept]

        logger.log(energy, E_fc, E_dis, E_pen, E_spen, E_joints, E_tpen, E_function, E_rotation, step, show=False)


# save results
translation_names = ['WRJTx', 'WRJTy', 'WRJTz']
rot_names = ['WRJRx', 'WRJRy', 'WRJRz']
joint_names = [
    'robot0:FFJ3', 'robot0:FFJ2', 'robot0:FFJ1', 'robot0:FFJ0',
    'robot0:MFJ3', 'robot0:MFJ2', 'robot0:MFJ1', 'robot0:MFJ0',
    'robot0:RFJ3', 'robot0:RFJ2', 'robot0:RFJ1', 'robot0:RFJ0',
    'robot0:LFJ4', 'robot0:LFJ3', 'robot0:LFJ2', 'robot0:LFJ1', 'robot0:LFJ0',
    'robot0:THJ4', 'robot0:THJ3', 'robot0:THJ2', 'robot0:THJ1', 'robot0:THJ0'
]
try:
    shutil.rmtree(os.path.join('../data/experiments', args.name, 'results'))
except FileNotFoundError:
    pass
os.makedirs(os.path.join('../data/experiments', args.name, 'results'), exist_ok=True)
result_path = os.path.join('../data/experiments', args.name, 'results')
os.makedirs(result_path, exist_ok=True)
for i in range(len(args.object_code_list)):
    data_list = []
    for j in range(args.batch_size):
        idx = i * args.batch_size + j
        scale = object_model.object_scale_tensor_old[i][j].item()
        hand_pose = hand_model.hand_pose[idx].detach().cpu()
        qpos = dict(zip(joint_names, hand_pose[9:].tolist()))
        rot = robust_compute_rotation_matrix_from_ortho6d(hand_pose[3:9].unsqueeze(0))[0]
        euler = transforms3d.euler.mat2euler(rot, axes='sxyz')
        qpos.update(dict(zip(rot_names, euler)))
        qpos.update(dict(zip(translation_names, hand_pose[:3].tolist())))
        hand_pose = hand_pose_st[idx].detach().cpu()
        qpos_st = dict(zip(joint_names, hand_pose[9:].tolist()))
        rot = robust_compute_rotation_matrix_from_ortho6d(hand_pose[3:9].unsqueeze(0))[0]
        euler = transforms3d.euler.mat2euler(rot, axes='sxyz')
        qpos_st.update(dict(zip(rot_names, euler)))
        qpos_st.update(dict(zip(translation_names, hand_pose[:3].tolist())))
        data_list.append(dict(
            scale=scale,
            plane=object_model.plane_parameters[idx].tolist(), 
            qpos=qpos,
            qpos_st=qpos_st,
            energy=energy[idx].item(),
            E_fc=E_fc[idx].item(),
            E_dis=E_dis[idx].item(),
            E_pen=E_pen[idx].item(),
            E_spen=E_spen[idx].item(),
            E_joints=E_joints[idx].item(),
            E_tpen=E_tpen[idx].item(),
            E_function=E_function[idx].item(), 
            E_rotation=E_rotation[idx].item()
        ))
    np.save(os.path.join(result_path, args.object_code_list[i] + '.npy'), data_list, allow_pickle=True)
