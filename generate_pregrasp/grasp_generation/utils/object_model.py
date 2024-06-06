"""
Last modified date: 2023.02.23
Author: Ruicheng Wang, Jialiang Zhang
Description: Class ObjectModel
"""

import os
import trimesh as tm
import plotly.graph_objects as go
import torch
import pytorch3d.structures
import pytorch3d.ops
import numpy as np

from torchsdf import index_vertices_by_faces, compute_sdf
class ObjectModel:

    # def __init__(self, data_root_path, batch_size_each, contact_point_set,num_samples=2000, device="cuda"):
    def __init__(self, data_root_path, batch_size_each, num_samples=2000, device="cuda"):

        """
        Create a Object Model
        
        Parameters
        ----------
        data_root_path: str
            directory to object meshes
        batch_size_each: int
            batch size for each objects
        num_samples: int
            numbers of object surface points, sampled with fps
        device: str | torch.Device
            device for torch tensors
        """

        self.device = device
        self.batch_size_each = batch_size_each
        self.data_root_path = data_root_path
        self.num_samples = num_samples



        self.object_code_list = None
        self.object_scale_tensor = None
        self.object_mesh_list = None
        self.object_verts_list = None
        self.object_faces_list = None
        self.object_face_verts_list = None
        self.scale_choice = torch.tensor([0.1, 0.1, 0.1, 0.1, 0.1], dtype=torch.float, device=self.device)



    def initialize(self, object_code_list, contact_point_set_list):
        """
        Initialize Object Model with list of objects
        
        Choose scales, load meshes, sample surface points
        
        Parameters
        ----------
        object_code_list: the name of the object you choose
        contact_point_set_list: contact points between the hand and the object 
        """
        if not isinstance(object_code_list, list):
            object_code_list = [object_code_list]
        self.object_code_list = object_code_list
        self.object_scale_tensor = []
        self.object_scale_tensor_old = []
        self.object_mesh_list = []
        self.object_verts_list = []
        self.object_faces_list = []
        self.object_face_verts_list = []
        self.surface_points_tensor = []
        self.object_scale_list = []
        self.object_face_sample_points_list = []
        self.contact_point_set_list = [torch.Tensor(contact_point_set_list[i]).to(device=self.device) for i in range(len(contact_point_set_list))]
        # get the enlargement center
        self.center_point_list = [self.contact_point_set_list[i].mean(dim=0) for i in range(len(self.contact_point_set_list))]
        self.contact_point_set_list = [self.contact_point_set_list[i] - self.center_point_list[i] for i in range(len(self.center_point_list))]
        cnt=-1
        for object_code in object_code_list:
            cnt+=1
            self.object_scale_tensor.append(self.scale_choice[torch.randint(0, self.scale_choice.shape[0], (self.batch_size_each, ), device=self.device)])
            self.object_mesh_list.append(tm.load(os.path.join(self.data_root_path, object_code, "coacd", "decomposed.obj"), force="mesh", process=False))

            vert = torch.Tensor(self.object_mesh_list[-1].vertices).to(self.device) - self.center_point_list[cnt].to(self.device)
            face = torch.Tensor(self.object_mesh_list[-1].faces).long().to(self.device)
            self.object_mesh_list[-1] = tm.Trimesh(vertices=vert.cpu(), faces=face.cpu())

            self.object_verts_list.append(torch.Tensor(self.object_mesh_list[-1].vertices).to(self.device))
            self.object_faces_list.append(torch.Tensor(self.object_mesh_list[-1].faces).long().to(self.device))
            
            self.object_face_verts_list.append(index_vertices_by_faces(self.object_verts_list[-1], self.object_faces_list[-1]))
            self.object_face_sample_points_list.append(index_vertices_by_faces(self.object_verts_list[-1], self.object_faces_list[-1]).view(-1, 3))
            if self.num_samples != 0:
                vertices = torch.tensor(self.object_mesh_list[-1].vertices, dtype=torch.float, device=self.device)
                faces = torch.tensor(self.object_mesh_list[-1].faces, dtype=torch.float, device=self.device)
                mesh = pytorch3d.structures.Meshes(vertices.unsqueeze(0), faces.unsqueeze(0))
                dense_point_cloud = pytorch3d.ops.sample_points_from_meshes(mesh, num_samples=100 * self.num_samples)
                surface_points = pytorch3d.ops.sample_farthest_points(dense_point_cloud, K=self.num_samples)[0][0]
                surface_points.to(dtype=float, device=self.device)
                self.surface_points_tensor.append(surface_points)
        self.object_scale_tensor = torch.stack(self.object_scale_tensor, dim=0)
        if self.num_samples != 0:
            self.surface_points_tensor = torch.stack(self.surface_points_tensor, dim=0).repeat_interleave(self.batch_size_each, dim=0)  





    def cal_distance(self, x, with_closest_points=False):
        """
        Calculate signed distances from hand contact points to object meshes and return contact normals
        
        Interiors are positive, exteriors are negative
        
        Use our modified Kaolin package
        
        Parameters
        ----------
        x: (B, `n_contact`, 3) torch.Tensor
            hand contact points
        with_closest_points: bool
            whether to return closest points on object meshes
        
        Returns
        -------
        distance: (B, `n_contact`) torch.Tensor
            signed distances from hand contact points to object meshes, inside is positive
        normals: (B, `n_contact`, 3) torch.Tensor
            contact normal vectors defined by gradient
        closest_points: (B, `n_contact`, 3) torch.Tensor
            contact points on object meshes, returned only when `with_closest_points is True`
        """
        _, n_points, _ = x.shape
        x = x.reshape(-1, self.batch_size_each * n_points, 3)
        distance = []
        normals = []
        closest_points = []
        scale = self.object_scale_tensor.repeat_interleave(n_points, dim=1)
        x = x / scale.unsqueeze(2)
        for i in range(len(self.object_mesh_list)):
            face_verts = self.object_face_verts_list[i]
            dis, dis_signs, normal, _ = compute_sdf(x[i], face_verts)
            if with_closest_points:
                closest_points.append(x[i] - dis.sqrt().unsqueeze(1) * normal)
            dis = torch.sqrt(dis + 1e-8)
            dis = dis * (-dis_signs)
            distance.append(dis)
            normals.append(normal * dis_signs.unsqueeze(1))
        distance = torch.stack(distance)
        normals = torch.stack(normals)
        distance = distance * scale
        distance = distance.reshape(-1, n_points)
        normals = normals.reshape(-1, n_points, 3)
        if with_closest_points:
            closest_points = (torch.stack(closest_points) * scale.unsqueeze(2)).reshape(-1, n_points, 3)
            return distance, normals, closest_points
        return distance, normals

    



    def cal_distance_to_specific_point(self, x, target_points_list):
        """
        Calculate the shortest distance from a specific point on the hand to each contact point

        Parameters
        ----------
        x: (B, n_contact, 3) torch.Tensor
            hand contact points
        target_points_list: (M, N, 3) torch.Tensor
            contact points of M objects
        
        Returns
        -------
        avg_distance_list: (1,B*M) torch.Tensor
            list of M objects' average distance from each specific point on the hand to each contact point
        """
        avg_distance_list=[]
        for i in range(len(target_points_list)):
            x_point=x[self.batch_size_each*i:self.batch_size_each*(i+1)]
            target_points = target_points_list[i]
            B, n_contact, _ = x_point.shape
            N = target_points.size(0)
            target_points = target_points.unsqueeze(0).unsqueeze(0).expand(B, n_contact, N, 3)
            x_point = x_point.unsqueeze(2).expand_as(target_points)

            # Calculate the distance from a specific point on the hand to each contact point
            distance = torch.sqrt(torch.sum((x_point - target_points) ** 2, dim=3))

            # find the distance from a specific point on the hand to the nearest contact point
            min_distance = distance.min(dim=2)[0]

            avg_distance = min_distance.mean(dim=1)
            avg_distance_list.append(avg_distance.squeeze())
        avg_distance_list=torch.stack(avg_distance_list)
        avg_distance_list = avg_distance_list.reshape(1,-1).squeeze()
        return avg_distance_list


    

    def get_plotly_data(self, i, color='#82bda2', opacity=1, pose=None, enlargement_visible=False, contact_points_visible=False):
        """
        Get visualization data for plotly.graph_objects
        
        Parameters
        ----------
        i: int
            index of data
        color: str
            color of mesh
        opacity: float
            opacity
        pose: (4, 4) matrix
            homogeneous transformation matrix
        enlargement_visible: bool
            whether the enlarged object is visible
        contact_points_visible: bool
            whether contact points are visible
        
        Returns
        -------
        data: list
            list of plotly.graph_object visualization data
        """
        model_index = i // self.batch_size_each
        model_scale = self.object_scale_tensor[model_index, i % self.batch_size_each].detach().cpu().numpy()
        mesh = self.object_mesh_list[model_index]
        vertices = mesh.vertices * model_scale
        vertices_enlargement = mesh.vertices * model_scale * 1.5
        if pose is not None:
            pose = np.array(pose, dtype=np.float32)
            vertices = vertices @ pose[:3, :3].T + pose[:3, 3]
            vertices_enlargement = vertices_enlargement @ pose[:3, :3].T + pose[:3, 3]
            contact_points = self.contact_point_set_list[0] * 0.1@ pose[:3, :3].T + pose[:3, 3]
        data = [go.Mesh3d(x=vertices[:, 0],y=vertices[:, 1], z=vertices[:, 2], i=mesh.faces[:, 0], j=mesh.faces[:, 1], k=mesh.faces[:, 2], color=color, opacity=opacity)]
        print("the enlargement center: ",self.center_point_list[0])
        if enlargement_visible:
            data+= [go.Mesh3d(x=vertices_enlargement[:, 0],y=vertices_enlargement[:, 1], z=vertices_enlargement[:, 2], i=mesh.faces[:, 0], j=mesh.faces[:, 1], k=mesh.faces[:, 2], color=color, opacity=0.2)]
        if contact_points_visible:
            data+= [go.Scatter3d(x=contact_points[:, 0], y=contact_points[:, 1], z=contact_points[:, 2], mode='markers')]
        return data

