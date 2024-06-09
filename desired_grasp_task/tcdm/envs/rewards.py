# Copyright (c) Meta Platforms, Inc. and affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import abc
import copy
import numpy as np
from tcdm.motion_util import rotation_distance
from scipy.spatial.transform import Rotation as R
import torch
import math


def quat_dot(quat1, quat2):
    return quat1[0] * quat2[0] + quat1[1] * quat2[1] + quat1[2] * quat2[2] + quat1[3] * quat2[3]

def quat_norm(quat):
    return math.sqrt(quat[0] ** 2 + quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2)

def angle_between_quaternions(q1, q2):
    q1_norm = quat_norm(q1)
    q2_norm = quat_norm(q2)
    q1_normalized = np.array(q1) / q1_norm
    q2_normalized = np.array(q2) / q2_norm
    
    dot_product = quat_dot(q1_normalized, q2_normalized)
    dot_product = max(min(dot_product, 1.0), -1.0)
    
    angle = math.acos(dot_product)
    return angle


def get_reward(name):
    if name == 'objectmimic':
        return ObjectMimic
    if name == 'dummy':
        return Dummy
    raise ValueError("Reward {} not supported!".format(name)) 


def norm2(x):
    return np.sum(np.square(x))


class _ErrorTracker:
    def __init__(self, targets, thresh=0.01, start=0):
        self._targets = targets.copy()
        self._values = np.zeros_like(self._targets)
        self._thresh, self._i = thresh, 0
        self._start = start
    
    def append(self, v):
        if self._i >= len(self._values):
            return
        self._values[self._i:] = v[None]
        self._i += 1
    
    @property
    def N(self):
        return len(self._targets) - self._start

    @property
    def error(self):
        v, t = self._values[self._start:], self._targets[self._start:]
        return np.linalg.norm(v - t) / self.N

    @property
    def success(self):
        v, t = self._values[self._start:], self._targets[self._start:]
        deltas = np.sqrt(np.sum(np.square(v - t), axis=-1))
        if len(deltas.shape) > 1:
            deltas = np.mean(deltas.reshape((self.N, -1)), axis=1)
        return np.mean(deltas <= self._thresh)


class RewardFunction(abc.ABC):
    def __init__(self, **override_hparams):
        """
        Overrides default hparams with values passed into initializer
        """
        self.episode_end=False
        self.episode_step_count=0
        self.episode_success_sum=0
        self.relocation_success_count=0
        self.episode_total_count=0
        self.success_rate=0.0
        self.episode_end=False
        self.episode_step_count=0

        self.last_delta_hp= None
        self.last_delta_hr= None
        self.last_delta_hj= None
        params = copy.deepcopy(self.DEFAULT_HPARAMS)
        for k, v in override_hparams.items():
            assert k in params, "Param {} does not exist in struct".format(k)
            params[k] = v
        
        for k, v in params.items():
            setattr(self, k, v)
    
    @abc.abstractproperty
    def DEFAULT_HPARAMS(self):
        """
        Returns default hyperparamters for reward function
        """
    
    def initialize_rewards(self, parent_task, physics):
        """
        Gets parent task and sets constants as required from it
        """
        self._parent_task = parent_task

    @abc.abstractmethod
    def get_reward(self, physics):
        """
        Calculates reward and success stats from phyiscs data
        Returns reward, info_dict
        """

    @abc.abstractmethod
    def check_termination(self, physics):
        """
        Checks if trajectory should terminate
        Returns terminate_flag
        """
    
    def __call__(self, physics):
        return self.get_reward(physics)


class Dummy(RewardFunction):
    @property
    def DEFAULT_HPARAMS(self):
        return dict()
    
    def get_reward(self, _):
        return 0.0, dict()
    
    def check_termination(self, _):
        return False


class ObjectMimic(RewardFunction):
    @property
    def DEFAULT_HPARAMS(self):
        return  {
                    'obj_err_scale': 50,
                    'object_reward_scale': 10,
                    'lift_bonus_thresh': 0.02,
                    'lift_bonus_mag': 2.5,
                    'obj_com_term': 0.25,
                    'n_envs': 1,
                    'obj_reward_ramp': 0,
                    'obj_reward_start': 0
                }
    
    def initialize_rewards(self, parent_task, physics):
        self._step_count = parent_task.step_count
        self._reference_motion = parent_task.reference_motion
        self._object_name = self._reference_motion.object_name
        floor_z = physics.named.data.xipos[self._object_name][2]
        self._lift_z = floor_z + self.lift_bonus_thresh

        # register metric tracking data
        self._obj = _ErrorTracker(self._reference_motion['object_translation'][1:])
        
    def desired_grasp_reward(self,physics):
        # obtain the current hand pose in the world frame
        world2palm_translation=physics.named.data.xpos['adroit/palm']
        world2palm_orientation=np.array(physics.named.data.xmat['adroit/palm']).reshape((3,3))
        # obtain the current object pose in the world frame
        world2object_translation=physics.named.data.xpos[self._object_name]
        world2object_orientation=np.array(physics.named.data.xmat[self._object_name]).reshape((3,3))
        # obtain the current hand pose in the object frame
        object2hand_translation=np.dot(np.linalg.inv(world2object_orientation),world2palm_translation-world2object_translation)
        object2hand_orientation=np.dot(np.linalg.inv(world2object_orientation), world2palm_orientation) 
        object2hand_quat = R.from_matrix(object2hand_orientation).as_quat()
        
        desired_translation, desired_orientation = self._reference_motion.target_desired_grasp
        desired_finger_joint_pose = self._reference_motion.target_finger_joint_pose

        # R_hp
        delta_hp = np.sqrt(norm2(desired_translation - object2hand_translation))
        if self.last_delta_hp is None :
            self.last_delta_hp = delta_hp
        delta_hp_max = 0.0333
        
        r_hp = (self.last_delta_hp - delta_hp) / delta_hp_max
        self.last_delta_hp = delta_hp

        # R_hr
        delta_hr = angle_between_quaternions(desired_orientation, object2hand_quat)
        if self.last_delta_hr is None:
            self.last_delta_hr = delta_hr
        delta_hr_max = 0.1045
        r_hr = (self.last_delta_hr - delta_hr) / delta_hr_max
        self.last_delta_hr = delta_hr

        # R_hj
        qpos = physics.named.data.qpos
        finger_joint_pose = np.array([qpos[i] for i in range(8, 30)])
        
        delta_hj_max = 0.1045
        delta_hj = sum(abs(finger_joint_pose - desired_finger_joint_pose)) / len(finger_joint_pose)

        if self.last_delta_hj is None:
            self.last_delta_hj = delta_hj
        r_hj = (self.last_delta_hj - delta_hj) / delta_hj_max
        self.last_delta_hj = delta_hj

        h_prox_r = 1
        h_prox_p = 10 
        w_r = (1 - min(h_prox_p, delta_hp) / h_prox_p)*(1 - min(h_prox_r, delta_hr) / h_prox_r)
    
        return r_hp + r_hr + w_r * r_hj


    def own_success(self,physics,thresh_pos=0.05,thresh_rot=0.2,thresh_joint=0.2):
        # obtain the current hand pose in the world frame
        world2palm_translation=physics.named.data.xpos['adroit/palm']
        world2palm_orientation=np.array(physics.named.data.xmat['adroit/palm']).reshape((3,3))
        # obtain the current object pose in the world frame
        world2object_translation=physics.named.data.xpos[self._object_name]
        world2object_orientation=np.array(physics.named.data.xmat[self._object_name]).reshape((3,3))
        # obtain the current hand pose in the object frame
        object2hand_translation=np.dot(np.linalg.inv(world2object_orientation),world2palm_translation-world2object_translation)
        object2hand_orientation=np.dot(np.linalg.inv(world2object_orientation), world2palm_orientation) 
        object2hand_quat = R.from_matrix(object2hand_orientation).as_quat()
        # obtain the current finger joint pose
        qpos = physics.named.data.qpos
        finger_joint_pose = np.array([qpos[i] for i in range(8, 30)])
        
        desired_translation, desired_orientation = self._reference_motion.target_desired_grasp
        desired_finger_joint_pose = self._reference_motion.target_finger_joint_pose

        delta_hp = np.sqrt(norm2(desired_translation - object2hand_translation))
        delta_hr = angle_between_quaternions(desired_orientation, object2hand_quat)
        delta_hj = sum(abs(finger_joint_pose - desired_finger_joint_pose)) / len(finger_joint_pose)
        
        if delta_hp <= thresh_pos and delta_hr <= thresh_rot and delta_hj <= thresh_joint:
            self.relocation_success_count+=1
        if self.episode_end or self.episode_step_count ==19:
            if self.relocation_success_count>=3:
                self.episode_success_sum+=1
            self.relocation_success_count=0
            self.episode_end=False
            self.episode_step_count=0
            self.episode_total_count+=1
            if self.episode_total_count%10==0:
                self.success_rate  = float(self.episode_success_sum)/float(self.episode_total_count)    
                self.episode_total_count=0
                self.episode_success_sum=0
        
        return self.success_rate
    
    def get_reward(self, physics):
        # get targets from reference object
        tgt_obj_com = self._reference_motion.object_pos
        tgt_obj_rot = self._reference_motion.object_rot
        # get real values from physics object
        obj_com = physics.named.data.xipos[self._object_name].copy()
        obj_rot = physics.named.data.xquat[self._object_name].copy()
        self._obj.append(obj_com)

        # calculate both object "matching" reward and lift bonus
        obj_com_err = np.sqrt(norm2(tgt_obj_com - obj_com))
        obj_rot_err = rotation_distance(obj_rot, tgt_obj_rot) / np.pi
        obj_reward = np.exp(-self.obj_err_scale * (obj_com_err + 0.1 * obj_rot_err))
        lift_bonus = (tgt_obj_com[2] >= self._lift_z) and (obj_com[2] >= self._lift_z)

        obj_scale = self._object_reward_scale()
        # relocation reward
        reward = obj_scale * obj_reward + self.lift_bonus_mag * float(lift_bonus)
        # desired grasp reward
        r_grasp_reward=self.desired_grasp_reward(physics)
        weight_grasp=(1-min(tgt_obj_com[2],abs(tgt_obj_com[2]-obj_com[2]))/tgt_obj_com[2])
        success = torch.sigmoid(torch.tensor([r_grasp_reward]))
        self.episode_step_count+=1
        # populate info dict
        info = {
            'time_frac': self._reference_motion.time,
            'obj_err': self._obj.error,
            'obj_success': self.own_success(physics),
            'step_obj_err': obj_com_err,
            'r_grasp':r_grasp_reward,
            'r_follow':reward
        }
        info['obj_err_scale'] = obj_scale / self.object_reward_scale \
                                if self.object_reward_scale else 0
        # the total reward
        return r_grasp_reward*weight_grasp*5 + reward*3, info



    def check_termination(self, physics):
        # terminate if object delta greater than threshold
        tgt_obj_com = self._reference_motion.object_pos
        obj_com = physics.named.data.xipos[self._object_name].copy()
        if norm2(obj_com - tgt_obj_com) >= self.obj_com_term ** 2:
            self.episode_end=True
        return norm2(obj_com - tgt_obj_com) >= self.obj_com_term ** 2

    def _object_reward_scale(self):
        if self.obj_reward_ramp > 0:
            delta = self._step_count * self.n_envs - self.obj_reward_start
            delta /= float(self.obj_reward_ramp)
        else:
            delta = 1.0 if self._step_count >= self.obj_reward_start \
                    else 0.0
        return self.object_reward_scale * min(max(delta, 0), 1)
