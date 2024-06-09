# Copyright (c) Meta Platforms, Inc. and affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import copy
import numpy as np
from tcdm.motion_util import Pose, PoseAndVelocity


class HandReferenceMotion(object):
    def __init__(self, motion_file, start_step=0):
        self._load_motion(motion_file)
        self._substeps = int(self._reference_motion['SIM_SUBSTEPS'])                
        self._data_substeps = self._reference_motion.get('DATA_SUBSTEPS', self._substeps)              
        self._step, self._start_step = 0, int(start_step)
    
    def _load_motion(self, motion_file):
        motion_file = np.load(motion_file, allow_pickle=True)
        self._reference_motion =  {k:v for k, v in motion_file.items()}
        self._reference_motion['s_0'] = self._reference_motion['s_0'][()]

    def reset(self):
        self._step = 0
        return copy.deepcopy(self._reference_motion['s_0'])

    def step(self):
        self._check_valid_step()
        self._step += self._data_substeps
    
    def revert(self):
        self._step -= self._data_substeps
        self._check_valid_step()

    def __len__(self):
        return self.length

    @property
    def t(self):
        return self._step
    
    @property
    def data_substep(self):
        return self._data_substeps

    @property
    def time(self):
        return float(self._step) / self.length

    @property
    def qpos(self):
        self._check_valid_step()
        return self._reference_motion['s'][self._step].copy()

    @property
    def qvel(self):
        self._check_valid_step()
        return self._reference_motion['sdot'][self._step].copy()

    @property
    def eef_pos(self):
        self._check_valid_step()
        return self._reference_motion['eef_pos'][self._step].copy()
    
    @property
    def human_joint_coords(self):
        self._check_valid_step()
        return self._reference_motion['human_joint_coords'][self._step].copy()

    @property
    def eef_quat(self):
        self._check_valid_step()
        return self._reference_motion['eef_quat'][self._step].copy()

    @property
    def eef_linear_velocity(self):
        self._check_valid_step()
        return self._reference_motion['eef_velp'][self._step].copy()

    @property
    def eef_angular_velocity(self):
        self._check_valid_step()
        return self._reference_motion['eef_velr'][self._step].copy()

    @property
    def body_poses(self):
        pos = self.eef_pos
        rot = self.eef_quat
        lv = self.eef_linear_velocity
        av = self.eef_angular_velocity
        return PoseAndVelocity(pos, rot, lv, av)

    @property
    def substeps(self):
        return self._substeps

    @property
    def done(self):
        assert self._step is not None, "Motion must be reset before it can be done"
        return self._step >= self.length
    
    @property
    def next_done(self):
        assert self._step is not None, "Motion must be reset before it can be done"
        return self._step >= self.length - self._data_substeps

    @property
    def n_left(self):
        assert self._step is not None, "Motion must be reset before lengths calculated"
        n_left = (self.length - self._step) / float(self._data_substeps) - 1
        return int(max(n_left, 0))
    
    @property
    def n_steps(self):
        n_steps = self.length / float(self._data_substeps) - 1
        return int(max(n_steps, 0))

    @property
    def length(self):
        if 'length' in self._reference_motion:
            return self._reference_motion['length']
        return self._reference_motion['s'].shape[0]
    
    @property
    def start_step(self):
        return self._start_step

    def _check_valid_step(self):
        assert not self.done, "Attempting access data and/or step 'done' motion"
        assert self._step >= self._start_step, "step must be at least start_step"
    
    def __getitem__(self, key):
        value =  copy.deepcopy(self._reference_motion[key])
        if not isinstance(value, np.ndarray):
            return value
        if len(value.shape) >= 2:
            return value[self._start_step::self._data_substeps]
        return value


class HandObjectReferenceMotion(HandReferenceMotion):
    def __init__(self, object_name, motion_file):
        super().__init__(motion_file)
        self._object_name = object_name

    @property
    def object_name(self):
        return self._object_name

    @property
    def object_pos(self):
        self._check_valid_step()
        return self._reference_motion['object_translation'][self._step].copy()
    
    @property
    def floor_z(self):
        return float(self._reference_motion['object_translation'][0,2])
    
    @property
    def object_rot(self):
        self._check_valid_step()
        return self._reference_motion['object_orientation'][self._step].copy()
    
    @property
    def object_pose(self):
        pos = self.object_pos[None]
        rot = self.object_rot[None]
        return Pose(pos, rot)

    @property
    def goals(self):
        g = []
        for i in [1, 5, 10]:
            i = min(self._step + i, self.length-1)
            for k in ('object_orientation', 'object_translation'):
                g.append(self._reference_motion[k][i].flatten())
        return np.concatenate(g)
    
    
    @property
    def target_desired_grasp(self):
        # the hand pose including hand position and hand orientation of desired grasp
        # hammer
        if self.object_name == 'hammer/object':
            desired_translation = [ 0.04094999,-0.05173023 , 0.02222911]
            desired_orientation = [-0.07151932,  0.70318082 , 0.70561092, -0.05034831]
        # knife
        if self.object_name == 'knife/object':
            desired_translation= [-0.07691809, -0.0772799 ,  0.01733831]
            desired_orientation=[0.47592615, 0.52394372, 0.48218629, 0.51621086]
        # flute
        if self.object_name == 'flute/object':
            desired_translation= [ 0.05818609, -0.07350232,  0.02632541]
            desired_orientation= [-0.02834436,  0.74105551,  0.67002352, -0.03319352]
        return desired_translation, desired_orientation
    
    @property
    def target_finger_joint_pose(self):
        # the finger joint pose of desired grasp
        # hammer
        if self.object_name == 'hammer/object':
            desired_finger_joint_pose = np.array([- 9.94715767e-03,  1.39037769e+00,
            1.25629876e+00,  1.28208797e+00,- 1.27525318e-01,  1.40187807e+00,
            1.23195438e+00,  1.25464143e+00,- 1.26315454e-01,  1.39991139e+00,
            1.21558336e+00,  1.23513631e+00, 5.85334148e-04 , 1.94472532e-02,
            1.40353025e+00,  1.19965137e+00, 1.21606934e+00 , 5.44385020e-01,
            1.16720185e+00,  1.99281304e-01, 1.98100386e-02 ,- 5.12473242e-01])
        # knife
        if self.object_name == 'knife/object':
            desired_finger_joint_pose=np.array([ 
            -4.87688668e-03,  1.48282989e+00,  1.40820512e+00,
            1.46063394e+00 , 3.59416011e-03 , 1.56232028e+00 , 1.36127476e+00,
            1.40261216e+00 ,-1.97810770e-02 , 1.50851118e+00 , 1.31762979e+00,
            1.35497545e+00 , 4.63481929e-03 ,-4.57848335e-02 , 1.56216984e+00,
            1.39497532e+00 , 1.44292081e+00 , 5.47230267e-01 , 1.18731647e+00,
            1.58987402e-01 ,-1.68825657e-01 ,-8.09119508e-01])
        # flute
        if self.object_name == 'flute/object':
            desired_finger_joint_pose=np.array( [1.67763564e-02,  1.50182377e+00,  1.41982324e+00,
            1.47285288e+00, 1.16004660e-02,  1.56627407e+00,  1.36865745e+00,
            1.40590332e+00, 1.10734967e-02,  1.56163298e+00,  1.38250675e+00,
            1.42902648e+00, 7.74659425e-03, -2.99860155e-03,  1.47157315e+00,
            1.44815110e+00, 1.50655932e+00,  6.08606663e-01,  1.19346000e+00,
            1.26515893e-01, 9.74385327e-02, -1.21704817e+00])
        return desired_finger_joint_pose
    





