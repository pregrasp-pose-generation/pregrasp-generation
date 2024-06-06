import yaml, os, imageio, cv2
from tcdm import suite
from stable_baselines3 import PPO
from argparse import ArgumentParser
import numpy as np
import json
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
parser = ArgumentParser(description="Example code for loading pre-trained policies")
parser.add_argument('--object', default='hammer', type=str)
parser.add_argument('--render', action="store_true", help="Supply flag to render mp4")
object_task_map={'hammer':'hammer_use1','knife':'knife_chop1','flute':'flute_pass1'}
object_pose_map={'hammer':[-1.94971568e-03,  1.95830716e-02 , 1.23636204e-02,-1.05889432e-03, -7.86119640e-04, -3.90730238e-01,9.20504287e-01],'knife':[-1.94971568e-03,  1.95830716e-02 , 0.006004,0,0,0,1],'flute':[-0.00728576, -0.00140855 ,0.00838198,0,0,0,1]}
hand_geom_list = [i for i in range(11, 72)]
table_geom_list=[6]

def hand_rot_world(WRJeuler,WORLD2OBJECT_QUAT):
    object2hand_rot=np.array(R.from_euler("xyz",WRJeuler).as_matrix())
    world2object_rot = R.from_quat(WORLD2OBJECT_QUAT).as_matrix()
    world2hand_rot=np.dot(world2object_rot,object2hand_rot)
    return world2hand_rot

def hand_trans_world(WRJtrans,OBJECT_CONTACT_CENTER,OBJECT_DMAX,OBJECT_CENTER,WORLD2OBJECT_TRANS,WORLD2OBJECT_QUAT):
    object2hand_trans=(np.array(WRJtrans)/0.115+np.array(OBJECT_CONTACT_CENTER))*OBJECT_DMAX+np.array(OBJECT_CENTER)
    world2object_rot = R.from_quat(WORLD2OBJECT_QUAT).as_matrix()
    world2hand_trans=np.dot(world2object_rot,object2hand_trans)+np.array(WORLD2OBJECT_TRANS)
    return world2hand_trans

def angle_between_vectors(vector1, vector2):
    dot_product = np.dot(vector1, vector2)
    norm_vector1 = np.linalg.norm(vector1)
    norm_vector2 = np.linalg.norm(vector2)
    cos_theta = dot_product / (norm_vector1 * norm_vector2)
    theta = np.arccos(cos_theta)
    return theta

def acquire_forearm_pose(hand_trans,hand_quat):
    world2forearm_quat=hand_quat
    world2forearm_trans=np.dot(R.from_quat(hand_quat).as_matrix(),np.array([0,0,-0.43]))+hand_trans
    wrist_angle_ud=0
    return world2forearm_trans, world2forearm_quat, wrist_angle_ud

def render(writer, physics, AA=2, height=640, width=640):
    if writer is None:
        return
    img = physics.render(camera_id=0, height=height * AA, width=width * AA)
    writer.append_data(cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA))
    
def rollout(save_folder, writer):
    config =  yaml.safe_load(open(os.path.join(save_folder, 'exp_config.yaml'), 'r'))
    o, t = config['env']['name'].split('-')
    env = suite.load(o, t, config['env']['task_kwargs'], gym_wrap=True)
    policy = PPO.load('model.zip')
    s, done, _ = env.reset(), False, 0
    data_list  = []
    render(writer, env.wrapped.physics)
    object_geom_list=[i for i in range(72,len(np.array(env.wrapped.physics.named.data.geom_xpos)))]
    while not done:
        action, _ = policy.predict(s['state'], deterministic=True)
        s, _, done, _ = env.step(action)
        hand_contact_object_flag=0
        object_contact_table_flag=0
        for j, c in enumerate(env.wrapped.physics.data.contact):
          if (c.geom2 in table_geom_list and c.geom1 in object_geom_list) or (c.geom1 in table_geom_list and c.geom2 in object_geom_list):
              object_contact_table_flag=1
          if ((c.geom1 in hand_geom_list and c.geom2 in object_geom_list) or (c.geom2 in hand_geom_list and c.geom1 in object_geom_list)):
              hand_contact_object_flag=1
        if ~object_contact_table_flag and hand_contact_object_flag:
            data_list.append(s['state'][30:36][2])
        render(writer, env.wrapped.physics)
    if len(data_list) > 10:
        return True
    return False


def check_quality(object_code,data_dict):
    pre_grasp_dict = data_dict['index']
    object_center=data_dict['object_center']
    object_dmax=data_dict['object_dmax']
    world2object_trans=object_pose_map[object_code][:3]
    world2object_quat=object_pose_map[object_code][3:]
    object_contact_center=data_dict['object_contact_center']
    pregrasp_pass_list = []
    for pre_grasp_index in pre_grasp_dict:
        os.makedirs(f'check_quality_results/{object_code}', exist_ok=True)
        writer = imageio.get_writer(f'check_quality_results/{object_code}/rollout{pre_grasp_index}.mp4', fps=25)
        # obtain pre-grasp pose
        data_qpos = list(pre_grasp_dict[pre_grasp_index].values())
        normobj2palm_euler=data_qpos[-6:-3]
        normobj2palm_trans=data_qpos[-3:]
        world2palm_trans=hand_trans_world(normobj2palm_trans,object_contact_center,object_dmax,object_center,world2object_trans,world2object_quat)
        world2palm_rot=hand_rot_world(normobj2palm_euler,world2object_quat)
        world2palm_quat=R.from_matrix(world2palm_rot).as_quat()
        world2forearm_trans, world2forearm_quat, wrist_angle_ud=acquire_forearm_pose(world2palm_trans,world2palm_quat)
        # correct the trajectory npz file 
        data=np.load('trajectories/'+object_task_map[object_code]+'_origin.npz' ,allow_pickle=True)
        s_0=data['s_0']
        object_translation=data['object_translation']
        object_orientation=data['object_orientation']
        length=data['length']
        SIM_SUBSTEPS=data['SIM_SUBSTEPS']
        DATA_SUBSTEPS=data['DATA_SUBSTEPS']
        s_0=dict(s_0.tolist())
        position_list = s_0['initialized']['position']
        s_0['initialized']['position']=np.concatenate((np.array([0,0,0,0,0,0]),np.array([0, wrist_angle_ud]), data_qpos[:22],position_list[-6:]))
        np.savez('trajectories/'+object_task_map[object_code]+'.npz' ,s_0=s_0,object_translation=object_translation,object_orientation=object_orientation,length=length,SIM_SUBSTEPS=SIM_SUBSTEPS,DATA_SUBSTEPS=DATA_SUBSTEPS)
        tree = ET.parse('tcdm/envs/assets/robots/adroit/base_origin.xml')
        root = tree.getroot()
        element = root.find('.//body[@name="forearm"]')
        element.set('pos', str(world2forearm_trans[0]) + ' ' + str(world2forearm_trans[1]) + ' ' + str(world2forearm_trans[2]))
        element.set('quat',str(world2forearm_quat[3])+' '+str(world2forearm_quat[0])+' '+str(world2forearm_quat[1])+' '+str(world2forearm_quat[2]))
        tree.write('tcdm/envs/assets/robots/adroit/base.xml')
        # simulation screening
        if rollout(f'configs/{object_task_map[object_code]}/', writer):
            print(pre_grasp_index," is great")
            pregrasp_pass_list.append(pre_grasp_index)
        else:
            print(pre_grasp_index," is bad")
        writer.close()
    return pregrasp_pass_list


if __name__ == "__main__":
    args = parser.parse_args()
    with open('pregrasp_json/'+args.object+'_correct.json', 'r') as f:
        data_dict = json.load(f)
    pregrasp_pass_list = check_quality(args.object,data_dict)
    print("filtered pre-grasp pose list:",pregrasp_pass_list)