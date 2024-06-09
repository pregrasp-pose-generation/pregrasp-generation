from util import *

parser = argparse.ArgumentParser()
parser.add_argument('--object', default='hammer', type=str)
args = parser.parse_args()

tree = ET.parse('assets/environments/total_environment.xml')
root = tree.getroot()
includes = root.findall('.//include')
includes[1].set('file', '../objects/'+args.object+'.xml')
tree.write('assets/environments/total_environment.xml')

PALM_BODY_ID=4
OBJECT_BODY_ID=1

# load environment
m = mujoco.MjModel.from_xml_path('assets/environments/total_environment.xml')
d = mujoco.MjData(m)
desired_grasp=[]
last_desired_grasp=[]
joint_pose=[]
last_joint_pose=[]
with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < 1000:
    # dict to record the desired grasp information of one timestep
    step_start = time.time()
    mujoco.mj_step(m, d)

    world2object_trans = d.xpos[OBJECT_BODY_ID]
    world2object_rot = np.array(d.xmat[OBJECT_BODY_ID]).reshape((3, 3))
    world2hand_trans=d.xpos[PALM_BODY_ID]
    world2hand_rot=np.array(d.xmat[PALM_BODY_ID]).reshape((3, 3))
    # information1:desired grasp(hand position+hand orientation) in object's coordinate
    object2hand_trans=np.dot(np.linalg.inv(world2object_rot),np.array(world2hand_trans)-np.array(world2object_trans))
    object2hand_quat=R.from_matrix(np.dot(np.linalg.inv(world2object_rot),world2hand_rot)).as_quat()
    last_desired_grasp=desired_grasp
    desired_grasp=object2hand_trans.tolist()+object2hand_quat.tolist()
    # information2:finger joint pose
    name_list=['robot0:FFJ3', 'robot0:FFJ2', 'robot0:FFJ1', 'robot0:FFJ0', 'robot0:MFJ3', 'robot0:MFJ2', 'robot0:MFJ1', 'robot0:MFJ0', 'robot0:RFJ3', 'robot0:RFJ2', 'robot0:RFJ1', 'robot0:RFJ0', 'robot0:LFJ4', 'robot0:LFJ3', 'robot0:LFJ2', 'robot0:LFJ1', 'robot0:LFJ0', 'robot0:THJ4', 'robot0:THJ3', 'robot0:THJ2', 'robot0:THJ1', 'robot0:THJ0']
    robot_joint_angles=d.qpos[14:]
    qpos_list=list(robot_joint_angles)
    qpos_dict={}
    cnt=0
    for i in range(len(qpos_list)):
      qpos_dict[name_list[cnt]]=qpos_list[cnt]
      cnt=cnt+1
    last_joint_pose=joint_pose
    joint_pose=qpos_dict

    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1
      # automatically close hand within the first second
      if time.time() - start < 1:
        action_list = [0,0,0,0,0,0,0,0,0,1.57,1.57,0.5,0,1.57,1.57,0.5,0,1.57,1.57,0.5,0,0,1.57,1.57,0.5,-1.05,1.22,-0.209,0.524,0]
        ctrl_set_action(m, d, np.array(action_list))

    viewer.sync()

    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step*10)
print('desired_grasp:',last_desired_grasp)
print('joint_pose:',last_joint_pose)







