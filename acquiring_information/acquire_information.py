from util import *
# desired grasp and finger joint pose reference
# hammer
HAMMER_HAND_POSE_IN_OBJBODY = [0.045727008221729516, -0.04945806741672108, 0.025896688075467147, -0.051674922652902376, 0.7052531364605145, 0.7070371432201024, 0.0067966159653046095]
HAMMER_HAND_JOINT_POSE = {"robot0:FFJ3": 0.023384790613801904, "robot0:FFJ2": 1.5579785931207446, "robot0:FFJ1": 1.0249559100895693, "robot0:FFJ0": 0.9851826405981005, "robot0:MFJ3": -0.004694632617074976, "robot0:MFJ2": 1.561981077356856, "robot0:MFJ1": 1.0572207716015372, "robot0:MFJ0": 1.0194365810677901, "robot0:RFJ3": -0.0005977565775568668, "robot0:RFJ2": 1.5614267307388816, "robot0:RFJ1": 1.024767902385633, "robot0:RFJ0": 0.984809577334278, "robot0:LFJ4": 0.0035486506203113322, "robot0:LFJ3": -0.03951222578578184, "robot0:LFJ2": 1.4179014838034403, "robot0:LFJ1": 1.0240948266295027, "robot0:LFJ0": 0.9840094095314703, "robot0:THJ4": 0.31851007276925464, "robot0:THJ3": 1.2318342276214829, "robot0:THJ2": 0.2528727376726022, "robot0:THJ1": -0.18771641925932084, "robot0:THJ0": -0.9833350541295902}
# knife
KNIFE_HAND_POSE_IN_OBJBODY=[-0.0816818785905482, -0.0651469424596285, 0.032516443365419714, 0.4710136394396935, 0.567625497364354, 0.4026084516440363, 0.5420829095889631]
KNIFE_HAND_JOINT_POSE={"robot0:FFJ3": 0.008519496087041221, "robot0:FFJ2": 1.5620341844081196, "robot0:FFJ1": 1.3943463334464423, "robot0:FFJ0": 1.4485179910904082, "robot0:MFJ3": 0.06644744103133868, "robot0:MFJ2": 1.5628853275447134, "robot0:MFJ1": 1.3212726311082497, "robot0:MFJ0": 1.3631636336846005, "robot0:RFJ3": 0.0030237816513004594, "robot0:RFJ2": 1.3853651001700638, "robot0:RFJ1": 1.2920948257147327, "robot0:RFJ0": 1.358282139469869, "robot0:LFJ4": -0.003722990080782885, "robot0:LFJ3": -0.2276912747601621, "robot0:LFJ2": 1.329299554523081, "robot0:LFJ1": 1.3350624687041819, "robot0:LFJ0": 1.3738892813991916, "robot0:THJ4": 0.616460869282127, "robot0:THJ3": 1.2785171060544673, "robot0:THJ2": 0.2523749231215278, "robot0:THJ1": 0.005626397850239274, "robot0:THJ0": -0.8241850180496485}
# flute
FLUTE_HAND_POSE_IN_OBJBODY =  [0.019351117809207827, 0.027605681207801425, 0.07375156749695928, 0.00556225901797531, 0.9981813557142319, -0.05728415437265004, -0.017932318225074986]
FLUTE_HAND_JOINT_POSE = {"robot0:FFJ3": -0.03311306577486593, "robot0:FFJ2": 1.5066085174213837, "robot0:FFJ1": 1.4056458151561129, "robot0:FFJ0": 1.4850945899589545, "robot0:MFJ3": -0.03276523797570374, "robot0:MFJ2": 1.5614790826194036, "robot0:MFJ1": 1.3818671139222458, "robot0:MFJ0": 1.4348563889717774, "robot0:RFJ3": -0.04004815467231828, "robot0:RFJ2": 1.5659738418628313, "robot0:RFJ1": 1.4131592510640887, "robot0:RFJ0": 1.4683912340458904, "robot0:LFJ4": 0.008059625672822274, "robot0:LFJ3": -0.008511280649534958, "robot0:LFJ2": 1.3671560954886404, "robot0:LFJ1": 1.4721275796839965, "robot0:LFJ0": 1.5424936928729847, "robot0:THJ4": 0.6453097990729212, "robot0:THJ3": 1.3029790461696538, "robot0:THJ2": 0.25464929471385445, "robot0:THJ1": -0.05515193561023643, "robot0:THJ0": -0.8431454077093187}

parser = argparse.ArgumentParser()
parser.add_argument('--object', default='hammer', type=str)
INFORMATION_DICT={'hammer':{'OBJECT_GEOM_ID':6,'HAND_BEGIN':13,'HAND_END_ADD1':70,'FLOOR_GEOM_ID':0,'SKY_GEOM_ID':1,'TABLE_GEOM_ID':70,'HAND_POSE_IN_OBJBODY':HAMMER_HAND_POSE_IN_OBJBODY,'HAND_JOINT_POSE':HAMMER_HAND_JOINT_POSE},'knife':{'OBJECT_GEOM_ID':6,'HAND_BEGIN':13,'HAND_END_ADD1':70,'FLOOR_GEOM_ID':0,'SKY_GEOM_ID':1,'TABLE_GEOM_ID':70,'HAND_POSE_IN_OBJBODY':KNIFE_HAND_POSE_IN_OBJBODY,'HAND_JOINT_POSE':KNIFE_HAND_JOINT_POSE},'flute':{'OBJECT_GEOM_ID':6,'HAND_BEGIN':12,'HAND_END_ADD1':69,'FLOOR_GEOM_ID':0,'SKY_GEOM_ID':1,'TABLE_GEOM_ID':69,'HAND_POSE_IN_OBJBODY':FLUTE_HAND_POSE_IN_OBJBODY,'HAND_JOINT_POSE':FLUTE_HAND_JOINT_POSE}}
args = parser.parse_args()
tree = ET.parse('assets/environments/total_environment.xml')
root = tree.getroot()
includes = root.findall('.//include')
includes[1].set('file', '../objects/'+args.object+'.xml')
tree.write('assets/environments/total_environment.xml')
OBJECT_MESH_ID=0
OBJECT_BODY_ID=1
PALM_BODY_ID=4
OBJECT_GEOM_ID=INFORMATION_DICT[args.object]['OBJECT_GEOM_ID']
HAND_BEGIN=INFORMATION_DICT[args.object]['HAND_BEGIN']
HAND_END_ADD1=INFORMATION_DICT[args.object]['HAND_END_ADD1']
FLOOR_GEOM_ID=INFORMATION_DICT[args.object]['FLOOR_GEOM_ID']
SKY_GEOM_ID=INFORMATION_DICT[args.object]['SKY_GEOM_ID']
TABLE_GEOM_ID=INFORMATION_DICT[args.object]['TABLE_GEOM_ID']
HAND_POSE_IN_OBJBODY=INFORMATION_DICT[args.object]['HAND_POSE_IN_OBJBODY']
HAND_JOINT_POSE=INFORMATION_DICT[args.object]['HAND_JOINT_POSE']
desired_grasp_list=[0,0,0,0,0,0,0,0,HAND_JOINT_POSE['robot0:FFJ3'],HAND_JOINT_POSE['robot0:FFJ2'],HAND_JOINT_POSE['robot0:FFJ1'],HAND_JOINT_POSE['robot0:FFJ0'],HAND_JOINT_POSE['robot0:MFJ3'],HAND_JOINT_POSE['robot0:MFJ2'],HAND_JOINT_POSE['robot0:MFJ1'],HAND_JOINT_POSE['robot0:MFJ0'],HAND_JOINT_POSE['robot0:RFJ3'],HAND_JOINT_POSE['robot0:RFJ2'],HAND_JOINT_POSE['robot0:RFJ1'],HAND_JOINT_POSE['robot0:RFJ0'],HAND_JOINT_POSE['robot0:LFJ4'],HAND_JOINT_POSE['robot0:LFJ3'],HAND_JOINT_POSE['robot0:LFJ2'],HAND_JOINT_POSE['robot0:LFJ1'],HAND_JOINT_POSE['robot0:LFJ0'],HAND_JOINT_POSE['robot0:THJ4'],HAND_JOINT_POSE['robot0:THJ3'],HAND_JOINT_POSE['robot0:THJ2'],HAND_JOINT_POSE['robot0:THJ1'],HAND_JOINT_POSE['robot0:THJ0']]

# acquire the object's pose in the world coordinate system
object2hand_trans=HAND_POSE_IN_OBJBODY[:3]
object2hand_rot=R.from_quat(HAND_POSE_IN_OBJBODY[3:]).as_matrix()
hand2object_trans=np.dot(np.linalg.inv(object2hand_rot),np.array([0,0,0])-np.array(object2hand_trans))
forearm2object_trans=hand2object_trans+np.array([0,0,0.43])
forearm2object_rot=np.linalg.inv(object2hand_rot)
tree = ET.parse('assets/robots/adroit/base.xml')
root = tree.getroot()
element = root.find('.//body[@name="forearm"]')
world2forearm_trans = [float(element.attrib['pos'].split(' ')[0]),float(element.attrib['pos'].split(' ')[1]),float(element.attrib['pos'].split(' ')[2])]
world2forearm_rot = R.from_euler("zyx",[float(element.attrib['euler'].split(' ')[2]),float(element.attrib['euler'].split(' ')[1]),float(element.attrib['euler'].split(' ')[0])]).as_matrix()
world2object_trans=np.dot(world2forearm_rot,forearm2object_trans)+np.array(world2forearm_trans)
world2object_rot=np.dot(world2forearm_rot,forearm2object_rot)
world2object_euler=R.from_matrix(world2object_rot).as_euler("zyx")
world2object_euler=[world2object_euler[2],world2object_euler[1],world2object_euler[0]] 

# set the object to the correct initial pose
tree = ET.parse('assets/objects/'+args.object+'.xml')
root = tree.getroot()
element = root.find('.//body[@name="object"]')
backup_origin_pos=element.attrib['pos']
backup_origin_euler=element.attrib['euler']
element.set('pos','0 0 0')
element.set('euler','0 0 0')
tree.write('assets/objects/'+args.object+'.xml')

# model and data
m = mujoco.MjModel.from_xml_path('assets/environments/total_environment.xml')
d = mujoco.MjData(m)
get_information(m)
# restore the object xml 
tree = ET.parse('assets/objects/'+args.object+'.xml')
root = tree.getroot()
element = root.find('.//body[@name="object"]')
element.set('pos',backup_origin_pos)
element.set('euler',backup_origin_euler)
tree.write('assets/objects/'+args.object+'.xml')

object_index = m.mesh_vertadr[OBJECT_MESH_ID]#first vertex address
object_vertices_num = m.mesh_vertnum[OBJECT_MESH_ID]#number of vertices
object_vertices = m.mesh_vert[object_index:object_index+object_vertices_num]

object_body2geom_trans = m.geom_pos[OBJECT_GEOM_ID]
object_body2geom_rot = R.from_quat(
  (list(m.geom_quat[OBJECT_GEOM_ID][1:4]) + list([m.geom_quat[OBJECT_GEOM_ID][0]])) / np.linalg.norm(
    m.geom_quat[OBJECT_GEOM_ID])).as_matrix()

# acquire object vertices in the object's body coordinate system
object_vertices_body=[]
for point in object_vertices:
  object_vertices_body.append((np.dot(object_body2geom_rot,np.array(point))+object_body2geom_trans).tolist())

hand_contact_list=[]
for k in range(HAND_BEGIN,HAND_END_ADD1):
  hand_contact_list.append(k)
excluded_contact_list=[FLOOR_GEOM_ID,SKY_GEOM_ID,TABLE_GEOM_ID]

with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < 1000:
    if os.path.exists('acquire_information.json'):
      with open("acquire_information.json", 'r', encoding='utf-8') as f:
        data_dict = json.load(f)
    else:
      data_dict={}
    step_data_dict = {}
    step_start = time.time()

    mujoco.mj_step(m, d)
    if (time.time() - start)<5:
        ctrl_set_action(m,d,np.array(desired_grasp_list))
    if (time.time()-start)>=5 and (time.time()-start)<6:
        d.qpos[:6]=world2object_trans.tolist()+world2object_euler


    world2objectbody_tran = d.xpos[OBJECT_BODY_ID]
    world2objectbody_rot = np.array(d.xmat[OBJECT_BODY_ID]).reshape((3, 3))

    # end points of the palm direction vector in the object coordinate system
    wrj_xmat=np.array(d.xmat[PALM_BODY_ID]).reshape((3,3))
    palm_direction_vector_end_world = np.dot(wrj_xmat,np.array([0, 0, 1]))
    palm_direction_vector_begin_world = np.array([0,0,0])
    palm_direction_vector_end_objbody = np.dot(np.linalg.inv(world2objectbody_rot),palm_direction_vector_end_world-world2objectbody_tran)
    palm_direction_vector_begin_objbody = np.dot(np.linalg.inv(world2objectbody_rot),palm_direction_vector_begin_world-world2objectbody_tran)
    palm_direction_vector_objbody=[palm_direction_vector_begin_objbody.tolist(),palm_direction_vector_end_objbody.tolist()]

    # contact point set in the object coordinate system
    contact_point_objbody = []
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1
      for j, c in enumerate(d.contact):
          if c.geom1 not in excluded_contact_list and c.geom2 not in excluded_contact_list and (not (c.geom1 in hand_contact_list and c.geom2 in hand_contact_list)):
            pos=d.contact[j].pos
            contact_point_objbody.append(np.dot(np.linalg.inv(world2objectbody_rot),pos-world2objectbody_tran).tolist())
    
    # consistent with the normalization operation in dexgraspnet
      xcenter = (np.max(np.array(object_vertices_body)[:, 0]) + np.min(np.array(object_vertices_body)[:, 0])) / 2
      ycenter = (np.max(np.array(object_vertices_body)[:, 1]) + np.min(np.array(object_vertices_body)[:, 1])) / 2
      zcenter = (np.max(np.array(object_vertices_body)[:, 2]) + np.min(np.array(object_vertices_body)[:, 2])) / 2
      object_vert1 = np.array(object_vertices_body) - np.array([xcenter, ycenter, zcenter])
      dmax = np.max(np.sqrt(np.sum(np.square(object_vert1), axis=1))) * 1.03
      # end points of the palm direction vector in the normalized object coordinate system
      palm_direction_vector_normal_objbody = np.array(palm_direction_vector_objbody) - np.array([xcenter, ycenter, zcenter])
      palm_direction_vector_normal_objbody /= dmax
      step_data_dict['dmax']=dmax
      step_data_dict['object center']=[xcenter,ycenter,zcenter]
      step_data_dict['palm direction vector(in the normalized object coordinate system)'] = palm_direction_vector_normal_objbody.tolist()

      # contact point set in the normalized object coordinate system
      if len(contact_point_objbody) > 0:
        contact_normal_objbody=np.array(contact_point_objbody)-np.array([xcenter, ycenter, zcenter])
        contact_normal_objbody/=dmax
        step_data_dict['contact_set(in the normalized object coordinate system)']=contact_normal_objbody.tolist()
      else:
        step_data_dict['contact_set(in the normalized object coordinate system)']=[]
    data_dict[str(time.time() - start)] = step_data_dict

    with open("acquire_information.json", 'w', encoding='utf-8') as f:
      json.dump(data_dict, f, ensure_ascii=False)
    viewer.sync()
  
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)






