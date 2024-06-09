from util import *
# desired grasp and finger joint pose reference
# hammer
HAMMER_HAND_POSE_IN_OBJBODY =  [0.04902601867230829, -0.0539545878946731, 0.015932237310727425, -0.07257110829393391, 0.644432059689558, 0.7611783304520612, -0.006950103270944287]#[0.045727008221729516, -0.04945806741672108, 0.025896688075467147, -0.051674922652902376, 0.7052531364605145, 0.7070371432201024, 0.0067966159653046095]
HAMMER_HAND_JOINT_POSE = {'robot0:FFJ3': -0.0016731862590053219, 'robot0:FFJ2': 1.5127485315939382, 'robot0:FFJ1': 1.02400825970579, 'robot0:FFJ0': 0.983929620109556, 'robot0:MFJ3': 0.001019932193414375, 'robot0:MFJ2': 1.5614525870369924, 'robot0:MFJ1': 1.0358679561126742, 'robot0:MFJ0': 0.9965470918905945, 'robot0:RFJ3': -0.00042412913521778795, 'robot0:RFJ2': 1.5613792631162917, 'robot0:RFJ1': 1.0246817401451311, 'robot0:RFJ0': 0.9846760772738956, 'robot0:LFJ4': 0.004700975157808962, 'robot0:LFJ3': -0.027162253022301745, 'robot0:LFJ2': 1.4207645868191507, 'robot0:LFJ1': 1.0239463929341535, 'robot0:LFJ0': 0.9838523694019293, 'robot0:THJ4': 0.4197568898874011, 'robot0:THJ3': 1.241971847688739, 'robot0:THJ2': 0.2525730425059415, 'robot0:THJ1': -0.13287365977805685, 'robot0:THJ0': -0.7717249456939383}#{"robot0:FFJ3": 0.023384790613801904, "robot0:FFJ2": 1.5579785931207446, "robot0:FFJ1": 1.0249559100895693, "robot0:FFJ0": 0.9851826405981005, "robot0:MFJ3": -0.004694632617074976, "robot0:MFJ2": 1.561981077356856, "robot0:MFJ1": 1.0572207716015372, "robot0:MFJ0": 1.0194365810677901, "robot0:RFJ3": -0.0005977565775568668, "robot0:RFJ2": 1.5614267307388816, "robot0:RFJ1": 1.024767902385633, "robot0:RFJ0": 0.984809577334278, "robot0:LFJ4": 0.0035486506203113322, "robot0:LFJ3": -0.03951222578578184, "robot0:LFJ2": 1.4179014838034403, "robot0:LFJ1": 1.0240948266295027, "robot0:LFJ0": 0.9840094095314703, "robot0:THJ4": 0.31851007276925464, "robot0:THJ3": 1.2318342276214829, "robot0:THJ2": 0.2528727376726022, "robot0:THJ1": -0.18771641925932084, "robot0:THJ0": -0.9833350541295902}
# knife
KNIFE_HAND_POSE_IN_OBJBODY=[-0.07365477522284386, -0.06484037558647951, 0.02372236287006884, 0.5317139360151198, 0.5114857196420719, 0.46197258557538323, 0.49218287152880086]
KNIFE_HAND_JOINT_POSE={'robot0:FFJ3': -0.021726599143519723, 'robot0:FFJ2': 1.5597891737024219, 'robot0:FFJ1': 1.2678971512232797, 'robot0:FFJ0': 1.2792366385940928, 'robot0:MFJ3': 0.010936102772333332, 'robot0:MFJ2': 1.5611593573236258, 'robot0:MFJ1': 1.2554249557053718, 'robot0:MFJ0': 1.2623073401569582, 'robot0:RFJ3': 0.0012233931233515379, 'robot0:RFJ2': 1.56098495266736, 'robot0:RFJ1': 1.271117344880421, 'robot0:RFJ0': 1.2852477949877337, 'robot0:LFJ4': 0.009721272125999986, 'robot0:LFJ3': -0.04321207007033845, 'robot0:LFJ2': 1.4220372545992115, 'robot0:LFJ1': 1.357203651114484, 'robot0:LFJ0': 1.487528488249763, 'robot0:THJ4': 0.6547535076593693, 'robot0:THJ3': 1.225246839804224, 'robot0:THJ2': 0.2524206098817223, 'robot0:THJ1': 0.16264927151953035, 'robot0:THJ0': -0.8051746481651859}
# flute
FLUTE_HAND_POSE_IN_OBJBODY =   [0.04447267326764582, -0.07413848053328578, 0.02401101468826101, -0.06548920844215084, 0.7271766325693708, 0.6806787572424652, -0.06001448209844031]
FLUTE_HAND_JOINT_POSE = {'robot0:FFJ3': 0.004929465722538445, 'robot0:FFJ2': 1.369856330206327, 'robot0:FFJ1': 1.4626938169591768, 'robot0:FFJ0': 1.5302172654870803, 'robot0:MFJ3': -0.01325764264921796, 'robot0:MFJ2': 1.561227461882219, 'robot0:MFJ1': 1.4033250999883509, 'robot0:MFJ0': 1.460231631188409, 'robot0:RFJ3': 0.003878653764550705, 'robot0:RFJ2': 1.5543033913389612, 'robot0:RFJ1': 1.3704867681094954, 'robot0:RFJ0': 1.4777944178224824, 'robot0:LFJ4': 0.012643964129202172, 'robot0:LFJ3': -0.003477840262084248, 'robot0:LFJ2': 1.5273629483068822, 'robot0:LFJ1': 1.4330559990702199, 'robot0:LFJ0': 1.4935346625754748, 'robot0:THJ4': 0.7462058184173298, 'robot0:THJ3': 1.3006967815659871, 'robot0:THJ2': 0.25352735536679627, 'robot0:THJ1': 0.16356363905646495, 'robot0:THJ0': -0.846432798226273}

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






