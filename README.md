# Generate Task-free and Finger-level Pre-grasp Poses for Dexterous Manipulation
This codebase is the official repository of "Generate Task-free and Finger-level Pre-grasp Poses for Dexterous Manipulation". It is constructed based on [DexGraspNet: A Large-Scale Robotic Dexterous Grasp Dataset for General Objects Based on Simulation](https://github.com/PKU-EPIC/DexGraspNet) and [Learning Dexterous Manipulation from Exemplar Object Trajectories and Pre-Grasps](https://github.com/facebookresearch/TCDM).
## Preparation
### Environment for acquiring information
```
conda create -n acquiring_information python=3.8
conda activate acquiring_information
pip install mujoco==3.1.4
```
### Environment for obtaining pre-grasp poses
Please refer to the README.md in 'generate_pregrasp/grasp_generation'. Assume your environment is called 'obtaining_pregrasp'.
### Environment for simulation screening and pre-grasp pose validation
Please refer to the README.md in 'simulation_screening'. Assume your environment is called 'screening_validation'.
## Pre-grasp pose generation
### Acquiring information
#### Acquiring desired grasp
If you already have a desired grasp for [hammer/knife/flute], you can skip this step and proceed directly to the next one.
```
conda activate acquiring_information
cd acquiring_information
python acquire_desiredgrasp.py --object [hammer/knife/flute]
```
Based on the program, you can use the simulator interface to adjust the relative position and orientation between the hand and the object as well as the angles of the joints to form a desired grasp. When you close the interface, the terminal will output your final desired grasp information including 'desired grasp' and 'joint pose'. 'desired grasp' is a seven-element vector with the first three elements representing the hand position and the last four elements representing the hand orientation in quaternion form and 'joint pose' is a dictionary of finger joint angles at the desired grasp state.

#### Acquiring key information
Write the desired grasp information into the 'acquire_information.py'. The file provides reference values. If there is a new desired grasp, you can modify the corresponding object variables accordingly.'OBJECTNAME_HAND_POSE_IN_OBJBODY' is expected to be assigned the value of the key 'desired grasp' while 'OBJECTNAME_HAND_JOINT_POSE' is expected to be assigned the value of the key 'joint pose'.
```
# hammer
HAMMER_HAND_POSE_IN_OBJBODY = []
HAMMER_HAND_JOINT_POSE = {}
# knife
KNIFE_HAND_POSE_IN_OBJBODY=[]
KNIFE_HAND_JOINT_POSE={}
# flute
FLUTE_HAND_POSE_IN_OBJBODY = []
FLUTE_HAND_JOINT_POSE = {}
```
After modifying, reproduce the desired grasp in the simulator to obtain key information:
```
python acquire_information.py --object [hammer/knife/flute]
```
The obtained key information is recorded in 'acquire_information.json'. In the json file, 'palm direction vector (in the normalized object coordinate system)' is the direction vector of the palm in the object coordinate system calculated based on the hand orientation of the desired grasp and 'contact_set (in the normalized object coordinate system)' is the set of coordinates in the object coordinate system of the contact points between the hand and the object.

### Obtaining pre-grasp poses
```
conda activate obtaining_pregrasp
cd ../generate_pregrasp/grasp_generation
```
Rename 'acquire_information.json' to 'objectname.json'. Copy and paste it in 'generate_pregrasp/grasp_generation/object_json', then run the following command:
```
python main.py --object_code_list [hammer/knife/flute]
```
Once the process is complete, you can find the generated pre-grasp pose file for the object in 'generate_pregrasp/data/experiments/demo/results'. There are already generated files in 'generate_pregrasp/data/experiments_backup'. They can be moved to 'generate_pregrasp/data/experiments/demo/results' to proceed directly to the next steps.
### Filtering pre-grasp poses
#### Numerical screening
After obtaining pre-grasp poses of the target object, run the following command for numerical screening:
```
python visualize_result.py --object_code [hammer/knife/flute]
```
Information of the pre-grasp pose passing the numerical screening will be stored in 'generate_pregrasp/grasp_generation/numerical_screening/objectname_correct.json'.
#### Simulation screening
```
conda activate screening_validation
cd ../../simulation_screening
```
Copy and paste the file 'generate_pregrasp/grasp_generation/numerical_screening/objectname_correct.json' into the 'simulation_screening/pregrasp_json' folder. Use the existing lift policy 'model.zip' to execute the task in the Mujoco simulator, starting with the synthesized pre-grasp pose as the initial hand state:
```
python check_quality.py --object [hammer/knife/flute]
```
If the object can be successfully grasped and maintained for 10 timesteps without falling, the pre-grasp pose passes the simulation screening.
The video of the execution process is stored in 'simulation_screening/check_quality_results/objectname'.
## Pre-grasp pose validation
```
conda activate screening_validation
cd ../desired_grasp_task
```
Since both the relocation and following trajectory tasks use the code of [Learning Dexterous Manipulation from Exemplar Object Trajectories and Pre-Grasps](https://github.com/facebookresearch/TCDM), only the code for the desired grasp task is provided here.
If you wish to train the desired grasp task for the hammer with our pre-grasp pose, please make the following changes.
```
# desired_grasp_task/experiments/env/pgdm.yaml
name: hammer-use1 

# desired_grasp_task/tcdm/envs/assets/robots/adroit/base.xml
<body name="forearm" pos="-0.3577507064421061 -0.3661215219640778 0.11861461372622818" quat="0.28582921277958334 0.28582921277958334 0.6467624456644059 0.6467624456644059">

# desired_grasp_task/tcdm/envs/mujoco/mj_models/objects.py
def object_generator(path):
    class __XMLObj__(ObjectModel):
        def __init__(self, pos=[-1.94971568e-03,  1.95830716e-02 , 1.23636204e-02], 
                       quat=[9.20504287e-01 ,-1.05889432e-03, -7.86119640e-04, -3.90730238e-01]):
            xml_path = asset_abspath(path)
            super().__init__(pos, quat, xml_path)
    return __XMLObj__
```

If you wish to train the desired grasp task for the knife with our pre-grasp pose, please make the following changes.
```
# desired_grasp_task/experiments/env/pgdm.yaml
name: knife-chop1 

# desired_grasp_task/tcdm/envs/assets/robots/adroit/base.xml
<body name="forearm" pos="-0.5622396495635502 -0.022484890235721186 0.10223137914106836" quat="0.5254778041869002 0.5254778041869002 0.47315227708097707 0.47315227708097707">

# desired_grasp_task/tcdm/envs/mujoco/mj_models/objects.py
def object_generator(path):
    class __XMLObj__(ObjectModel):
        def __init__(self, pos=[-1.94971568e-03,  1.95830716e-02 , 0.006004], 
                       quat=[1,0,0,0]):
            xml_path = asset_abspath(path)
            super().__init__(pos, quat, xml_path)
    return __XMLObj__
```

If you wish to train the desired grasp task for the flute with our pre-grasp pose, please make the following changes.
```
# desired_grasp_task/experiments/env/pgdm.yaml
name: flute-pass1 

# desired_grasp_task/tcdm/envs/assets/robots/adroit/base.xml
<body name="forearm" pos="0.1866254502621924 -0.4955123012651998 0.145782669719882" quat="-0.08816865059427532 -0.08816865059427532 0.7015884043029676 0.7015884043029676">

# desired_grasp_task/tcdm/envs/mujoco/mj_models/objects.py
def object_generator(path):
    class __XMLObj__(ObjectModel):
        def __init__(self, pos=[-0.00728576, -0.00140855 ,0.00838198], 
                       quat=[ 1,0,0,0]):
            xml_path = asset_abspath(path)
            super().__init__(pos, quat, xml_path)
    return __XMLObj__
```
You can correct the target desired grasp in target_desired_grasp and target_finger_joint_pose of the file 'desired_grasp_task/tcdm/envs/reference.py'. Then run the code to train a model:
```
python train.py
```