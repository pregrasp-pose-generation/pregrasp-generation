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
### Environment for simulation screening
Please refer to the README.md in simulation_screening. Assume your environment is called 'screening_validation'.
## Pre-grasp pose generation
### Acquiring information
#### Acquiring desired grasp
If you already have a desired grasp for [hammer/knife/flute], you can skip this step and proceed directly to the next one.
```
cd acquiring_information
python acquire_desiredgrasp.py --object [hammer/knife/flute]
```
Based on the program, you can use the simulator interface to adjust the relative position and orientation between the hand and the object as well as the angles of the joints to form a desired grasp. The relevant information is recorded in 'acquire_desiredgrasp.json'. In the json file, 'desired grasp' is a seven-element vector, with the first three elements representing the hand position and the last four elements representing the hand orientation in quaternion form, and 'joint pose' is a dictionary of finger joint angles at the desired grasp state.

#### Acquiring key information
Write the desired grasp information at the last timestep from 'acquire_desiredgrasp.json' into the 'acquire_information.py'. The file provides reference values. If there is a new desired grasp, you can modify the corresponding object variables accordingly.'OBJECTNAME_HAND_POSE_IN_OBJBODY' is expected to be assigned the value of the key 'desired grasp' while 'OBJECTNAME_HAND_JOINT_POSE' is expected to be assigned the value of the key 'joint pose'.
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
Copy and paste the content in 'acquire_information.json' into 'objectname.json' in 'generate_pregrasp/grasp_generation/object_json', then run the following command:
```
python main.py --object_code_list [hammer/knife/flute]
```
Once the process is complete, you can find the generated pre-grasp pose file for the object in 'generate_pregrasp/data/experiments/demo/results'.
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
The video of the execution process is stored in 'simulation_screening/check_quality_results'.
