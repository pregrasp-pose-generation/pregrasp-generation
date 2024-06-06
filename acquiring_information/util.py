from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import mujoco
import mujoco.viewer
import argparse
import numpy as np
import json
import os
import xml.etree.ElementTree as ET

def get_information(m):
  mylen0=len(m.name_bodyadr) + len(m.name_jntadr) + len(m.name_geomadr) + len(m.name_siteadr) +len(m.name_camadr) + len(m.name_lightadr)
  mesh_name_list=(m.names.decode()).split(b'\x00'.decode())[1:-1]
  meshadr_list = m.mesh_vertadr
  mesh_len=len(meshadr_list)
  print("mesh:")
  for i in range(mesh_len):
    mylen=mylen0+i
    mesh_name=mesh_name_list[mylen]
    print(i,":", mesh_name)

  print("body:")
  l=[m.body(i).name for i in range(m.nbody)]
  for i in range(len(l)):
    print(i,":",l[i])

  print("geom:")
  l = [m.geom(i).name for i in range(m.ngeom)]
  for i in range(len(l)):
    print(i, ":", l[i])

  print("joint:")
  l = [m.joint(i).name for i in range(m.njnt)]
  for i in range(len(l)):
    print(i, ":", l[i])

def ctrl_set_action(model,data, action):
    if len(data.ctrl) > 0:
        for i in range(action.shape[0]):
            data.ctrl[i] =action[i]
