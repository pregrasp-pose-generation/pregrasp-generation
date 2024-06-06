# Installation Instructions

Please run the following commands to install our codebase:
```
git clone --recurse-submodules git@github.com:facebookresearch/TCDM.git && cd TCDM
conda env create -f environment.yml && conda activate tcdm
pip install -r requirements.txt
python setup.py develop
export MUJOCO_GL=egl        # NOTE: YOU PROBABLY WANT TO ADD THIS TO .bashrc
```

You should now be able to import our environment suite (python code below) and train policies. Happy experimenting!
```
>>> from tcdm import suite
>>> env = suite.load('hammer', 'use1')
```

