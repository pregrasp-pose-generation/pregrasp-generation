# Dependencies

## Common Packages

```bash
conda create -n dexgraspnet python=3.9
conda activate dexgraspnet

conda install pytorch3d

conda install transforms3d
conda install trimesh
conda install plotly

pip install sapien  # simulator for generating table top scenes

pip install urdf_parser_py

conda install tensorboardx  # this seems to be useless
conda install tensorboard
conda install setuptools=59.5.0

conda install python-kaleido  # soft dependency for plotly

pip install yapf
conda install nbformat  # soft dependency for plotly
pip install networkx  # soft dependency for trimesh
conda install rtree  # soft dependency for trimesh
pip install --user healpy
```

## TorchSDF

[TorchSDF](https://github.com/wrc042/TorchSDF) is a our custom version of [Kaolin](https://github.com/NVIDIAGameWorks/kaolin). 

```bash
git clone git@github.com:wrc042/TorchSDF.git
cd torchsdf
git checkout 0.1.0
bash install.sh
```

## Pytorch Kinematics

We modified [pytorch_kinematics](https://github.com/UM-ARM-Lab/pytorch_kinematics) to increase calculation speed. The code is included in this repo. 

```bash
cd thirdparty/pytorch_kinematics
pip3 install -e .
```



