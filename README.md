# LearningToNavigate

#### Preparing conda env
Assuming you have [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/) installed, let's prepare a conda env:
```bash
# We require python>=3.7 and cmake>=3.10
conda create -n habitat python=3.7 cmake=3.14.0
conda activate habitat
```

#### conda install habitat-sim
- To install habitat-sim with bullet physics
   ```
   conda install habitat-sim withbullet -c conda-forge -c aihabitat
   ```

#### Clone the repository
```
git clone https://github.com/kpant14/LearningToNavigate.git
cd LearningToNavigate
```



Install Habitat-lab using following commands: 
```
git clone --branch stable https://github.com/facebookresearch/habitat-lab.git
cd habitat-lab
pip install -r requirements.txt
python setup.py develop --all # install habitat and habitat_baselines
```


#### Datasets
Dataset can be downloaded from [here](https://dl.fbaipublicfiles.com/habitat/data/datasets/pointnav/gibson/v1/pointnav_gibson_v1.zip) </br>
Gibson Scene datasets can be downloaded from [here](https://docs.google.com/forms/d/e/1FAIpQLScWlx5Z1DM1M-wTSXaa6zV8lTFkPmTHW1LqMsoCBDWsTDjBkQ/viewform)
</br>


#### Folder Structure
Folder structure should be as follows:</br>
```
LearningToNavigate/
  data/
    scene_datasets/
      gibson/
        Adrian.glb
        Adrian.navmesh
        ...
    datasets/
      pointnav/
        gibson/
          v1/
            train/
            val/
            ...
```

