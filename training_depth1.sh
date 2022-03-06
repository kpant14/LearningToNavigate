#!/bin/bash

variable=$(conda info | grep -i 'base environment' | awk '{ print $4 }')
source ${variable}/etc/profile.d/conda.sh
conda activate habitat
python main.py --print_images 1 -d ./training_data/ -el 10000 --task generate_train
python train_depth1.py