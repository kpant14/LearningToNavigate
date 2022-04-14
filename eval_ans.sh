#!/bin/bash

variable=$(conda info | grep -i 'base environment' | awk '{ print $4 }')
source ${variable}/etc/profile.d/conda.sh
conda activate habitat_nav

python eval_policy.py --split val --eval 1 --train_global 0 --train_local 0 --train_slam 0 --load_global pretrained_models/model_best.ppo --load_local pretrained_models/model_best.local --load_slam pretrained_models/model_best.slam -n 1 --print_images 1 --agent ans