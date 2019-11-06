#!/bin/bash

mkdir -p Wyniki
cd Wyniki

mkdir -p 50s
cd 50s
bash $HOME/velma/src/stero_velma/system_files/run_config.sh 50s
wait
bash $HOME/velma/src/stero_velma/system_files/single_config_run.sh
wait
cd ..

mkdir -p 40s
cd 40s
bash $HOME/velma/src/stero_velma/system_files/run_config.sh 40s
wait
bash $HOME/velma/src/stero_velma/system_files/single_config_run.sh
wait
cd ..
