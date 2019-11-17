#!/bin/bash
#Configs="default 80s 70s 60s 50s 40s 30s 20s 10s 0s 90s035x0018z 50s035x0018z 45s"
Configs="10s 0s 90s035x0018z 50s035x0018z 45s"

mkdir -p Wyniki
cd Wyniki

for config in $Configs; do
   mkdir -p $config
   cd $config
   bash $HOME/velma/src/stero_velma/system_files/run_config.sh $config
   wait
   bash $HOME/velma/src/stero_velma/system_files/single_config_run.sh
   wait
   cd ..
done
