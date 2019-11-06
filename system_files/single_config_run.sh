#!/bin/bash
for i in {1..10}
do
   bash $HOME/velma/src/stero_velma/system_files/runtime.sh "${i}.txt"
   wait
done
clear
for i in {1..10}
do
   bash $HOME/velma/src/stero_velma/system_files/runtime2.sh "${i} - inverted.txt"
   wait
done
