#!/bin/sh

#Inicializa CAN BUS
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0

#Cria nós virtuais
gnome-terminal -- /bin/bash /home/joao-ishida/projetoEmbarcados2021/prjMotorEncoder/master.sh

gnome-terminal -- /bin/bash /home/joao-ishida/projetoEmbarcados2021/prjMotorEncoder/1slave.sh

gnome-terminal -- /bin/bash /home/joao-ishida/projetoEmbarcados2021/prjMotorEncoder/2slave.sh

gnome-terminal -- /bin/bash /home/joao-ishida/projetoEmbarcados2021/prjMotorEncoder/candumpLog.sh
