#!/bin/sh

#Inicializacao do CAN BUS
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0

#Obtencao de nós virtuais
gnome-terminal -- /bin/bash /home/mayara-cruz/embarcados2021/Projeto-braco/master.sh

gnome-terminal -- /bin/bash /home/mayara-cruz/embarcados2021/Projeto-braco/servo1.sh

gnome-terminal -- /bin/bash /home/mayara-cruz/embarcados2021/Projeto-braco/servo2.sh

gnome-terminal -- /bin/bash /home/mayara-cruz/embarcados2021/Projeto-braco/candumpLog.sh
