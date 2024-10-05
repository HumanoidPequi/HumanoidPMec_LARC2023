#!/bin/bash

# Nome da imagem Docker
IMAGE_NAME="martha_sim:latest"

CONTAINER_NAME="Marta_Simulation"
ROS_IP=$(hostname -I | awk '{print $1}')

# Comando para inicializar o contêiner Docker com bash
rocker --device /dev/dri --x11 \
  --name $CONTAINER_NAME \
  --network host \
  --env ROS_IP=$ROS_IP \
  --env DISPLAY=$DISPLAY \
  --env XAUTHORITY=$XAUTHORITY \
  --oyr-run-arg " -v /tmp:/tmp -v /var/log:/var/log -v /tmp/.X11-unix:/tmp/.X11-unix" \
  $IMAGE_NAME \
  ${@:-"bash"}