#!/usr/bin/env bash

USER_ID=`id -u`

IMAGE_TAG=env/alicevision

# Create the docker environment for building AliceVision
docker build -t $IMAGE_TAG docker/build_env/. \
    --build-arg "USER_ID=${USER_ID}" --build-arg "USER_NAME=${USER}"

# Start the command line
docker run --rm -it \
    -e DISPLAY=${DISPLAY} \
    -v $(pwd):/src \
    -v /media/datasets:/media/datasets \
    -v /tmp:/tmp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --runtime=nvidia -e NVIDIA_DRIVER_CAPABILITIES=all \
    --gpus all $IMAGE_TAG /bin/bash
