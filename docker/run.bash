#!/bin/env bash

# start in from docker directory

docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -v $(pwd)/../:/workspace \
    esp-builder shell