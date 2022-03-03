#!/bin/bash
RUN_DIR=$(dirname $(readlink -f $0))

docker build \
    -t shikishimatasakilab/pointsmap_renderer:foxy \
    -f ${RUN_DIR}/Dockerfile.foxy \
    ${RUN_DIR}
