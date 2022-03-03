#!/bin/bash
RUN_DIR=$(dirname $(readlink -f $0))
PKG_DIR=$(dirname ${RUN_DIR})

DISTRO_DEFAULT='foxy'

ROS_DOMAIN_ID="0"
DISTRO=${DISTRO_DEFAULT}

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: run.sh [OPTIONS...]
  OPTIONS:
    -h, --help          Show this help
    -i, --ros-domain-id ID  ROS_DOMAIN_ID (Default: ${ROS_DOMAIN_ID})
    -d, --distro        ROS distro. (default: ${DISTRO_DEFAULT})
_EOS_
  exit 1
}

while (( $# > 0 )); do
  if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
    usage_exit
  elif [[ $1 == "-i" ]] || [[ $1 == "--ros-domain-id" ]]; then
    if [[ $2 == -* ]]; then
      echo "Invalid parameter"
      usage_exit
    else
      ROS_DOMAIN_ID=$2
    fi
    shift 2
  elif [[ $1 == "-d" ]] || [[ $1 == "--distro" ]]; then
    if [[ $2 == -* ]]; then
      echo "Invalid parameter"
      usage_exit
    fi
    DISTRO=$2
    shift 2
  else
    echo "Invalid parameter: $1"
    usage_exit
  fi
done

DOCKER_VOLUME="${DOCKER_VOLUME} -v ${PKG_DIR}:/workspace/src/pointsmap_renderer:rw"
DOCKER_ENV="${DOCKER_ENV} -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

docker run \
  -it \
  --rm \
  --net host \
  ${DOCKER_VOLUME} \
  ${DOCKER_ENV} \
  --name ros-pointsmap-renderer \
  shikishimatasakilab/pointsmap_renderer:${DISTRO}
