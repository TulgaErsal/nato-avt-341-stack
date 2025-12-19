#!/bin/bash

set -e

PROGRESS=plain
VERSION=$(git rev-parse --short HEAD)
BASE_PATH=$(realpath $(dirname "$0"))
CONTEXT=$(realpath ${BASE_PATH})/../../
BUILD_PATH=$(realpath "${BASE_PATH}/../build")

printf "Docker build context: $(realpath ${CONTEXT})\n"
printf "Docker build file context ${BUILD_PATH}\n"

cd ${CONTEXT}

printf "Building tulgaersal/nato-avt-341-stack:ros2-devel ..."
docker build --build-arg "TARGET=cpu" \
             --file ${BUILD_PATH}/ros2/devel/Dockerfile \
             --progress ${PROGRESS} \
             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel \
             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel-latest \
             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel-${VERSION} \
             .

printf "Building tulgaersal/nato-avt-341-stack:ros2-runtime ..."
docker build --build-arg "TARGET=cpu" \
             --file ${BUILD_PATH}/ros2/runtime/Dockerfile \
             --progress ${PROGRESS} \
             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-runtime \
             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-runtime-latest \
             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-runtime-${VERSION} \
             .

#printf "Building tulgaersal/nato-avt-341-stack:ros2-devel-cuda ..."
#docker build --build-arg "TARGET=cuda" \
#             --file ${BUILD_PATH}/ros2/devel/Dockerfile \
#             --progress ${PROGRESS} \
#             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel-cuda \
#             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel-cuda-latest \
#             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel-cuda-${VERSION} \
#             .

#printf "Building tulgaersal/nato-avt-341-stack:ros2-runtime-cuda ..."
#docker build --build-arg "TARGET=cuda" \
#             --file ${BUILD_PATH}/ros2/runtime/Dockerfile \
#             --progress ${PROGRESS} \
#             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-runtime-cuda \
#             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-runtime-cuda-latest \
#             --tag ghcr.io/tulgaersal/nato-avt-341-stack:ros2-runtime-cuda-${VERSION} \
#             .