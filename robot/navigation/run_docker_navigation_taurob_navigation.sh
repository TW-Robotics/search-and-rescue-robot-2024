#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONTAINER_NAME="taurob_navigation"

# this function will be called on [ctrl + c]
function trap_ctrlc()
{
    echo -e "\e[33m[ctrl+c] cought, stopping continater with name: $CONTAINER_NAME \e[0m"
    docker container stop $CONTAINER_NAME
    exit 2
}
# initialize trap to call trap_ctrlc function when signal 2 (SIGINIT) is received
trap "trap_ctrlc" 2

docker run -it --rm \
    --name $CONTAINER_NAME \
    --network=host \
    --privileged \
    -v ./src/taurob_navigation/configuration:/root/catkin_ws/src/taurob_navigation/configuration \
    taurob/navigation:$CONTAINER_NAME bash

echo -e "\e[32mContainer started: $CONTAINER_NAME\e[0m"

until docker container ls | grep -q $CONTAINER_NAME; do sleep 0.1; done
mkdir -p "$SCRIPT_DIR"/logs/
docker logs -f $CONTAINER_NAME > "$SCRIPT_DIR"/logs/$CONTAINER_NAME.log &

read -rp "Press enter to stop container: $CONTAINER_NAME"
trap_ctrlc