#!/bin/bash

docker build --rm -q -t taurob/navigation:comehome_pose_tf2 -f Dockerfile.navigation.comehome_pose_tf2 .
docker build --rm -q -t taurob/navigation:taurob_navigation -f Dockerfile.navigation.taurob_navigation .