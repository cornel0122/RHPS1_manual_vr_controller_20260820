#!/usr/bin/env bash

OPERATOR_PC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID="${RHPS1_ROS_DOMAIN_ID:-200}"
export RMW_IMPLEMENTATION="${RHPS1_RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_LOCALHOST_ONLY="${RHPS1_ROS_LOCALHOST_ONLY:-0}"
export ZENOH_SESSION_CONFIG_URI="${ZENOH_SESSION_CONFIG_URI:-$OPERATOR_PC_DIR/config/rhps1_operator_zenoh_session.json5}"
