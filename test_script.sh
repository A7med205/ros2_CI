#!/bin/bash
set -e  # Exit on error

# Variables
IMAGE_NAME="a7med205/ahmedhmair-cp22:tortoisebot-ros2-test"
CONTAINER_NAME="test_container2"
SIMULATION_TIMEOUT=${SIMULATION_TIMEOUT:-100s} # Default timeout of 120s

# Print current directory and contents
pwd
ls -la

# Build Docker image
echo "Building Docker image..."
sudo docker build -t "$IMAGE_NAME" .

# Run container
echo "Starting container..."
sudo docker run -d --rm --name "$CONTAINER_NAME" \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "$IMAGE_NAME" bash -c \
  'source /ros2_ws/install/setup.bash && ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True'

# Wait for simulation readiness
echo "Waiting for simulation to initialize..."
sleep 20s

# Run action server
echo "Running action server..."
sudo docker exec -d "$CONTAINER_NAME" bash -c \
  'source /ros2_ws/install/setup.bash && ros2 run tortoisebot_waypoints tortoisebot_action_server'

# Run test
echo "Running tests..."
sudo docker exec "$CONTAINER_NAME" bash -c \
  'source /ros2_ws/install/setup.bash && colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+'

# Cleanup: Kill simulation after timeout
echo "Simulation timeout: ${SIMULATION_TIMEOUT}..."
sleep "$SIMULATION_TIMEOUT" && docker container kill "$CONTAINER_NAME" || true

# Print finish text
echo "Job finished successfully"
