# The lab's image (ur3e_experiments/Dockerfile: ROS 2 Humble + ur_robot_driver + ENME480 bridge),
# plus what slatphys_real needs. Build the lab image first:
#   cd ur3e_experiments && docker build -t ur3e-real:humble .
#   cd <this repo> && docker build -t ur3e-slatphys:humble -f docker/Dockerfile.robot .
FROM ur3e-real:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy python3-scipy python3-yaml python3-opencv python3-pil \
    ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*
