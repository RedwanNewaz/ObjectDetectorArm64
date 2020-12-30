#!/usr/bin/env bash 

camera(){
  echo "[+] turn on camera .."
  cd /home/ubuntu/ros-docker-camera
  docker-compose up -d 
}

stop_camera(){

  cd /home/ubuntu/ros-docker-camera
  docker-compose down 
}

detector(){
  echo "[+] open object detector is running ... "
  docker run -it --rm -p 8080:8080 snowzach/doods:arm64
}


republisher()
{
  docker run -it --rm --net host -v $(pwd):/home \
	  -e ROS_IP=192.168.1.136 \ 
	  -e ROS_MASTER_URI=http://192.168.1.136:11311 \
	  doods/ros bash -c "rosrun image_transport republish raw in:=/uav_camera/detected compressed out:=/uav_camera/detected"
}

classifier(){
  docker run -it --rm --net host  -v $(pwd):/home  \
	  -e ROS_IP=192.168.1.136 \
	  -e ROS_MASTER_URI=http://192.168.1.136:11311 \
	  doods/ros bash -c "/home/send.py"
}
#republisher
#classifier

docker run -it --rm  --net host \
	-e ROS_IP=192.168.1.136 \
	-e ROS_MASTER_URI=http://192.168.1.136:11311 \
       	doods/ros bash -c 'rosrun image_transport republish raw in:=/uav_camera/detected compressed out:=/uav_camera/detected'

