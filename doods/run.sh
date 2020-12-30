#!/usr/bin/env bash 

camera(){
  echo "[+] turn on camera .."
  cd /home/ubuntu/ros-docker-camera
  docker-compose up -d 
}

stop_camera(){

  cd /home/ubuntu/ros-docker-camera
  docker-compose down
  cd /home/ubuntu/shared
  docker-compose down
}

detector(){
  echo "[+] open object detector is running ... "
  docker run -it --rm -p 8080:8080 snowzach/doods:arm64
}

translator(){
	sleep 5
	echo "[+] translator started ..."
  cd /home/ubuntu/shared
  docker-compose up
}



coproc cam (camera)
echo "[+] cam started ${cam_PID}"
coproc trans (translator)
detector

echo "[+] waiting to finish the process"
stop_camera
