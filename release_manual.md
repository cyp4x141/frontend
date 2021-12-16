# release_manual

1.install dt_os
2.build and install workspace
3.create service
4.frontend


# install dt_os


# build and install workspace

$ catkin config --install && catkin build
$ sudo rm -rf src

# create service
### 1.Create a shell script for roslaunch
	-autostart.sh
	-rosbridge.sh 

shell script templates:
############################################################################################################
##########										templates.sh									 ###########
############################################################################################################

#!/bin/bash
source /opt/ros/($ROS_DISTRO)/setup.bash
source /home/($USER_NAME)/catkin_ws/devel/setup.bash
roslaunch ($PACKAGE_NAME) ($LAUNCH_NAME).launch

############################################################################################################
############################################################################################################

### 2.change permissions mode for bash
$ sudo chmod +x ($BASH_NAME).sh

### 3.create 2 services in /lib/systemd/system
	-autostart.serivce
	-rosbridge.service

service templates:
############################################################################################################
##########									templates.service									 ###########
############################################################################################################

[Unit]
Description=($SERVICE_NAME)

[Service]
Type=simple
ExecStart=/($START_NAME).sh
Restart=on-failure

[Install]
WantedBy=multi-user.target

############################################################################################################
############################################################################################################

### 4.reload systemctl_daemon
$ sudo systemctl daemon-reload

### 5.enable/disable service on boot
$ sudo systemctl enable ($SERIVCE_NAME).service
$ sudo systemctl disable ($SERVICE_NAME).service

# frontend
## npm pm2 setup
$ sudo apt install npm

## install pm2/serve as global
$ sudo npm install pm2 -g
$ sudo npm install serve -g 

## install dt-frontend
$ npm install
$ npm run build
$ cp build /var/www/dt-frontend

## enable/disable pm2 startup hookup
$ pm2 startup
$ pm2 unstartup

## serve a directory over http via port
pm2 serve ($DIRECTORY) ($PORT) 
--name ($PROCESS_NAME)
--spa always serving index.html on inexistant sub path

$ pm2 serve /var/www/dt-frontend/build/ 3000 --name "dt-frontent" --spa

## save pm2 config
$ pm2 save