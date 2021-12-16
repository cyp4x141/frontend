# command_line

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

$ cd /
$ sudo gedit autostart.sh

############################################################################################################
##########										autostart.sh									 ###########
############################################################################################################

#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/dzt/dt_ws/install/setup.bash
roslaunch dt_launch dt_.launch

############################################################################################################
############################################################################################################

$ sudo gedit rosbridge.sh

############################################################################################################
##########										rosbridge.sh									 ###########
############################################################################################################

#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/dzt/dt_ws/install/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch 

############################################################################################################
############################################################################################################

### 2.change permissions mode for bash
$ sudo chmod +x autostart.sh
$ sudo chmod +x rosbridge.sh

### 3.create 2 services in /lib/systemd/system

$ cd /lib/systemd/system
$ sudo gedit autostart.service
############################################################################################################
##########									autostart.service									 ###########
############################################################################################################

[Unit]
Description=autostart

[Service]
Type=simple
ExecStart=/autostart.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target

############################################################################################################
############################################################################################################

$ sudo gedit rosbridge.service
############################################################################################################
##########									rosbridge.service									 ###########
############################################################################################################

[Unit]
Description=rosbridge

[Service]
Type=simple
ExecStart=/rosbridge.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target

############################################################################################################
############################################################################################################

### 4.reload systemctl_daemon
$ sudo systemctl daemon-reload

### 5.enable service 
$ sudo systemctl enable autostart.service

# frontend
## npm pm2 setup
$ sudo apt install npm

## install pm2/serve as global
$ sudo npm install pm2 -g
$ sudo npm install serve -g 

## install dt-frontend
$ cd dt-frontend
$ npm install
$ npm run build
$ cp build /var/www/dt-frontend

## enable/disable pm2 startup hookup
$ pm2 startup

## serve directory(/var/www/dt-frontend/build/) over http via port 3000
$ pm2 serve /var/www/dt-frontend/build/ 3000 --name "dt-frontent" --spa

## save pm2 config

## off
$ pm2 save

## off
$ pm2 unstartup systemd