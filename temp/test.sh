#!/bin/bash

#<gui输入密码>
# password=`zenity --password`
# echo "$password"
# echo "$password" |sudo -S systemctl disable autostart.service

check_status="disabled"
autostart_status=`systemctl is-enabled autostart`
rosbridge_status=`systemctl is-enabled rosbridge`

#<zenity--info>
  zenity --title="info_test" --info --text="text=info" --ellipsize

#<zenity--error>
  zenity --error --text="text=error" --ellipsize

#<zenity--title>
input=$(zenity --title="title_test" --text "please input" --entry) 
zenity --info --title="input_ans" --text="input is $input" 

#<exit>
#exit 0