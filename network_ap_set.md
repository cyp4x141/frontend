# network_ap_set
tp-link pcie wireless card


# install some tools
$ sudo apt install ifupdown
$ sudo apt install hostapd isc-dhcp-server
then

# if cable + wireless are "eth0" + "wlan9"
$ ifconfig
eth0为ubuntu机器的有线接口。
wlan9为ubuntu机器无线接口。

## creat interfaces
creat /etc/network/interfaces and set value for ap-network
or
$ sudo cp interfaces /etc/network/

# creat wpa2.conf
sudo gedit /etc/hostapd/wpa2.conf
########################################
interface=wlan9
driver=nl80211
ssid=dt_car_control 
hw_mode=g
channel=11
macaddr_acl=0
auth_algs=3
wpa=2
wpa_passphrase=87654321
wpa_key_mgmt=WPA-PSK
rsn_pairwise=TKIP CCMP
########################################

# add wlan9 in /run/network/ifstate
wlan9=wlan9

# add subnet for dhcpd.conf
sudo gedit /etc/dhcp/dhcpd.conf
########################################
subnet 10.66.0.0 netmask 255.255.255.0
{
    range 10.66.0.2 10.66.0.10;
    option routers 10.66.0.1;
    option domain-name-servers 10.66.0.1;
}
########################################

# ip addr flush and reboot
$ sudo ip addr flush wlan9
$

# reboot

# set ap on
sudo ifup wlan9

# set ap off
sudo ifdown wlan9

