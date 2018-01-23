sudo apt install hostapd haveged dnsmasq
sudo cp hostapd.conf /etc/hostapd/hostapd.conf
sudo cp sysctl.conf /etc/sysctl.conf
sudo cp rc.local /etc/rc.local
sudo cp dnsmasq.conf /etc/dnsmasq.conf
sudo cp interfaces /etc/network/interfaces
sudo cp NetworkManager.conf /etc/NetworkManager/NetworkManager.conf
sudo chmod +x /etc/rc.local

