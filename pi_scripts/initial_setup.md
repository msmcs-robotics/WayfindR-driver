ubuntu
ubuntu

sudo nano /etc/netplan/50-cloud-init.yaml




network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "EagleNet": {}        # <-- empty mapping for EagleNet is an open network





sudo netplan try
sudo netplan generate



rm -rf /etc/ssh/sshd_config/*.conf



sudo nano /etc/ssh/sshd_config

#PubkeyAuthentication no
#PubkeyAuthentication yes

#PasswordAuthentication no
#PasswordAuthentication yes


PubkeyAuthentication yes

PasswordAuthentication yes



