#!/bin/bash

service_name="spes_daemon"
script_name="spes_daemon"

user=$(id -u -n)
group=$(id -g -n)
echo $user:$group

sudo cp -v ${script_name}  /usr/bin/
sudo chown ${user}:${group} /usr/bin/${script_name}
sudo chmod +x /usr/bin/${script_name}

# set service parameters from install script
sudo cp -v ${service_name}.service  /lib/systemd/system/
sudo sed -i "s/User=%i/User=${user}/" /lib/systemd/system/${service_name}.service
sudo sed -i "s/Group=%i/Group=${group}/" /lib/systemd/system/${service_name}.service
sudo sed -i "s|ExecStart=/usr/bin/%i|ExecStart=/usr/bin/${script_name}|" /lib/systemd/system/${service_name}.service

# install service
systemctl stop ${service_name}.service
sudo systemctl daemon-reload

systemctl start ${service_name}.service
systemctl enable ${service_name}.service
systemctl status ${service_name}.service
