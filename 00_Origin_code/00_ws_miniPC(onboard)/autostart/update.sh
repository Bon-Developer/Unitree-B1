#!/bin/bash
sleep 7
scriptPath=/home/unitree/Unitree/autostart/
export SUDO_ASKPASS=${scriptPath}passwd.sh
export toStartlog=">>/home/unitree/Unitree/autostart/.startlog"
# Do startup applications
cd $scriptPath
# val=`ls -a | grep ".updateCheckedFlag" | wc -w`
val=1
if [[ $val -gt 0 ]]; then
  mv .startlog .startlog_bak
  rm .detailedVersion
  for dir in `cat .startlist.sh`
  do
      if [[ $dir = \#* ]] ; then
        echo $dir': skipped' >>/home/unitree/Unitree/autostart/.startlog
      else
        cd $dir
        echo $dir':'`sed -n '1p' version.txt` >> ${scriptPath}.detailedVersion
        ./$dir.sh
        sleep 3
      fi
      cd $scriptPath
  done
  cd $scriptPath
fi


sudo iptables -F
sudo iptables -t nat -F
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE

sudo iptables -A FORWARD -i wlan0 -o eth0  -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT


# end
