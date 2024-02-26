#!/bin/sh

sudo apt-get update
sudo apt-get install -y libssl-dev libproj-dev

echo "[Desktop Entry]" > flight-route-planner.desktop
echo "Encoding=UTF-8" >> flight-route-planner.desktop
echo "Version=1.0" >> flight-route-planner.desktop
echo "Type=Application" >> flight-route-planner.desktop
echo "Terminal=false" >> flight-route-planner.desktop
echo "Icon=${PWD}/flight-route-planner.png" >> flight-route-planner.desktop
echo "Path=${PWD}/plan-bestanden" >> flight-route-planner.desktop
echo "Exec=${PWD}/flight-route-planner" >> flight-route-planner.desktop
echo "Name=FlightRoutePlanner" >> flight-route-planner.desktop

mv ./flight-route-planner.desktop /home/"${USER}"/.local/share/applications
