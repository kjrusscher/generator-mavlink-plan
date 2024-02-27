#!/bin/sh

echo "Ik ga de benodigde software installeren, mag ik het wachtwoord?"
sudo apt-get update
sudo apt-get install -y libssl-dev libproj-dev

echo "Bezig met het genereren van de Desktop koppeling..."
echo "[Desktop Entry]" > flight-route-planner.desktop
echo "Encoding=UTF-8" >> flight-route-planner.desktop
echo "Version=1.0" >> flight-route-planner.desktop
echo "Type=Application" >> flight-route-planner.desktop
echo "Terminal=false" >> flight-route-planner.desktop
echo "Icon=${PWD}/flight-route-planner.png" >> flight-route-planner.desktop
echo "Path=${PWD}/plan-bestanden" >> flight-route-planner.desktop
echo "Exec=${PWD}/flight-route-planner" >> flight-route-planner.desktop
echo "Name=FlightRoutePlanner" >> flight-route-planner.desktop

echo "Bezig met het installeren van de applicatie..."
mv ./flight-route-planner.desktop /home/"${USER}"/.local/share/applications
gio set /home/"${USER}"/.local/share/applications/flight-route-planner.desktop metadata::trusted true
chmod a+x /home/"${USER}"/.local/share/applications/flight-route-planner.desktop
ln -sf /home/"${USER}"/.local/share/applications/flight-route-planner.desktop /home/"${USER}"/Desktop/flight-route-planner.desktop
gio set /home/"${USER}"/Desktop/flight-route-planner.desktop metadata::trusted true

echo "Installatie klaar!"
