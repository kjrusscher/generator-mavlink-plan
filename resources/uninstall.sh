#!/bin/sh

echo "Desktop koppeling wordt verwijderd..."
rm -f /home/"${USER}"/Desktop/flight-route-planner.desktop

echo "Applicatie wordt verwijderd..."
rm -f /home/"${USER}"/.local/share/applications/flight-route-planner.desktop

echo "De applicatie is verwijderd!"
