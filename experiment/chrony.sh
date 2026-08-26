#!/bin/bash
# This script checks if chrony is installed and running, and if not, it installs and starts it. Then it restarts the chrony service and waits for synchronization. If synchronization takes too long, it forces a time step.

# Define color variables for output
GREEN=$'\e[32m'
YELLOW=$'\e[33m'
RED=$'\e[31m'
RESET=$'\e[0m'

MAX_TIME=10  # seconds
SYNCED=0

# check if chrony exists
if command -v chronyc &> /dev/null
then
    echo "[INFO] ${GREEN}Chrony is installed${RESET}"
else
    echo "[INFO] ${YELLOW}Chrony could not be found, installing chrony...${RESET}"
    sudo apt install chrony
fi

# check if chrony is running
if systemctl is-active --quiet chrony
then
    echo "[INFO] ${GREEN}Chrony is running${RESET}"
else
    echo "[INFO] ${YELLOW}Chrony is not running, starting chrony...${RESET}"
    sudo systemctl start chrony
fi

# restart chrony service
echo "[INFO] ${YELLOW}Restarting chronyc service${RESET}"
sudo systemctl restart chrony

# wait 1 second for the service to restart
sleep 1

chronyc sources

# Ctrl+C handler
current_pid=""

cleanup() {
    echo "[INFO] Ctrl+C → exiting cleanly"

    if [ -n "$current_pid" ]; then
        kill "$current_pid" 2>/dev/null
    fi

    exit 1
}

trap cleanup SIGINT

# Force sycnchnization with server
sudo chronyc burst 10/20

while [ "$SYNCED" -eq 0 ]; do

    timeout --signal=INT "$MAX_TIME" chronyc waitsync 0 0.001 100 1 &
    current_pid=$!

    if wait "$current_pid"; then
        echo "[INFO] ${GREEN}Synced in time${RESET}"
        SYNCED=1
    else
        echo "[INFO] ${RED}Waitsync took too long → forcing step${RESET}"
        sudo chronyc makestep
        echo "[INFO] Forced time step"
    fi

    current_pid=""
done

chronyc tracking