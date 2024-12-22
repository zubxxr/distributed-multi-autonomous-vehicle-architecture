#!/bin/bash

# Launch Unity Hub in Terminal 1
gnome-terminal -- bash -c "cd Unity/ && ./UnityHub.AppImage; exec bash"

# Launch VS Code in Terminal 2
gnome-terminal -- bash -c "cd Unity/AWSIM-Labs/Assets/Scripts/ && code .; exec bash"

# Activate Python virtual environment in Terminal 3
gnome-terminal -- bash -c "cd Multi-AVP/ && source env/bin/activate && echo Activated: \$VIRTUAL_ENV && exec bash"
