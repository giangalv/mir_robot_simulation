#!/bin/bash

# Start the MIR REST API server in the background
ros2 run mir_restapi mir_restapi_pause --ros-args -p mir_hostname:='130.251.13.90' -p mir_restapi_auth:='Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==' &
MIR_PID=$!

# Wait a few seconds to ensure the server is running
sleep 5

# Call the mir_set_pause_control service
ros2 service call /mir_250_set_pause std_srvs/Trigger

# Wait a moment for the service to complete
sleep 3

# Kill the MIR REST API server
kill $MIR_PID
echo "MIR REST API server stopped."