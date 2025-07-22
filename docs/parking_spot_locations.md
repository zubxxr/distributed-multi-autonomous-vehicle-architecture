1. Launch Planning sim

2. Run: ros2 topic echo /planning/mission_planning/goal

3. Go to RViz and publish goal pose in each spot in incremental order.
4. Go back to ros2 topic output and u will see all messages at once in bulk. Copy full output down to last triple dash.
5. Paste into script: python3 generate_parking_spot_locations.py
