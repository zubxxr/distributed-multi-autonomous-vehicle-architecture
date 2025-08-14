# AVP Troubleshooting
- **Vehicle doesn’t move** → Check initial pose and that the goal lies on a valid lane/parking region.  
- **Vehicle 2 waiting for pose** → Verify Zenoh bridge connectivity and `/vehicle2` namespace.  
- **No parking spots detected** → Ensure detection server is running and publishing; confirm topics and frame IDs.
