#!/bin/bash

# Define only the localization-relevant topics
TOPICS=(
  "/vehicle_count"
  "/parking_spots/empty"
  "/avp/dropoff_queue"
  "/parking_spots/reserved"
)

# Loop through each topic and echo it silently
for topic in "${TOPICS[@]}"
do
    echo "🔄 Echoing: $topic"
    ros2 topic echo "$topic" > /dev/null &
done

echo "✅ All essential localization topics are being echoed in the background."

