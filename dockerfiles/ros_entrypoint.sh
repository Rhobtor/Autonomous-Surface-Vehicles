#!/bin/bash
set -e

# Check if the MQTT_ADDR environment variable is set
if [ -n "$MQTT_ADDR" ]; then
  NEW_MQTT_ADDR="$MQTT_ADDR"
  # Update the mqtt_addr in the YAML file
  sed -i.bak "s/mqtt_addr: .*/mqtt_addr: \"$NEW_MQTT_ADDR\"/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  echo "mqtt_addr has been updated to $NEW_MQTT_ADDR in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  BUILD_NEEDED=true
fi

# Check if the DEBUG environment variable is set
if [ -n "$DEBUG" ]; then
  # Convert the DEBUG value to lowercase to handle true/false correctly
  DEBUG_LOWER=$(echo "$DEBUG" | tr '[:upper:]' '[:lower:]')
  if [ "$DEBUG_LOWER" = "true" ] || [ "$DEBUG_LOWER" = "false" ]; then
    # Update the debug in the YAML file with boolean value
    sed -i.bak "s/debug: .*/debug: $DEBUG_LOWER/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"
    echo "debug has been updated to $DEBUG_LOWER in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"
    BUILD_NEEDED=true
  else
    echo "Invalid value for DEBUG: $DEBUG. Expected 'true' or 'false'."
    exit 1
  fi
fi

# Check if the MQTT_ADDR environment variable is set
if [ -n "$DB_HOST" ]; then
  NEW_DB_HOST="$DB_HOST"
  # Update the db_addr in the YAML file
  sed -i.bak "s/db_host: .*/db_host: \"$NEW_DB_HOST\"/" "/home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  echo "db_host has been updated to $NEW_DB_HOST in /home/asv_workspace/src/asv_loyola_us/config/config.yaml"
  BUILD_NEEDED=true
fi

# Build and source the ROS workspace if needed
if [ "$BUILD_NEEDED" = true ]; then
  colcon build
fi

# Check if the NAVIO_ADDR environment variable is set
if [ -n "$NAVIO_ADDR" ]; then
  NEW_NAVIO_ADDR="$NAVIO_ADDR"
fi

# Debug: Check if USE_SENSORS is passed correctly
echo "USE_SENSORS passed in as: $USE_SENSORS"

# Check if the USE_SENSORS environment variable is set
if [ -n "$USE_SENSORS" ]; then
  # Convert the USE_SENSORS value to lowercase to handle true/false correctly
  SENSOR_LOWER=$(echo "$USE_SENSORS" | tr '[:upper:]' '[:lower:]')

  # Debug: Check conversion
  echo "USE_SENSORS converted to lowercase: $SENSOR_LOWER"

  # Ensure the value is either 'true' or 'false'
  if [ "$SENSOR_LOWER" = "true" ] || [ "$SENSOR_LOWER" = "false" ]; then
    echo "USE_SENSORS has been set to $SENSOR_LOWER"
  else
    echo "Invalid value for USE_SENSORS: $USE_SENSORS. Expected 'true' or 'false'."
    exit 1
  fi
else
  # Set a default value if USE_SENSORS is not provided
  SENSOR_LOWER="true"
  echo "No USE_SENSORS provided. Defaulting to $SENSOR_LOWER"
fi

COMMAND="$@"

source /opt/ros/humble/setup.bash
source install/setup.bash
# Launch the ROS nodes
ros2 launch asv_loyola_us system.launch.py use_sensors:="$SENSOR_LOWER" & ros2 launch mavros apm.launch fcu_url:=tcp://"$NEW_NAVIO_ADDR" gcs_url:=udp://@

# Execute the main process
exec $COMMAND
