#!/bin/bash

# Define the path for the temporary file
TEMP_CRON_FILE=$(mktemp)

# Copy python file to the home directory
FILE_NAME="send_igmp_report.py"
cp `rospack find aerial_robot_base`/scripts/$FILE_NAME $HOME/$FILE_NAME

# Retrieve the current crontab and save it to the temporary file
sudo crontab -l 2>/dev/null > "$TEMP_CRON_FILE"

# Add a new job
CRON_COMMAND="*/2 * * * * /usr/bin/python3 $HOME/$FILE_NAME >> /var/log/force_send_GIMP_report.log 2>&1"
grep -qxF "$CRON_COMMAND" "$TEMP_CRON_FILE" ||  echo "$CRON_COMMAND" >> "$TEMP_CRON_FILE"

# Update the crontab
sudo crontab "$TEMP_CRON_FILE"

# Delete the temporary file
rm -f "$TEMP_CRON_FILE"

echo "Crontab has been updated."
