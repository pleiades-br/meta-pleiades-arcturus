#!/bin/bash 
# Save as /usr/local/bin/send-at-command.sh 

# Set archive file path
ARCHIVE_FILE="/tmp/safetyrails/gps"

# Disconnect PPP 
nmcli con down "lte-modem" 
sleep 2 

# Send AT command and capture output
TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
GPS_OUTPUT=$(mmcli -m 0 --command="AT+QGPSLOC?")

# Add timestamp and output to archive file
echo "[$TIMESTAMP] $GPS_OUTPUT" > "$ARCHIVE_FILE"

sleep 2

# Reconnect 
nmcli con up "lte-modem"