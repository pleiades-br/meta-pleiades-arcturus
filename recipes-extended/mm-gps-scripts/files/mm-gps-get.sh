#!/bin/bash 

# Set archive file path
ARCHIVE_FILE="/tmp/safetyrails/gps"

# Disconnect PPP 
nmcli con down lte-modem 
sleep 2 

# Send AT command and capture output
mmcli -m 0 --command="AT+QGPSLOC?" > "$ARCHIVE_FILE"

# Reconnect 
sleep 2
nmcli con up lte-modem