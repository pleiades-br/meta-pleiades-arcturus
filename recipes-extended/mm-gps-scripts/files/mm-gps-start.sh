#!/bin/bash

# Script to check modem manager status
# Exits after successful execution of mmcli -m 0 or after 10 attempts

MAX_ATTEMPTS=10
attempts=0

while [ $attempts -lt $MAX_ATTEMPTS ]; do
    attempts=$((attempts + 1))
    echo "Attempt $attempts of $MAX_ATTEMPTS: Checking modem status..."
    
    # Execute the mmcli command
    mmcli -m 0
    
    # Check the exit status of the command
    if [ $? -eq 0 ]; then
        echo "Success! Modem check completed without errors."
        mmcli -m 0 --command='AT+QGPS=1'
        exit 0
    else
        echo "Attempt failed. Exit code: $?"
        
        # If this is not the last attempt, wait a bit before trying again
        if [ $attempts -lt $MAX_ATTEMPTS ]; then
            echo "Waiting 2 seconds before next attempt..."
            sleep 2
        fi
    fi
done

echo "Maximum attempts ($MAX_ATTEMPTS) reached without success."
exit 1