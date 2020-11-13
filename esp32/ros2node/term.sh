#!/bin/bash
ping_cancelled=false    # Keep track of whether the loop was cancelled, or succeeded
until date; ping -c1 "192.168.1.75" >/dev/null 2>&1; do :; done &    # The "&" backgrounds it
trap "kill $!; ping_cancelled=true" SIGINT
wait $!          # Wait for the loop to exit, one way or another
trap - SIGINT    # Remove the trap, now we're done with it
echo "Done pinging, cancelled=$ping_cancelled"
echo -ne '\007'
nc 192.168.1.75 23
