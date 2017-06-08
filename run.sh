#!/bin/bash
sleep 1
gnome-terminal -t "server" -x bash -c "./out/server_bin 172.29.2.17 4569"
sleep 1
cat >./debug.gdb <<EOF
set args 172.29.2.17 4569
EOF
if [ "$1" == "gdb" ]; then
    gnome-terminal -t "client" -x bash -c "gdb ./out/client_bin -x debug.gdb"
else
    gnome-terminal -t "client" -x bash -c "./out/client_bin 172.29.2.17 4569"
fi

#./sync-client 172.29.2.17 4567
sleep 3


