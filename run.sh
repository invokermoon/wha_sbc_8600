#!/bin/bash
sleep 1
gnome-terminal -t "server" -x bash -c "./out/server_bin 172.29.2.17 4569"
sleep 1
gnome-terminal -t "client" -x bash -c "./out/client_bin 172.29.2.17 4569"
#./sync-client 172.29.2.17 4567
sleep 3


