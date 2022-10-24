#!/bin/bash
read -p 'numero de terminales johanp :' num_terminal

for i in $(seq 1 $num_terminal);do
    echo $i
    if (($i == $num_terminal))
    then 
        sshpass -p javila2303 ssh -Y johanp@172.17.100.6
    else  
        gnome-terminal --tab --command="sshpass -p javila2303 ssh -Y johanp@172.17.100.6"
    fi
done

