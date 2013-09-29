#!/bin/sh 

make modules
make modules_install INSTALL_MOD_PATH=/home/barnard/work/rootfs
cp ./drivers/net/can/mcp251x.ko /home/barnard/work/rootfs/home
