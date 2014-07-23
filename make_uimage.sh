#!/bin/sh 

cp .config config_now
make uImage
cp arch/arm/boot/uImage /opt/tftpboot/uImage
