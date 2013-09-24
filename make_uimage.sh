#!/bin/sh 

make uImage
cp arch/arm/boot/uImage /opt/tftpboot/uImage
