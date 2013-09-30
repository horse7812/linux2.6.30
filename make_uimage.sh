#!/bin/sh 

make uImage
cp arch/arm/boot/uImage /var/lib/tftpboot/uImage
