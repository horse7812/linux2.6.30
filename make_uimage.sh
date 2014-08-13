#!/bin/sh 

cp .config config_now
make uImage
cp arch/arm/boot/uImage /opt/tftpboot/uImage


echo "cp arch/arm/boot/uImage /mnt/hgfs/windown-linux/board_9g25/gateway_images"
cp arch/arm/boot/uImage /mnt/hgfs/windown-linux/board_9g25/gateway_images




