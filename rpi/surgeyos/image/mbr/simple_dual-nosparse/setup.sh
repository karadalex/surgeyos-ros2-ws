#!/bin/bash

set -eu

LABEL="$1"
BOOTUUID="$2"
ROOTUUID="$3"

case $LABEL in
   ROOT)
      case $IGconf_image_rootfs_type in
         ext4)
            cat << EOF2 > $IMAGEMOUNTPATH/etc/fstab
UUID=${ROOTUUID} /               ext4 rw,relatime,errors=remount-ro,commit=30 0 1
EOF2
            ;;
         btrfs)
            cat << EOF2 > $IMAGEMOUNTPATH/etc/fstab
UUID=${ROOTUUID} /               btrfs defaults 0 0
EOF2
            ;;
         *)
            ;;
      esac

      cat << EOF2 >> $IMAGEMOUNTPATH/etc/fstab
UUID=${BOOTUUID} /boot/firmware  vfat defaults,rw,noatime,errors=remount-ro 0 2
EOF2
      ;;
   BOOT)
      sed -i "s|root=\([^ ]*\)|root=UUID=${ROOTUUID}|" $IMAGEMOUNTPATH/cmdline.txt
      ;;
   *)
      ;;
esac
