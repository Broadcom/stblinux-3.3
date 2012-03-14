#!/bin/bash

# UBIFS logical eraseblock limit - increase this number if mkfs.ubifs complains
max_leb_cnt=2047

# change to '1' to build NAND ubifs images (they might be large files,
# particularly for bigger eraseblock sizes)
build_nand_ubifs=0

LINUXDIR=linux

set -e

function make_ubi_img()
{
	pebk=$1
	page=$2

	peb=$(($1 * 1024))

	if [ $page -lt 64 ]; then
		leb=$(($peb - 64 * 2))
		minmsg="minimum write size $page (NOR)"
	else
		leb=$(($peb - $page * 2))
		minmsg="minimum write size $page (NAND)"
	fi

	out="images/ubifs-${pebk}k-${page}-${TARGET}.img"

	echo "Writing UBIFS image for ${pebk}kB erase, ${minmsg}..."

	bin/mkfs.ubifs -U -D misc/devtable.txt -r romfs -o tmp/ubifs.img \
		--nosquash-rino-perm -m $page -e $leb -c ${max_leb_cnt}

	vol_size=$(du -sm tmp/ubifs.img | cut -f1)

	cat > tmp/ubinize.cfg <<-EOF
	[ubifs]
	mode=ubi
	image=tmp/ubifs.img
	vol_id=0
	vol_size=${vol_size}MiB
	vol_type=dynamic
	vol_name=rootfs
	vol_flags=autoresize
	EOF

	bin/ubinize -o tmp/ubi.img -m $page -p $peb tmp/ubinize.cfg

	mv tmp/ubi.img $out
	echo "  -> $out"
}

function make_jffs2_img()
{
	pebk=$1
	out=images/jffs2-${pebk}k-${TARGET}.img

	echo "Writing JFFS2 image for ${pebk}kB eraseblock size (NOR)..."
	bin/mkfs.jffs2 -U -D misc/devtable.txt -r romfs \
		-o tmp/jffs2.img -e ${pebk}KiB $JFFS2_ENDIAN
	bin/sumtool -i tmp/jffs2.img -o $out -e ${pebk}KiB $JFFS2_ENDIAN
	echo "  -> $out"
}

#
# MAIN
#

TARGET=$(cat .target)

rm -rf tmp
mkdir -p tmp

if [[ "$TARGET" = *_be* ]]; then
	JFFS2_ENDIAN=-b
else
	JFFS2_ENDIAN=-l
fi

echo "Writing SQUASHFS image..."
rm -f images/squashfs-${TARGET}.img
bin/mksquashfs romfs images/squashfs-${TARGET}.img \
	-processors 1 -root-owned -p "/dev/console c 0600 0 0 5 1"
chmod 0644 images/squashfs-${TARGET}.img
echo "  -> images/squashfs-${TARGET}.img"
echo ""

# 64k erase / 1B unit size - NOR
make_ubi_img 64 1

# 128k erase / 1B unit size - NOR
make_ubi_img 128 1

# 256k erase / 1B unit size - NOR
make_ubi_img 256 1

if [ "$build_nand_ubifs" = "1" ]; then

	# 16k erase / 512B page - small NAND
	make_ubi_img 16 512

	# 128k erase / 2048B page - NAND
	make_ubi_img 128 2048

	# 256k erase / 4096B page - NAND
	make_ubi_img 256 4096

	# 512k erase / 4096B page - large NAND
	make_ubi_img 512 4096

	# 1MB erase / 4096B page - large NAND
	make_ubi_img 1024 4096

	# 1MB erase / 8192B page - large NAND
	make_ubi_img 1024 8192

else
	echo "Skipping NAND UBIFS images - set build_nand_ubifs=1 if they are needed"
	echo ""
fi

# jffs2 NOR images for 64k, 128k, 256k erase sizes
make_jffs2_img 64
make_jffs2_img 128
make_jffs2_img 256

echo "Writing NFS rootfs tarball..."
rm -f tmp/nfsroot.tar images/nfsroot-${TARGET}.tar.bz2
rm -rf tmp/romfs

mkdir -p tmp/romfs/boot
cp $LINUXDIR/.config tmp/romfs/boot/config
cp $LINUXDIR/Module.symvers tmp/romfs/boot/
cp $LINUXDIR/System.map tmp/romfs/boot/
cp $LINUXDIR/vmlinux tmp/romfs/boot/

cp misc/devconsole.tar tmp/nfsroot.tar
chmod u+w tmp/nfsroot.tar
tar --owner 0 --group 0 -rf tmp/nfsroot.tar romfs/
tar --owner 0 --group 0 -rf tmp/nfsroot.tar -C tmp romfs/boot/
bzip2 < tmp/nfsroot.tar > images/nfsroot-${TARGET}.tar.bz2
echo "  -> images/nfsroot-${TARGET}.tar.bz2"

exit 0
