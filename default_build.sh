#!/bin/bash

green='\033[01;32m'
red='\033[01;31m'
blink_red='\033[05;31m'
restore='\033[0m'

export ARCH=arm64
export BUILD_CROSS_COMPILE=android-toolchain-arm64/bin/arm-eabi-
export SYSROOT=android-toolchain-arm64/aarch64-SMG610-linux-gnu/sysroot/
export BUILD_JOB_NUMBER=`grep processor /proc/cpuinfo|wc -l`
export GIT_LOG1=`git log --oneline --decorate -n 1`

RDIR=$(readlink -f .)
OUTDIR=$RDIR/arch/$ARCH/boot
DTSDIR=$RDIR/arch/$ARCH/boot/dts
DTBDIR=$OUTDIR/dtb
DTCTOOL=$RDIR/scripts/dtc/dtc
INCDIR=$RDIR/include
WD=$RDIR/WORKING-DIR
RK=$RDIR/READY-KERNEL
PAGE_SIZE=2048
DTB_PADDING=0

FUNC_CLEAN_DTB()
{
	make ARCH=$ARCH mrproper;
	make clean;

# force regeneration of .dtb and zImage files for every compile
	rm -f arch/$ARCH/boot/*.dtb
	rm -f arch/$ARCH/boot/*.cmd

	if [ -d $WD/temp ]; then
		rm -rf $WD/temp/*
	else
		mkdir $WD/temp
	fi;

	if [ -d $DTBDIR ]; then
		rm -rf $DTBDIR/*
	else
		mkdir $DTBDIR
	fi;

### cleanup files creted previously
	for i in $(find "$RDIR"/ -name "boot.img"); do
		rm -fv "$i";
	done;
	for i in $(find "$RDIR"/ -name "Image"); do
		rm -fv "$i";
	done;
	for i in $(find "$RDIR"/ -name "dtb.img"); do
		rm -fv "$i";
	done;
	for i in $(find "$RDIR"/ -name "zImage"); do
		rm -fv "$i";
	done;

	git checkout android-toolchain-arm64/
}

FUNC_BUILD_DTIMAGE_TARGET()
{
	[ -f "$DTCTOOL" ] || {
		echo "You need to run ./build.sh first!"
		exit 1
	}

	case $MODEL in
	on7xelte)
		DTSFILES="exynos7870-on7xelte_ltn_open_00 exynos7870-on7xelte_ltn_open_01
				exynos7870-on7xelte_ltn_open_02 exynos7870-on7xelte_swa_open_00
				exynos7870-on7xelte_swa_open_01 exynos7870-on7xelte_swa_open_02"
		;;
	*)
		echo "Unknown device: $MODEL"
		exit 1
		;;
	esac

	mkdir -p $OUTDIR $DTBDIR

	cd $DTBDIR || {
		echo "Unable to cd to $DTBDIR!"
		exit 1
	}

	rm -f ./*

	echo "Processing dts files..."

	for dts in $DTSFILES; do
		echo "=> Processing: ${dts}.dts"
		${CROSS_COMPILE}cpp -nostdinc -undef -x assembler-with-cpp -I "$INCDIR" "$DTSDIR/${dts}.dts" > "${dts}.dts"
		echo "=> Generating: ${dts}.dtb"
		$DTCTOOL -p $DTB_PADDING -i "$DTSDIR" -O dtb -o "${dts}.dtb" "${dts}.dts"
	done

	echo "Generating dtb.img..."
	$RDIR/scripts/dtbTool/dtbTool -o "$OUTDIR/dtb.img" -d "$DTBDIR/" -s $PAGE_SIZE

	echo "Done."
}

FUNC_BUILD_KERNEL()
{
	echo -e "\nbuild variant config="$MODEL ""
	echo "build common config="$KERNEL_DEFCONFIG ""
	echo "git info="$GIT_LOG1 ""

	echo -e "\ncleaning..."
	FUNC_CLEAN_DTB | grep :

	echo "generating .config"
	make -j$BUILD_JOB_NUMBER ARCH=$ARCH \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE \
			$KERNEL_DEFCONFIG | grep :

	echo "compiling..."
	echo -e ""

	make -j$BUILD_JOB_NUMBER ARCH=$ARCH \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE \
			CC='ccache '${BUILD_CROSS_COMPILE}gcc' --sysroot='$SYSROOT'' | grep :

if [ -f $RDIR/arch/$ARCH/boot/Image ]; then
	FUNC_BUILD_DTIMAGE_TARGET
else
	echo -e "${red}"
	echo -e "Kernel STUCK in BUILD! no Image exist !"
	echo -e "${restore}"
	exit 1
fi;
}

FUNC_BUILD_RAMDISK()
{
	rm -f $WD/tools/split_img/boot.img-zImage
	rm -f $WD/tools/split_img/boot.img-dtb
	mv -f $RDIR/arch/$ARCH/boot/Image $WD/tools/split_img/boot.img-zImage
	mv -f $RDIR/arch/$ARCH/boot/dtb.img $WD/tools/split_img/boot.img-dtb

	if [ -d $WD/tools/ramdisk ]; then
		rm -rf $WD/tools/ramdisk/*
	else
		mkdir $WD/tools/ramdisk
	fi;

	\cp -r $WD/$RAMDISK/ramdisk $WD/tools
	\cp -r $WD/ramdisk/ramdisk $WD/tools
	cd $WD/tools
	cd ramdisk
		chmod 644 file_contexts
		chmod 644 se*
		chmod 644 *.rc
		chmod 750 init*
		chmod 640 fstab*
		chmod 644 default.prop
		chmod 771 data
		chmod 755 dev
		chmod 755 proc
		chmod 750 sbin
		chmod 750 sbin/*
		chmod 755 sys
		chmod 755 system
	cd ..
	./repackimg.sh
	echo SEANDROIDENFORCE >> image-new.img
	rm -rf $WD/tools/ramdisk/*
}

FUNC_BUILD_ZIP()
{
	if [ -d $WD/temp ]; then
		rm -rf $WD/temp/*
	else
		mkdir $WD/temp
	fi;

# to generate new file name if exist.(add a digit to new one)
	FILENAME=(Gabriel-$(date +"[%d-%m-%y]")-$MODEL);

	ZIPFILE=$FILENAME
	if [[ -e $RK/$ZIPFILE.zip ]] ; then
			i=0
		while [[ -e $RK/$ZIPFILE-$i.zip ]] ; do
			let i++
		done
	    FILENAME=$ZIPFILE-$i
	fi

	mv -f $WD/tools/image-new.img $WD/temp/boot.img
	mv -f $RDIR/build.log $WD/temp/build.log

	\cp -r $WD/package/* $WD/temp

	cd $WD/temp
	zip kernel.zip -r * > /dev/null
	cd $RDIR

	cp $WD/temp/kernel.zip $RK/$FILENAME.zip
	md5sum $RK/$FILENAME.zip > $RK/$FILENAME.zip.md5
}

echo -e "${green}"
echo "----------------------"
echo "Which kernel config ?!";
echo "----------------------"
echo -e "${restore}"
select CHOICE in g610-custom g610-stock; do
	case "$CHOICE" in
		"g610-custom")
			KERNEL_DEFCONFIG=on7xelte_defconfig
			MODEL=on7xelte
			RAMDISK=G610X
			break;;
		"g610-stock")
			KERNEL_DEFCONFIG=on7xelteswa_00_defconfig
			MODEL=on7xelte
			RAMDISK=G610X
			break;;
	esac;
done;

# MAIN FUNCTION
rm -rf ./build.log
(
	DATE_START=$(date +"%s")

	FUNC_BUILD_KERNEL
	FUNC_BUILD_RAMDISK
	FUNC_BUILD_ZIP

	DATE_END=$(date +"%s")

	echo -e "${green}"
	echo "File Name is: "$FILENAME
	echo -e "\n-------------------"
	echo "Build Completed in:"
	echo "-------------------"
	echo -e "${restore}"

	DATE_END=$(date +"%s")
	DIFF=$(($DATE_END - $DATE_START))
	echo "Time: $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
) 2>&1	| tee -a ./build.log
