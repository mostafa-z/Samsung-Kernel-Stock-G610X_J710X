#! /bin/bash
clear

LANG=C

#------------------------------------
# define variables
#------------------------------------
TODAY=`date '+%Y%m%d'`;
GIT_BRANCH=`git symbolic-ref --short HEAD`;
GITCCOUNT=$(git shortlog | grep -E '^[ ]+\w+' | wc -l);
COLOR_RED="\033[0;31m"
COLOR_GREEN="\033[1;32m"
COLOR_NEUTRAL="\033[0m"
#LOG=(WORKING-DIR/package/compile.log);
LOG=(WORKING-DIR/temp/compile.log);
TC=(android-toolchain-arm64/bin/arm-eabi-);
SYSROOT=(android-toolchain/aarch64-SMG610-linux-gnu/sysroot/);
KD=$(readlink -f .);
WD=(WORKING-DIR);
RK=(READY-KERNEL);
TOOLS=$WD/tools;
BOOT=(arch/arm64/boot);
DMODEL=(on7xelte);
STOCK_DEF=(on7xelteswa_00_defconfig);
#KERNEL_CONFIG_FILE=(gabriel_g610f_defconfig);
DCONF=(arch/arm64/configs);
#STOCK_DEF=(g3-global_com-perf_defconfig);
NAME=Gabriel-$(grep "CONFIG_LOCALVERSION=" arch/arm64/configs/gabriel_g610f_defconfig | cut -c 25-28);

#export PATH=$PATH:tools/lz4demo
#===============================================================================
# Define Functions
#===============================================================================

# to generate new file name if exist.(add a digit to new one)
FILENAME()
{
	ZIPFILE=$FILENAME
	if [[ -e $RK/$ZIPFILE.zip ]] ; then
    		i=0
    	while [[ -e $RK/$ZIPFILE-$i.zip ]] ; do
        	let i++
    	done
    FILENAME=$ZIPFILE-$i
	fi
}

# determine how many core your CPU have to build
NR_CPUS()
{
	# Idea by savoca
	NR_CPUS=$(grep -c ^processor /proc/cpuinfo)

	if [ "$NR_CPUS" -le "2" ]; then
		NR_CPUS=4;
		echo "Building kernel with 4 CPU threads";
	else
		echo -e "\e[1;44mBuilding kernel with $NR_CPUS CPU threads\e[m"
	fi;
}

# check if kernel zip file had build or not
FILE_CHECK()
{
	echo -e "\n---------------------------------------------------"
	echo -e "Check for coocked file:"
	if [ -f $RK/$FILENAME.zip ]; then
		echo -e $COLOR_GREEN
		echo "File name is: "$FILENAME".zip"
		echo -e $COLOR_NEUTRAL
	else
		echo -e $COLOR_RED
		echo "oops, &*%^&(%#!@#*(& !!"
		echo -e $COLOR_NEUTRAL		
	fi;
}

# check for errors
LOG_CHECK()
{
	echo -e "---------------------------------------------------"
	echo -e "Check for compile errors:"
	echo -e $COLOR_RED

#	grep error $WD/package/compile.log
#	grep forbidden $WD/package/compile.log
#	grep warning $WD/package/compile.log
#	grep fail $WD/package/compile.log
#	grep no $WD/package/compile.log
	grep error $WD/temp/compile.log
	grep forbidden $WD/temp/compile.log
	grep warning $WD/temp/compile.log
	grep fail $WD/temp/compile.log
	grep no $WD/temp/compile.log
	echo -e $COLOR_NEUTRAL

	echo -e "---------------------------------------------------"
}

# refresh for new build
CLEANUP()
{
	make ARCH=arm64 mrproper;
	make clean;

# force regeneration of .dtb and zImage files for every compile
	rm -f arch/arm64/boot/*.dtb
	rm -f arch/arm64/boot/*.cmd
	rm -f arch/arm64/boot/Image
	rm -f arch/arm64/boot/Image.gz

	if [ -d $WD/temp ]; then
		rm -rf $WD/temp/*
	else
		mkdir $WD/temp
	fi;

### cleanup files creted previously

	for i in $(find "$KD"/ -name "*.ko"); do
		rm -fv "$i";
	done;
	for i in $(find "$KD"/ -name "boot.img"); do
		rm -fv "$i";
	done;
	for i in $(find "$KD"/ -name "dt.img"); do
		rm -fv "$i";
	done;
	for i in $(find "$KD"/ -name "*.zip" -not -path "*$RK/*"); do
		rm -fv "$i";
	done;
	for i in $(find "$KD"/ -name "Image"); do
		rm -fv "$i";
	done;
	for i in $(find "$KD"/ -name "kernel_config_view_only"); do
		rm -fv "$i";
	done;
	for i in $(find "$KD"/ -name "compile.log"); do
		rm -fv "$i";
	done;

	git checkout android-toolchain-arm64/
}

#===============================================================================
# Build Process
#===============================================================================

REBUILD()
{
clear
FILENAME=($NAME-$(date +"[%d-%m-%y]")-$MODEL);
FILENAME;
NR_CPUS;

	echo -e "\e[41mREBUILD\e[m"
	echo ""
	echo -e $COLOR_GREEN"Git Branch is at: "$GIT_BRANCH" commits: "$GITCCOUNT" " $COLOR_NEUTRAL
	echo ""
	sleep 3

#	touch $WD/package/compile.log
	touch $WD/temp/compile.log
	echo -e "\n---------------------------------------------------" > $LOG
	echo -e "\nGIT branch and last commit : " >> $LOG
	git log --oneline --decorate -n 1 >> $LOG
	echo -e ""
	echo "CPU : compile with "$NR_CPUS"-way multitask processing" >> $LOG
	echo "Toolchain: "$TC >> $LOG

	TIMESTAMP1=$(date +%s)
	make ARCH=arm64 CROSS_COMPILE=$TC $CUSTOM_DEF
	echo -e $COLOR_GREEN"\nI'm coocking, make a coffee ..." $COLOR_NEUTRAL
	echo ""
	make ARCH=arm64 CROSS_COMPILE=$TC CC='ccache '${TC}gcc' --sysroot='$SYSROOT'' -j $NR_CPUS | grep :
	clear

POST_BUILD >> $LOG
FILE_CHECK;
LOG_CHECK;
}

REBUILD_NCONF()
{
clear
FILENAME=($NAME-$(date +"[%y-%m-%d]")-$MODEL);
FILENAME;
NR_CPUS;

	echo -e "\e[41mREBUILD\e[m"
	echo -e ""
	echo -e $COLOR_GREEN"Git Branch is at : "$GIT_BRANCH $COLOR_RED">>> "$COLOR_GREEN$GITCCOUNT" commits" $COLOR_NEUTRAL
	echo -e ""
	sleep 3

	echo -e "\n---------------------------------------------------" > $LOG

	echo -e "\nGIT branch and last commit : " >> $LOG
	git log --oneline --decorate -n 1 >> $LOG
	echo -e ""
	echo -e "CPU : compile with "$NR_CPUS"-way multitask processing" >> $LOG
	echo -e "Toolchain: "$TC >> $LOG

	TIMESTAMP1=$(date +%s)
	make ARCH=arm64 CROSS_COMPILE=$TC $CUSTOM_DEF
	make ARCH=arm64 CROSS_COMPILE=$TC nconfig
	echo -e $COLOR_GREEN"\nI'm coocking, make a coffee ..." $COLOR_NEUTRAL
	echo ""
	make ARCH=arm64 CROSS_COMPILE=$TC CC='ccache '${TC}gcc' --sysroot='$SYSROOT'' -j $NR_CPUS | grep :
	clear

POST_BUILD >> $LOG
FILE_CHECK;
LOG_CHECK;
}

CONTINUE_BUILD()
{
	clear
	echo -e "\e[41mCONTINUE_BUILD\e[m"
	sleep 3
	time make ARCH=arm64 CROSS_COMPILE=$TC -j ${CPUNUM}
	clear
}

POST_BUILD()
{
	echo -e "\nbuild for :" $MODEL
	echo -e "\nchecking for compiled kernel..."
	echo ""
if [ -f arch/arm64/boot/Image ]
	then

case $DMODEL in
on7xelte)
	DTSFILES="exynos7870-on7xelte_ltn_open_00 exynos7870-on7xelte_ltn_open_01
		exynos7870-on7xelte_ltn_open_02 exynos7870-on7xelte_swa_open_00
		exynos7870-on7xelte_swa_open_01 exynos7870-on7xelte_swa_open_02"
	;;
*)
	echo "Unknown device: $DMODEL"
	exit 1
	;;
esac

echo "Processing dts files."
for dts in $DTSFILES; do
	echo "=> Processing: ${dts}.dts"
	${CROSS_COMPILE}cpp -nostdinc -undef -x assembler-with-cpp -I "$KD/include" "$BOOT/dts/${dts}.dts" > "${dts}.dts"
	echo "=> Generating: ${dts}.dtb"
	$KD/scripts/dtc/dtc -p 0 -i "$BOOT/dts" -O dtb -o "${dts}.dtb" "${dts}.dts"
done

echo "Generating dtb.img."
$TOOLS/dtbTool -o "$BOOT/dt.img" -d "$BOOT/dtb/" -s 2048
echo "Done."

#	echo "generating device tree..."
#	echo ""
#	$TOOLS/dtbTool -o $BOOT/dt.img -s 2048 -p $DTC/ $BOOT/ >> $LOG

#	if [ -f $BOOT/dt.img ]; then
#		echo -e "\nDevice Tree : Builded" >> $LOG
#	else
#		echo -e "\nDevice Tree : Failed !" >> $LOG
#	fi;

	# copy all selected ramdisk files to temp folder
	\cp -r $WD/anykernel/* $WD/temp
#	\cp -r $WD/$RAMDISK/* $WD/temp
	\cp -r $WD/ramdisk/* $WD/temp

	echo "copy zImage and dt.img"
	echo ""
	\cp -v $BOOT/Image $WD/temp/zImage >> $LOG
	\cp -v $BOOT/dt.img $WD/temp/dtb >> $LOG

#	echo "creating boot.img"
#	echo ""
#	$TOOLS/mkboot $WD/temp $WD/package/boot.img >> $LOG

	echo "copy .config"
#	\cp -v .config $WD/package/kernel_config_view_only >> $LOG
	\cp -v .config $WD/temp/kernel_config_view_only >> $LOG

	echo "Create flashable zip"
#	cd $WD/package
	cd $WD/temp
	zip kernel.zip -r *

	echo "copy flashable zip to output > flashable"
	cd ..
	cd ..
#	cp -v $WD/package/kernel.zip $RK/$FILENAME.zip
	cp -v $WD/temp/kernel.zip $RK/$FILENAME.zip
	md5sum $RK/$FILENAME.zip > $RK/$FILENAME.zip.md5
		
	#This part is for me on Workin Dir
	echo -e "\nFlashable zip is ready" >> $LOG

	TIMESTAMP2=$(date +%s) 
	TIME=$(($TIMESTAMP2 - $TIMESTAMP1))
	echo -e "\nCompile time: "$TIME "seconds" >> $LOG

else
	echo "Kernel STUCK in BUILD! no zImage exist !"

### THANKS GOD

fi;
}

echo "What to do What not to do ?!";
select CHOICE in G610F CONTINUE_BUILD G610F_STOCK_DEF G610F_NCONF; do
	case "$CHOICE" in
		"G610F")
			CLEANUP;
			CUSTOM_DEF=gabriel_g610f_defconfig
			MODEL=G610F
			RAMDISK=G610F
			REBUILD;
			break;;
		"CONTINUE_BUILD")
			CONTINUE_BUILD;
			break;;
		"G610F_STOCK_DEF")
			CUSTOM_DEF=$STOCK_DEF 
			RAMDISK=G610F
			REBUILD;
			break;;
		"G610F_NCONF")
			CUSTOM_DEF=$STOCK_DEF
			RAMDISK=G610F
			REBUILD_NCONF;
			break;;
	esac;
done;
