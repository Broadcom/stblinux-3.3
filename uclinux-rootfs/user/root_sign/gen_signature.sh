#!/bin/sh
set -e
function generate_hash()
{
	if [ ! -f $2 ]; then
		echo "generate_hash; file $2 does not exists"
		exit 1
	elif [ ! -r $2 ]; then
		echo "generate_hash: file $2 can not read"
		exit 2
	fi
	echo "Genrating hash for file:" $2
	SIGNATURE=$($HASHTOOL $2 | awk -F' ' '{print $1}')
	echo $1":"$SIGNATURE
	echo $1":"$SIGNATURE >> sig.tmp
	
}
#$1 = relative parent dir,e.g. /lib/
#$2 = absolute parent dir.e.g. /project/stbtc/rootfs/lib/
function expand_dir()
{
	echo "Expanding directory" $2
	
	for filename in `ls -1 $2`; do
		if [ "$1" == "$2" ]; then
			prefix=$1
		else
			prefix=$2
		fi
		if [ -d $prefix$filename ]; then
			expand_dir $1$filename $prefix$filename
		elif [ ! -L $prefix$filename ]; then
			if ! generate_hash $1$filename  $prefix$filename; then
				exit 1;
			fi
		fi
	done
}
if [ $# -eq 0 ]
then 
	echo "Usage: gen_signature.sh signature_list [rootfs_dir]"
	echo " "
	echo "	signature_list: List of files that need to be verified"
	echo "	rootfs_dir: root file system directory, if omited, use '/'"
	exit -1
fi
INPUTFILE=$1
ROOTFSDIR=$2
if [ -a sig.tmp ]; then
	echo "Removing tmp file"
	rm -f sig.tmp
fi
if [ ! -f $INPUTFILE ]; then
	echo "$INPUTFILE does not exists"
	exit 1
elif [ ! -r $INPUTFILE ]; then
	echo "$INPUTFILE can not read"
	exit 2
fi

HASHTOOL="sha1sum"
BAKIFS=$IFS
IFS=$(echo -en "\n\b")
exec 3<&0
exec 0<"$INPUTFILE"

while read -r line
do
	FILENAME=$(echo $line | awk -F':' '{print $1}')
	if [ -n "$ROOTFSDIR" ]; then
		ROOTFSDIR=$(echo $ROOTFSDIR | sed -e 's,\(.\)/$,\1,')
		FILE="$ROOTFSDIR$FILENAME"
	else
		FILE=$FILENAME
	fi
	if [ -d $FILE ]; then
		if ! expand_dir $FILENAME $FILE; then
			exit 1
		fi
	else
		if ! generate_hash $FILENAME $FILE; then
			exit 1;
		fi
	fi
done
echo "updating signature file"
rm -f $INPUTFILE
mv sig.tmp $INPUTFILE

exec 0<&3
