#!/bin/sh

echo This script will release a new version of the firmware
echo
echo Please enter the new version number, e.g. 1.0.0

read VERSION
echo

RELEASE_FOLDER=${PWD}/releases/$VERSION

if [ -d "$RELEASE_FOLDER" ]; then
	echo Removing existing release folder: $RELEASE_FOLDER
	echo
	rm -R $RELEASE_FOLDER
fi

mkdir -p $RELEASE_FOLDER

cd src
make -f Makefile_linux clean
echo
make -f Makefile_linux
cp main.hex $RELEASE_FOLDER/TSDZ2-v$VERSION.hex
cd ..

echo
echo
echo Done! Find the file on:
echo
ls -la $RELEASE_FOLDER/TSDZ2-v$VERSION.hex
echo
echo

