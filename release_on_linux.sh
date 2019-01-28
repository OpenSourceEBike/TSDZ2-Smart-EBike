#!/bin/sh

echo This script will release a new version of the firmware
echo
echo Please enter the new version number, e.g. 1.0.0

read VERSION

RELEASE_FOLDER=${PWD}/releases/$VERSION

if [ ! -d "$RELEASE_FOLDER" ]; then
	
  # create folder
	mkdir -p $RELEASE_FOLDER

	cd src/controller
      make -f Makefile_linux
	cp main.hex $RELEASE_FOLDER/TSDZ2-v$VERSION.hex
	cd ../..

	cd src/display/KT-LCD3
    	make -f Makefile_linux
	cp main.hex $RELEASE_FOLDER/KT-LCD3-v$VERSION.hex

	echo Done! Find the files on: $RELEASE_FOLDER
else
	echo Release $VERSION already exists!
fi

