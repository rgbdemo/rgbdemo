#!/bin/sh
##

if ! test -f /usr/local/include/libusb-1.0/libusb.h; then
    echo "Libusb package does not seem to be installed."
    read -p "Install it? y/n " INSTALL_LIBUSB
    if test $INSTALL_LIBUSB = "y"; then
	echo "Installing it..."
	echo sudo installer -pkg nestk/deps/macosx/libusb_1.0.pkg -target /
	sudo installer -pkg nestk/deps/macosx/libusb_1.0.pkg -target /
    fi
fi

mkdir -p build || exit 1
cd build || exit 1

cmake .. \
    -Wno-dev \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_VERBOSE_MAKEFILE=1 \
    $*

echo "Program configured in directory build."
echo "Now go into build/ and run make."
