#!/bin/bash

BCM_LIB=bcm2835-1.71

if [[ ! -d "$BCM_LIB" ]]
then
  curl http://www.airspayce.com/mikem/bcm2835/$BCM_LIB.tar.gz > $BCM_LIB.tar.gz
  tar zxvf $BCM_LIB.tar.gz
fi

cd $BCM_LIB

./configure
make
sudo make check
sudo make install

cd src
cc -shared bcm2835.o -o libbcm2835.so
mv libbcm2835.so ../../libbcm2835.so
