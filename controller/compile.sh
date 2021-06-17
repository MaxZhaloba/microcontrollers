#!/bin/bash -x

temp=~/temp
build=$(pwd)/build

rm -rf $build
rm -rf $temp
cp -R . $temp
cd $temp
make
cd -
mkdir $build
cp $temp/build/*.elf $build/