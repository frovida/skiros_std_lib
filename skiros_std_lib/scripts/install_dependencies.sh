#!/bin/bash

tfd='tfd-src-0.4'

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Installing planner..."
echo "Type install folder [default: $DIR]:"

read folder

if ! [ -z $folder ];
then
	DIR=$folder
fi

cd $DIR
wget "http://gki.informatik.uni-freiburg.de/tools/tfd/downloads/version-0.4/${tfd}.tgz"
tar xzf "${tfd}.tgz"
cd "${tfd}" && ./build
sed -e s/"translate\/"//g -i ./downward/plan.py
sed -e s/"preprocess\/"//g -i ./downward/plan.py
sed -e s/"search\/"//g -i ./downward/plan.py
cd -
rm -r "${tfd}.tgz"
string="export TFD_HOME=$(pwd)"
string2="export PATH=$""TFD_HOME/${tfd}/downward:$""TFD_HOME/${tfd}/downward/translate:$""TFD_HOME/${tfd}/downward/preprocess:$""TFD_HOME/${tfd}/downward/search:""$""PATH"
temp=$(cat ~/.bashrc | grep "$string")
if [ -z "$temp" ]; then
	echo $string >> ~/.bashrc
	echo $string2 >> ~/.bashrc
	echo "Printing string in .bashrc: $string"
else
	echo "No export string added to bashrc (it is already there)"
fi

source ~/.bashrc
