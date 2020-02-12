#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

set -e
set -x

# clean temporary unreal folders
rm -rf Binaries
rm -rf Build
rm -rf Intermediate
rm -rf Saved
rm -rf LinuxNoEditor
rm -rf DerivedDataCache
rm -rf Plugins/AirSim/Binaries
rm -rf Plugins/AirSim/Intermediate
rm -rf Plugins/AirSim/Saved
rm -f CMakeLists.txt
rm -f Makefile
rm -f *.pri
rm -f *.txt
rm -f *.kdev4
rm -f *.pro
rm -f *.workspace
popd >/dev/null
