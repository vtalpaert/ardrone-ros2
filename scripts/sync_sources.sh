#! /bin/bash

# Solving dependencies
sudo apt update
sudo apt install -y repo autoconf pkg-config libavahi-client-dev libavcodec-dev libavformat-dev libjson-c-dev libncurses-dev libswscale-dev 

# Note that the build fails for python3.10 and requires to downgrade to python3.7. To avoid messing with our system, we use a virtual environment.
# Several solutions are possible: in a docker image, using a third party apt repository for python3.7 or rebuilding using pyenv.
# Here is my solution using the third party ppa, which you should only use at your own risk.
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3-virtualenv python3.7
virtualenv venv -p python3.7
source venv/bin/activate

# Clone SDK and build it
mkdir -p arsdk/
cd arsdk/
repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git
repo sync
./build.sh -p native -t build-sdk -j 3

# Clean up
deactivate
cd ../
rm -rf src/ardrone_sdk/include
rm -rf src/ardrone_sdk/Includes
rm -rf src/ardrone_sdk/Sources
rm -rf src/ardrone_sdk/JumpingSumoSample
rm -rf src/ardrone_sdk/BebopSample
mkdir src/ardrone_sdk/Sources
mkdir src/ardrone_sdk/Includes
mkdir src/ardrone_sdk/JumpingSumoSample
mkdir src/ardrone_sdk/BebopSample

# Copy sources
## Copy public headers
cp -r arsdk/out/arsdk-native/staging/usr/include src/ardrone_sdk

## Copy private sources
function copy_sdk_sources() {
    local library_name=$1
    local packages_files_dir="arsdk/packages/$library_name"
    rsync -a ${packages_files_dir}/Includes/ src/ardrone_sdk/Includes/
    if [ -d ${packages_files_dir}/Sources ]; then
        rsync -a ${packages_files_dir}/Sources/ src/ardrone_sdk/Sources/
    fi
    if [ -d ${packages_files_dir}/src ]; then
        rsync -a ${packages_files_dir}/src/ src/ardrone_sdk/Sources/
    fi
    if [ -d ${packages_files_dir}/gen/Sources ]; then
        rsync -a ${packages_files_dir}/gen/Sources/ src/ardrone_sdk/Sources/
    fi
    local generated_files_dir="arsdk/out/arsdk-native/build/$library_name/gen"
    if [ -d $generated_files_dir/Includes ]; then
        #cp -r $generated_files_dir/Includes/* src/ardrone_sdk/Includes/
        rsync -a $generated_files_dir/Includes/ src/ardrone_sdk/Includes/
    fi
    if [ -d $generated_files_dir/Sources ]; then
        rsync -a $generated_files_dir/Sources/ src/ardrone_sdk/Sources/
    fi
}
copy_sdk_sources libARCommands
copy_sdk_sources libARSAL
cp arsdk/packages/libARSAL/Config/linux/config.h src/ardrone_sdk/Sources/config.h
copy_sdk_sources libARController
copy_sdk_sources libARDiscovery
rm src/ardrone_sdk/Includes/libARDiscovery/ARDISCOVERY_BonjourDiscovery.h
copy_sdk_sources libARStream
copy_sdk_sources libARStream2
copy_sdk_sources libARNetwork
copy_sdk_sources libARNetworkAL
rm src/ardrone_sdk/Sources/BLE/ARNETWORKAL_BLENetwork.m


function copy_utils_sources() {
    local library_name=$1
    rsync -a arsdk/packages/$library_name/include/ src/ardrone_sdk/Includes/
    rsync -a arsdk/packages/$library_name/src/ src/ardrone_sdk/Sources/
}
copy_utils_sources librtsp
copy_utils_sources libpomp
copy_utils_sources libsdp
copy_utils_sources libfutils

function copy_ulog_sources() {
    local sub_path=$1
    rsync -a --ignore-missing-args arsdk/packages/ulog/$sub_path/*.h src/ardrone_sdk/Sources/
    rsync -a arsdk/packages/ulog/$sub_path/*.c src/ardrone_sdk/Sources/
    if [ -d arsdk/packages/ulog/$sub_path/include ]; then
        rsync -a arsdk/packages/ulog/$sub_path/include/ src/ardrone_sdk/Includes/
    fi
}
#copy_ulog_sources kernel
#copy_ulog_sources kmsgd
copy_ulog_sources libulog

# Copy Samples
cp arsdk/packages/Samples/Unix/JumpingSumoSample/*.{h,c} src/ardrone_sdk/JumpingSumoSample/
cp arsdk/packages/Samples/Unix/BebopSample/*.{h,c} src/ardrone_sdk/BebopSample/

# Final clean up
rm src/ardrone_sdk/Sources/*.m
rm src/ardrone_sdk/Sources/*.am
