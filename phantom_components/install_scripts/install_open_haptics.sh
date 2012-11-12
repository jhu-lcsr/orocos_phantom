# /bin/bash
rosdep install phantom_components
#wget http://dsc.sensable.com/3dtouch/OpenHaptics_Academic_linux/downloadFile.asp?file=3dtouch/OpenHapticsAE_Linux_v3_0.zip
unzip OpenHapticsAE_Linux_v3_0.zip
sudo apt-get install libmotif-dev libraw1394-dev csh libglw1-mesa-dev
sudo dpkg -i "OpenHapticsAE_Linux_v3_0/PHANTOM Device Drivers/`dpkg-architecture -qDEB_HOST_ARCH_BITS`-bit/phantomdevicedrivers_4.3-3_`dpkg-architecture -qDEB_HOST_ARCH_CPU`.deb"
sudo dpkg -i "OpenHapticsAE_Linux_v3_0/OpenHaptics-AE 3.0/`dpkg-architecture -qDEB_HOST_ARCH_BITS`-bit/openhaptics-ae_3.0-2_`dpkg-architecture -qDEB_HOST_ARCH_CPU`.deb"

#cd /usr/lib && sudo ln -s libraw1394.so libraw1394.so.8
cd /usr/lib/x86_64-linux-gnu && sudo ln -s libraw1394.so libraw1394.so.8 && cd -
#cd /usr/lib && sudo ln -s libXm.so.3 libXm.so.4 # this is not needed on 12.04
./set_permissions_1394.sh
/usr/sbin/PHANToMConfiguration
