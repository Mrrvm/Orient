## Install
There are quite a few things to be installed,

  1) [IDS uEye camera software](#ids-ueye-camera-software)
  2) [LPMS-CU software](#lpms-cu-software)
  3) [OpenCV 4.1.1](#opencv-411)
  4) [Ceres Solver](#ceres-solver)
  5) Optional: [CLion](clion) 
  
NOTE: This was only ran under Ubuntu 18.04 LTS, other distributions or operative systems 
might require other packages or different installation manners.
  
  <hr>
  
  ### IDS uEye camera software 
  
  1) Make an account and download the IDS software at 
  their [website](https://en.ids-imaging.com/download-ueye-lin64.html).
  
  2) Install the following dependencies, `libqt5gui5`, `libqt5network5`, `libjpeg62`, `libpng16-16`.
  
  3) Extract the contents of the file and run `sudo sh ./ueyesdk-setup*.run`.
  
  4) Start the uEye usb deamon `sudo systemctl start ueyeusbdrc`.
  
  5) Connect the camera on a USB 3.0 port, run `ueyedemo` and test if everything is alright.

  For troubleshoot, there is a readme on the extracted contents. 
  You can also consult [uEye manual](https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual_4.92.2/index.html).
  
  ### LPMS-CU software
  
  You must download [LpSensor-1.3.4-Linux-x86-64](https://bitbucket.org/lpresearch/openmat/downloads/), because
  version 1.3.5 will not work due to a timeout issue.
  
  Extract the contents of the file, there is a readme explaining the installation process in more detail. Succintly,
  
  1) Install libbluetooth.so `sudo apt install libbluetooth-dev`.
  2) Install libftd2xx.so. To do this, you must:
      - download the [FTDI drivers](https://www.ftdichip.com/Drivers/D2XX.htm);
      - extract the contents `gunzip libftd2xx1.1.12.tar.gz` `tar –xvf libftd2xx1.1.12.tar`;
      - copy the files to `/usr/local/lib`by `sudo cp  /releases/build/lib*  /usr/local/lib`
      - in the `/usr/local/lib` directory, `cd /usr/local/lib`, create a symbolic link `sudo ln –s libftd2xx.so.?.?.? libftd2xx.so` replacing `?.?.?` by your version of the drivers; 
      - give it the following permissions `sudo chmod 0755 libftd2xx.so.?.?.?`.
      
      For troubleshoot, consult the [FTDI installation manual](https://www.ftdichip.com/Support/Documents/AppNotes/AN_220_FTDI_Drivers_Installation_Guide_for_Linux.pdf).
  3) Install the deb package by `sudo dpkg -i liblpsensor-1.3.4-Linux.deb` and `dpkg -L liblpsensor`.
  
  Now to test:
  1) Plug in the LPMS-CU device.
  2) Unload the following modules `sudo rmmod ftdi_sio` and `sudo rmmod usbserial`
  3) `cd` into the `sample` directory on the extracted files;
  4) Change the line `LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B, "00:11:22:33:44:55");` to your correct device ID and device type.
  The type is `DEVICE_LPMS_U` and the ID should be something alike `A5014194`. You can find this ID through 3 different ways:
      - Using the Windows [OpenMAT-1.3.5-Setup-Build20180418.exe](https://bitbucket.org/lpresearch/openmat/downloads/).
      - Through the [SensorDiscovery program](https://www.ftdichip.com/Support/Documents/AppNotes/AN_220_FTDI_Drivers_Installation_Guide_for_Linux.pdf).
      - Or by running FTDI `read` sample. (Section 3.1 of the [FTDI manual](https://www.ftdichip.com/Support/Documents/AppNotes/AN_220_FTDI_Drivers_Installation_Guide_for_Linux.pdf)).
  4) and build the example 
```
    mkdir build
    cd build
    cmake ..
    make
    sudo ./LpmsSimpleExample
```
  5) Check if the sensor connected and is producing results (this might take a bit).
 
 If it's not working, you might be missing dependencies `libqwt-dev`, `libqt5-dev`, `libftdi-dev`, `libice-dev` or `libboost-all-dev`.
   
  ### OpenCV 4.1.1
  
  1) Create an installation directory, e.g. `installed/opencv4/`.
  2) Update and upgrade your system `sudo apt -y update` `sudo apt -y upgrade`.
  3) Install the following dependencies
  
```
    sudo apt -y remove x264 libx264-dev
    sudo apt -y install build-essential checkinstall cmake pkg-config yasm
    sudo apt -y install git gfortran
    sudo apt -y install libjpeg8-dev libpng-dev

    sudo apt -y install software-properties-common
    sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    sudo apt -y update

    sudo apt -y install libjasper1
    sudo apt -y install libtiff-dev

    sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
    sudo apt -y install libxine2-dev libv4l-dev
    cd /usr/include/linux
    sudo ln -s -f ../libv4l1-videodev.h videodev.h

    sudo apt -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    sudo apt -y install libgtk2.0-dev libtbb-dev qt5-default
    sudo apt -y install libatlas-base-dev
    sudo apt -y install libfaac-dev libmp3lame-dev libtheora-dev
    sudo apt -y install libvorbis-dev libxvidcore-dev
    sudo apt -y install libopencore-amrnb-dev libopencore-amrwb-dev
    sudo apt -y install libavresample-dev
    sudo apt -y install x264 v4l-utils

    # Optional dependencies
    sudo apt -y install libprotobuf-dev protobuf-compiler
    sudo apt -y install libgoogle-glog-dev libgflags-dev
    sudo apt -y install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```
  
3) Download opencv and opencv_contrib
```
    git clone https://github.com/opencv/opencv.git
    git clone https://github.com/opencv/opencv_contrib.git
```

4) Build opencv. Create and `cd` into a `build` directory on the downloaded opencv folder. Run
```
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/path/to/installed/opencv4 \
    -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_OPENCL=ON \
    -D WITH_TBB=OFF -D WITH_OPENMP=ON -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D BUILD_EXAMPLES=ON ..
```

5) Go drink 30 minutes worth of coffees and come back.

6) Run `make` (if you want, with flag `-j<nb of cores>` to speed up) and then `make install`. All done.


### Ceres Solver

1) Download ceres `git clone https://ceres-solver.googlesource.com/ceres-solver`.

2) Install dependencies.
```
      # CMake
      sudo apt-get install cmake
      # google-glog + gflags
      sudo apt-get install libgoogle-glog-dev
      # BLAS & LAPACK
      sudo apt-get install libatlas-base-dev
      # Eigen3
      sudo apt-get install libeigen3-dev
      # SuiteSparse and CXSparse (optional)
      # - If you want to build Ceres as a *static* library (the default)
      #   you can use the SuiteSparse package in the main Ubuntu package
      #   repository:
      sudo apt-get install libsuitesparse-dev
      # - However, if you want to build Ceres as a *shared* library, you must
      #   add the following PPA:
      sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
      sudo apt-get update
      sudo apt-get install libsuitesparse-dev
```

3) Build and install by,
```
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-?.?.? 
make -j3
make test
make install
```
where `?.?.?` is your ceres version.

### CLion

1) Download [CLion](https://www.jetbrains.com/clion/) and unpack.

2) On the downloaded directory `cd` into bin and run `./clion.sh`.

3) Add the installation directory to your `PATH` environment variable.

4) Run `clion`. All good!

### Links

IDS uEye camera software: https://en.ids-imaging.com/download-ueye-lin64.html

Manual for IDS uEye camera: https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual_4.92.2/index.html

LPMS-CU software download: https://bitbucket.org/lpresearch/openmat/downloads/

FTDI drivers download: https://www.ftdichip.com/Drivers/D2XX.htm

FTDI installation manual: https://www.ftdichip.com/Support/Documents/AppNotes/AN_220_FTDI_Drivers_Installation_Guide_for_Linux.pdf

LPMS programming examples: https://bitbucket.org/lpresearch/lpmsexamples/src/master/

OpenCV: git clone https://github.com/opencv/

Ceres Solver: http://ceres-solver.org/

CLion: https://www.jetbrains.com/clion/
  
