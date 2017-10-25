# h3d-testing
Hacking h3d for testing purposes
It consist of the following:

First we clone the official H3D repo, with a known (tested) revision:
svn export https://h3dapi.org:8090/H3DAPI/metarepos/H3DWithToolkitsWin/trunk/H3D H3DWithToolkitsWinTrunk -r 3571

Libnifalcon (actually just used to spoof H3D Cmake build system)
https://github.com/forsslund/h3d-testing.git rev c1546ec8d9a94b3be7d74f8ba9f1a121a0ebb0c2 (Jul 4, 2016)

Then we forcefully shoehorn in some preliminary woodenhaptics code, under the diguise as "NiFalconDevice".

<pre>
To install do following (Ubuntu 16.04 64 bit)
1. Install dependencies: 
     sh ubuntudep-16.04.sh
2. Build and install remotehaptics
     cd libremotehaptics
     qmake
     make
     sh makeinstall.sh
3. Build and install libnifalcon
     cd libnifalcon/build
     cmake ..
     make -j5
     sudo make install
4. Build H3D (with our hacks). 
   Note: to build debug mode run cmake with flag: -DCMAKE_BUILD_TYPE=Debug
     cd H3DWithToolkitsWinTrunk/build
     cmake .
     make -j5
     sudo make install

5. Now you should be able to run some tests:
     cd examples
     H3DLoad SuperShape.x3d
</p>