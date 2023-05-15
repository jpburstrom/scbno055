# SCBNO055

Author: Johannes Burström

A BNO055 sensor plugin for Bela and SuperCollider

### Requirements

- CMake >= 3.7
- SuperCollider source code

### Installing

If you don't want to build the extension yourself, there is a compiled binary in the Extensions directory. Install it from the host computer (with the Bela attached) with the `install-bela.sh` script in the repository root.

### Building

Here are instructions for building on the Bela. Another option is to cross-compile, for example with the [xcBela](https://github.com/TheTechnobear/xcBela) environment. See its [CMake faq](https://github.com/TheTechnobear/xcBela/blob/master/cmake/cmakefaq.md) for instructions on how to do that.

#### Building on Bela

Clone the project. If your Bela has internet connection:

    cd /root
    mkdir src && cd src
    # Clone a shallow copy of the SuperCollider repo
    git clone --depth 1 https://github.com/supercollider/supercollider
    # Clone the scbno055 repo
    git clone https://github.com/jpburstrom/scbno055
    
If you haven't set up an internet connection on your Bela, you can, with the Bela attached, clone the repo on the host computer and copy the whole directory over to your Bela, eg using rsync:

    # Clone a shallow copy of the SuperCollider repo
    git clone --depth 1 https://github.com/supercollider/supercollider
    # Clone the scbno055 repo
    git clone https://github.com/jpburstrom/scbno055
    rsync -avh supercollider scbno055 root@bela:/root/src/
    
Ssh to your Bela, then:
    
    cd /root/src/scbno055
    mkdir build
    cd build

Then, use CMake to configure, build and install it:

    cmake .. -DBELA=On -DCMAKE_INSTALL_PREFIX=/root/.local/share/SuperCollider/Extensions -DCMAKE_BUILD_TYPE=Release -DSUPERNOVA=Off
    cmake --build . --config Release --target install

It's expected that the SuperCollider repo is cloned at `../supercollider` relative to this repo. If it's not: add the option `-DSC_PATH=/path/to/sc/source`.
