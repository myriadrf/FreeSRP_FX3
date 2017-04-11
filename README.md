# FreeSRP USB Controller

This is the source code for the firmware of the FreeSRP's [Cypress EZ-USB FX3](http://www.cypress.com/products/ez-usb-fx3-superspeed-usb-30-peripheral-controller) controller.

## Getting started

Install the [FX3 SDK](http://www.cypress.com/documentation/software-and-drivers/ez-usb-fx3-software-development-kit). Make sure you ``source`` its ``envsetup.sh`` script so the build system can find the libraries and compilers.

There's one utility in the SDK you need to compile first:
```
cd $FX3_INSTALL_PATH/util/elf2img
gcc elf2img.c -o elf2img
```

Then, just run:

```
mkdir build && cd build
cmake ..
make
```

to compile the firmware. You can now load ``FreeSRP.img`` onto the FX3.

## Note on the AD9364 related files in this repository

Right now, the FPGA runs a MicroBlaze processor that handles interfacing with the AD9364. However, this MicroBlaze will be eliminated and the FX3 will handle all of the interfacing with the AD9364. That's why there's a driver for the AD9364 in this repo, but it's not urrently being used and still needs some work (on both the FX3 and FPGA).
