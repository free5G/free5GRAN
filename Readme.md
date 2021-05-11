# free5GRAN

free5GRAN is an open-source 5G RAN stack. The current version includes a receiver which decodes MIB & SIB1 data. It also acts as a cell scanner. free5GRAN works in SA mode.

free5GRAN includes a library which can be reused for further developments. Documentation is available https://free5g.github.io/free5GRAN-documentation/.

## LGTM scores

[![Total alerts](https://img.shields.io/lgtm/alerts/g/free5G/free5GRAN.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/free5G/free5GRAN/alerts/)

[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/free5G/free5GRAN.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/free5G/free5GRAN/context:cpp)

## Codacy score
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/81153c8875f147b3aa9565b8ea5e56b6)](https://app.codacy.com/gh/free5G/free5GRAN?utm_source=github.com&utm_medium=referral&utm_content=free5G/free5GRAN&utm_campaign=Badge_Grade)

## Installation & Running

### Requirements
Before compiling, make sure that the following libraries are installed:
* cmake
* libfftw3
* libconfig++
* boost
* cpufrequtils

On ubuntu, those libraries can be installed via:
```
sudo apt-get install cmake libfftw3-dev libconfig++-dev libboost-program-options-dev libboost-log-dev cpufrequtils
```

### Radio frontend

We currently support USRP B210, N210 and X310 natively. `libuhd-dev` is required. It can be installed on Ubuntu via:
```
sudo apt-get install libuhd-dev
```

USRP N210 is partially supported (it can be unstable) and `free5GRAN` requires Liquid SDR library for sampling rate resampling : `https://liquidsdr.org/doc/installation/`. To build `free5GRAN` with USRP N210 support, just run the `cmake` command with option `-DINCLUDE_N210=ON`.

### Compiling

free5GRAN executable can be compiled from sources with the following commands:
```
git clone https://github.com/free5G/free5GRAN.git
cd free5GRAN
mkdir build
cd build
cmake ..
make
sudo make install
```

### Configuration and running

Two functions are supported by free5GRAN.

#### Cell search
Searching a cell with pre-defined parameters. Example config file is provided (`free5GRAN/config/search_cell.cfg`).

#### Band scanning
Scanning one or more 5G NR bands. Example config file is provided (`free5GRAN/config/scan_band.cfg`).

#### USRP configuration

##### Pre-defined USRP device
If you want to use a pre-defined RF device, USRP device has to be configured:
* Run `uhd_find_devices`
* Find the device you want to use
* Copy the `serial` field of the device and paste it in the config file.

##### USRP network configuration
To run `free5GRAN` with USRP X310, set mtu size to 8000: `ifconfig ETHERNET_INTERFACE mtu 8000`.

To run `free5GRAN` with USRP N210, set mtu size to 1500: `ifconfig ETHERNET_INTERFACE mtu 1500`.

#### Running

`free5GRAN` should be available globally after installation. Configuration files are located in `/root/.config/free5GRAN/config`. There is two possible options for running free5GRAN:
* Run `sudo free5GRAN` to run `free5GRAN` with default config file `/root/.config/free5GRAN/config/free5GRAN.cfg`.
* Run `sudo free5GRAN RELATIVE_CONFIG_FILE_PATH` to run `free5GRAN` with a specific configuration file.

### Debugging

free5GRAN writes logs in `/var/log/free5GRAN/free5GRAN.log`. It can be used for debugging. Foremost, free5RGAN writes some data files in `/root/.files/free5GRAN/execution_raw_files/` directory (if it exists). For plotting those files, run `python analyse.py` in `/root/.files/free5GRAN/`. This python script generates files that are stored in `/root/.files/visualization_files/` directory.

## Testing note 

free5GRAN has been successfully tested on Ubuntu 18.04 with USRP B210. It is currently under active development, which means that it might not be fully stable. 

## Issues

All the issues have to be reported in the `issues` page of this repository. Some known issues about the current version are already detailed in `issues` section.

## 5G NSA support

As 5G NSA uses 4G cell for attachment, SIB are transmitted on a 4G cell and this receiver won't be able to decode SIB1 data from 5G NSA mode. However, this receiver should be able to detect 5G NSA cell and decode MIB data.

## Supported bands

A number of bands are already supported by free5GRAN. New bands can be added in `free5GRAN/lib/variables/common_structures`
