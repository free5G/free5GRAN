# free5GRAN

free5GRAN is an open-source 5G RAN stack. The current version includes a receiver which decodes MIB & SIB1 data. It also acts as a cell scanner. free5GRAN works in SA mode.

free5GRAN includes a library which can be reused for further developments. Documentation is available HERE.

## Installation & Running

### Requirements
Before compiling, make sure that the following libraries are installed:
* cmake
* libfftw3
* libconfig++
* boost

On ubuntu, those libraries can be installed via:
```
sudo apt-get install cmake libfftw3-dev libconfig++-dev libboost-program-options-dev libboost-log-dev
```

### Radio frontend

We currently support USRP B210. `libuhd` and `libuhd-dev` are required. They can be installed on Ubuntu via:
```
sudo apt-get install libuhd libuhd-dev
```

### Compiling

free5GRAN executable can be compiled from sources with the following commands:
```
git clone https://github.com/free5G/free5GRAN.git
cd free5GRAN
mkdir files
mkdir build
cd build
mkdir output_files
cmake ..
make
```

### Configuration and running

Two functions are supported by free5GRAN.

#### Cell search
Searching a cell with pre-defined parameters. Example config file is provided (`free5GRAN/config/search_cell.cfg`).

#### Band scanning
Scanning one or more 5G NR bands. Example config file is provided (`free5GRAN/config/scan_band.cfg`).

### USRP configuration
Before running free5GRAN, USRP B210 device has to be configured:
* Run `uhd_find_devices`
* Find the device you want to use
* Copy the `serial` field of the device and paste it in the config file.

To run free5GRAN: `sudo ./free5GRAN ../config/CONFIG_FILE`


### Debugging

free5GRAN writes logs in `build/free5GRAN.log`. It can be used for debugging. Foremost, free5RGAN writes some data files in `free5gran/build/output_files` directory. For plotting those files, run `python analyse.py` in project `free5gran/` directory. This python script generates files that are stored in `free5gran/files` directory.

## Testing note 

free5GRAN has been successfully tested on Ubuntu 18.04 with USRP B210. It is currently under active development, which means that it might not be fully stable. 