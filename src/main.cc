/*
 * Copyright 2020 Telecom Paris

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <libconfig.h++>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include "rf/rf.h"
#include "../lib/variables/common_variables/common_variables.h"
#include "phy/phy.h"
#include "../lib/phy/libphy/libphy.h"
#include "../lib/asn1c/nr_rrc/SIB1.h"
#include "../lib/variables/common_variables/common_variables.h"

using namespace std;

/*
 * Define structure to store found cells
 */
typedef struct found_cell_ {
    free5GRAN::band band_obj;
    phy phy_obj;
    double  gain;
    int gscn, scs;
    bool dci_found;

} found_cell;

void search_cell_with_defined_params(double frequency, double ssb_period, const string &rf_address, free5GRAN::band band, int input_gain);
void scan_bands(vector<free5GRAN::band> BANDS, double ssb_period, const string &rf_address, int input_gain);
void init_logging(const string &level);


int main(int argc, char *argv[]) {

    //ofstream out("out.txt");
    //cout.rdbuf(out.rdbuf());



    libconfig::Config cfg;

    try
    {
        cfg.readFile(argv[1]);
        cout << "Using configuration file $PWD/" << argv[1] << endl;
    }
    catch(const libconfig::FileIOException &fioex)
    {
        try
        {
            string path = "/root/.config/free5GRAN/config/free5GRAN.cfg";
            cfg.readFile(path.c_str());
            cout << "Using configuration file " << path << endl;
        }
        catch(const libconfig::FileIOException &fioex)
        {
            cerr << "Please provide a config file!" << endl;
            return(EXIT_FAILURE);
        }
    }
    catch(const libconfig::ParseException &pex)
    {
        cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
             << " - " << pex.getError() << endl;
        return(EXIT_FAILURE);
    }

    try
    {
        string level;
        if (!cfg.lookupValue("logging", level)){
            level = "info";
        }
        init_logging(level);
        BOOST_LOG_TRIVIAL(info) << "Initializing free5GRAN";
        string func = cfg.lookup("function");
        string rf_address = cfg.lookup("rf_address");
        int gain = cfg.lookup("gain");
        const libconfig::Setting& root = cfg.getRoot();
        if (func == "CELL_SEARCH"){
            cout << "##  CELL SEARCH  ##" << endl;
            BOOST_LOG_TRIVIAL(info) << "CELL SEARCH";
            const libconfig::Setting& cell_info = root["cell_info"];
            double ssb_period;
            int band;
            free5GRAN::band band_obj;
            if (!root.lookupValue("ssb_period", ssb_period)){
                // Default ssb period to 20 ms
                ssb_period = 0.02;
            }if (cell_info.lookupValue("band", band) ){
                bool found_band = false;
                for (int j = 0; j < free5GRAN::NUM_SUPPORTED_BANDS; j ++){
                    if (free5GRAN::AVAILABLE_BANDS[j].number == band){
                        band_obj = free5GRAN::AVAILABLE_BANDS[j];
                        found_band = true;
                        BOOST_LOG_TRIVIAL(trace) << "Band n" + to_string(band);
                        break;
                    }
                }
                if (found_band) {
                    double frequency;
                    int gscn;
                    if (cell_info.lookupValue("ssb_frequency", frequency) || cell_info.lookupValue("gscn", gscn)){
                        if (!cell_info.lookupValue("ssb_frequency", frequency)){
                            frequency = free5GRAN::phy::signal_processing::compute_freq_from_gscn(gscn);
                        }
                        search_cell_with_defined_params(frequency, ssb_period, rf_address,band_obj, gain);
                    }
                    else {
                        cerr << "Missing parameters (frequency or GSCN) in config file" << endl;
                    }
                }else {
                    cout << "BAND not supported" << endl;
                }
            }
            else {
                cerr << "Missing parameters (band) in config file" << endl;
            }
        }else if (func == "BAND_SCAN"){
            BOOST_LOG_TRIVIAL(info) << "BAND SCANNING";
            cout << "##  BAND SCAN  ##" << endl;
            const libconfig::Setting& bands = root["bands"];
            int count = bands.getLength();
            vector<free5GRAN::band> band_array;
            BOOST_LOG_TRIVIAL(trace) << "Adding bands to scan: ";
            for (int i = 0; i < count; i ++){
                int band_id = bands[i];
                for (int j = 0; j < free5GRAN::NUM_SUPPORTED_BANDS; j ++){
                    if (free5GRAN::AVAILABLE_BANDS[j].number == band_id){
                        band_array.push_back(free5GRAN::AVAILABLE_BANDS[j]);
                        BOOST_LOG_TRIVIAL(trace) << "Band n" + to_string(band_id);
                        break;
                    }
                }
            }
            double ssb_period;
            if (!root.lookupValue("ssb_period", ssb_period)){
                // Default ssb period to 20 ms
                ssb_period = 0.02;
            }
            scan_bands(band_array, ssb_period, rf_address, gain);
        }
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        cerr << "Missing function, RF device address of gain in config file" << endl;
    }

    return EXIT_SUCCESS;
}


void scan_bands(vector<free5GRAN::band> BANDS, double ssb_period, const string &rf_address, int input_gain){
    /*
     * USRP parameters
     */
    string device_args("serial="+rf_address);
    string subdev("A:A");
    string ant("TX/RX");
    string ref("internal");
    /*
     * Cell default parameters
     */
    double freq(1000e6);
    double rate(3.84e6);
    double gain(input_gain);
    double bw(3.84e6);

    free5GRAN::band current_band;


    // Search cell with 15kHz SCS
    rf rf_device(rate, freq, gain, bw, subdev, ant, ref, device_args);
    vector<found_cell> found_cells;
    cout << "\n";
    auto start = chrono::high_resolution_clock::now();
    float received_power;
    /*
     * Loop over each supported band
     */
    for (int i = 0; i < BANDS.size() ; i ++){
        current_band = BANDS[i];
        BOOST_LOG_TRIVIAL(info) << "###########################################";
        BOOST_LOG_TRIVIAL(info) << "Scanning band n"+ to_string(current_band.number);
        /*
         * Search a cell by scanning different possible GSCN
         */
        for (int gscn = current_band.min_gscn; gscn <= current_band.max_gscn; gscn ++){
            /*
             * Reconfigure RF device and instanciate a phy object
             */
            freq = free5GRAN::phy::signal_processing::compute_freq_from_gscn(gscn);
            BOOST_LOG_TRIVIAL(info) << "Scanning gscn= "+ to_string(gscn) + " and freq= " + to_string(freq/1e6) + " MHz";

            free5GRAN::bandwidth_info band_info;
            if (freq < 3000e6){
                band_info = free5GRAN::BANDWIDTH_15_KHZ;
            }else {
                band_info = free5GRAN::BANDWIDTH_30_KHZ;
            }
            BOOST_LOG_TRIVIAL(trace) << "Bandwidth informations are: ";
            BOOST_LOG_TRIVIAL(trace) << "SCS: " + to_string(band_info.scs);

            int fft_size_pss = 128;
            rf_device.setCenterFrequency(freq);
            rf_device.setSampleRate(fft_size_pss * band_info.scs);
            rf_device.setGain(100);
            phy phy_layer(&rf_device, ssb_period, fft_size_pss, band_info.scs, current_band);


            cout << "\r## Searching in band n" + to_string(current_band.number) + " - " + to_string((((float) gscn - (float) current_band.min_gscn)/((float) current_band.max_gscn - (float) current_band.min_gscn)) * 100.0) + "% (found " + to_string(found_cells.size()) + " cells)";

            /*
             * If a PSS is correlate with a good peak value
             */
            if (phy_layer.cell_synchronization(received_power)==0){
                /*
                 * Power down-ramping to avoid saturation
                 */
                BOOST_LOG_TRIVIAL(trace) << "Received power: " + to_string(received_power) + " dB (Gain= "+ to_string(rf_device.getGain()) +" dB)";
                while (received_power > -2 && rf_device.getGain() > 0){
                    rf_device.setGain(rf_device.getGain()-10);
                    phy_layer.cell_synchronization(received_power);
                    BOOST_LOG_TRIVIAL(trace) << "Received power: " + to_string(received_power) + " dB (Gain= "+ to_string(rf_device.getGain()) +" dB)";
                }
                BOOST_LOG_TRIVIAL(trace) << "Final received power: " + to_string(received_power) + " dB (Final gain= "+ to_string(rf_device.getGain()) +" dB)";
                /*
                 * If the re-computed PCI is equal to the initially computed one
                 */
                rf_device.setSampleRate(30.72e6);
                int new_fft_size;
                new_fft_size = (int) (30.72e6/band_info.scs);
                phy_layer.reconfigure(new_fft_size);

                if(phy_layer.extract_pbch() == 0){
                    //phy_layer.print_cell_info();
                    bool dci_found;
                    phy_layer.search_pdcch(dci_found);
                    if (dci_found){
                        phy_layer.extract_pdsch();
                    }
                    found_cell new_cell;
                    new_cell.band_obj = current_band;
                    new_cell.scs = current_band.scs;
                    new_cell.gscn = gscn;
                    new_cell.phy_obj = phy_layer;
                    new_cell.gain = rf_device.getGain();
                    new_cell.dci_found = dci_found;
                    found_cells.push_back(new_cell);
                }
            }
        }
    }
    for (int i = 0; i < found_cells.size(); i ++){
        cout << "\n";
        cout << "#######################################################################" << endl;
        cout << "########################## CELL " + to_string(i) + " ##########################" << endl;
        cout << "#######################################################################" << endl;
        cout << "# GSCN: " + to_string(found_cells[i].gscn) + " / Frequency: " + to_string(free5GRAN::phy::signal_processing::compute_freq_from_gscn(found_cells[i].gscn)/1e6) + " MHz (band n" + to_string(found_cells[i].band_obj.number) + ")" << endl;
        cout << "# Gain: " + to_string(found_cells[i].gain) + " dB"<< endl;
        found_cells[i].phy_obj.print_cell_info();
        if (found_cells[i].dci_found || found_cells[i].phy_obj.getSIB1RV() == 1 || found_cells[i].phy_obj.getSIB1RV() == 2){
            found_cells[i].phy_obj.print_dci_info();
            try {
                found_cells[i].phy_obj.print_sib1();
            } catch (const std::exception& e) { cout << "NO SIB1" << endl; }
        }else {
            cout << "###### DCI" << endl;
            cout << "# DCI not found, not CRC validated, or unsupported RV";
            cout << "\n";
        }
        cout << "#######################################################################" << endl;

    }
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Executed in "+ to_string(duration.count()/1e6) + " seconds" << endl;
}



void search_cell_with_defined_params(double frequency, double ssb_period, const string &rf_address, free5GRAN::band band, int input_gain){
    /*
     * USRP parameters
     */
    string device_args("serial="+rf_address);
    string subdev("A:A");
    string ant("TX/RX");
    string ref("internal");
    /*
     * Cell parameters
     */
    double gain(input_gain);
    free5GRAN::bandwidth_info band_info;
    if (frequency < 3000e6){
        band_info = free5GRAN::BANDWIDTH_15_KHZ;
    }else {
        band_info = free5GRAN::BANDWIDTH_30_KHZ;
    }
    BOOST_LOG_TRIVIAL(trace) << "Bandwidth informations are: ";
    BOOST_LOG_TRIVIAL(trace) << "SCS: " + to_string(band_info.scs);
    /*
     * Instanciate a rf layer instance to provide exchanges with USRP device
     */
    int fft_size_pss = 128;
    rf rf_device(fft_size_pss * band_info.scs, frequency, gain, fft_size_pss * band_info.scs, subdev, ant, ref, device_args);

    //FIX BAND SELECTION

    phy phy_layer(&rf_device, ssb_period, fft_size_pss, band_info.scs, band);


    cout << "\n";
    cout << "########################## Searching cell ##########################" << endl;
    cout << "\n";
    cout << "###### RADIO" << endl;
    cout << "# Frequency: " + to_string(frequency/1e6) + " MHz" << endl;
    cout << "# SCS: " + to_string(band_info.scs/1e3) + " kHz" << endl;
    cout << "\n";

    BOOST_LOG_TRIVIAL(info) << "Searching cell with frequency = " + to_string(frequency/1e6) + " MHz and SCS = " + to_string(band_info.scs/1e3) + " kHz";

    float received_power;
    if (phy_layer.cell_synchronization(received_power) == 0){
        /*
         * Power down-ramping to avoid saturation
         */
        cout << "###### Power ramping" << endl;
        cout << "Received power: "+ to_string(received_power) + " dB" << endl;
        BOOST_LOG_TRIVIAL(trace) << "Received power: " + to_string(received_power) + " dB (Gain= "+ to_string(rf_device.getGain()) +" dB)";

        while (received_power > -2 && rf_device.getGain() > 0){
            rf_device.setGain(rf_device.getGain()-10);
            phy_layer.cell_synchronization(received_power);
            cout << "New gain: "+ to_string(rf_device.getGain()) + " dB" << endl;
            cout << "Received power: "+ to_string(received_power) + " dB" << endl;
            BOOST_LOG_TRIVIAL(trace) << "Received power: " + to_string(received_power) + " dB (Gain= "+ to_string(rf_device.getGain()) +" dB)";
        }
        cout << "\n";
        BOOST_LOG_TRIVIAL(trace) << "Final received power: " + to_string(received_power) + " dB (Final gain= "+ to_string(rf_device.getGain()) +" dB)";
        rf_device.setSampleRate(30.72e6);
        int new_fft_size;
        new_fft_size = (int) (30.72e6/band_info.scs);
        phy_layer.reconfigure(new_fft_size);

        if(phy_layer.extract_pbch() == 0){
            phy_layer.print_cell_info();
            bool dci_found;
            phy_layer.search_pdcch(dci_found);
            phy_layer.print_dci_info();
            if (dci_found){
                phy_layer.extract_pdsch();
                phy_layer.print_sib1();
            }
            cout << "####################################################################" << endl;
        }else {
            cout << "########################## Cell not confirmed ##########################" << endl;
            cout << "####################################################################" << endl;
        }
    }else {
        cout << "########################## Cell not found ##########################" << endl;
        cout << "####################################################################" << endl;
    }
    cout << "####################################################################" << endl;

}

void init_logging(const string &level)
{
    boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");

    boost::log::add_file_log
            (
                    boost::log::keywords::file_name = "/var/log/free5GRAN/free5GRAN.log",
                    boost::log::keywords::format = "[%TimeStamp%] [%ThreadID%] [%Severity%] %Message%"
            );

    if (level == "trace"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::trace
                );
    }else if (level == "debug"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::debug
                );
    }else if (level == "info"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::info
                );
    }else if (level == "warning"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::warning
                );
    }else if (level == "error"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::error
                );
    }else {
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::fatal
                );
    }

    boost::log::add_common_attributes();
}
