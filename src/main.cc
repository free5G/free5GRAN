/*
 * Copyright 2020-2021 Telecom Paris

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

#include <sched.h>
#include <semaphore.h>
#include <unistd.h>

#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <iostream>
#include <libconfig.h++>
#include <mutex>
#include <thread>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>

#include "../lib/rf/rf.h"
#include "../lib/rf/usrp_b200/usrp_b200.h"
#include "../lib/rf/usrp_x300/usrp_x300.h"
#ifdef INCLUDE_N210_OPT
#include "../lib/rf/usrp_usrp2/usrp_usrp2.h"
#endif

#include <fftw3.h>
#include <sys/resource.h>
#include <sys/sysinfo.h>
#include "../lib/asn1c/nr_rrc/SIB1.h"
#include "../lib/phy/libphy/libphy.h"
#include "../lib/utils/common_utils/common_utils.h"
#include "../lib/variables/common_variables/common_variables.h"
#include "phy/phy.h"

using namespace std;

/*
 * Define structure to store found cells
 */
using found_cell = struct found_cell_ {
  free5GRAN::band band_obj;
  phy phy_obj;
  double gain;
  int gscn, scs;
  bool dci_found;
};

void search_cell_with_defined_params(double frequency,
                                     double ssb_period,
                                     const string& rf_address,
                                     free5GRAN::band band,
                                     int input_gain);
void scan_bands(vector<free5GRAN::band> BANDS,
                double ssb_period,
                const string& rf_address,
                int input_gain);
void init_logging(const string& level);

static bool stop_signal = false;

void sigint(int sig) {
  cout << "\nStopping free5GRAN\n" << endl;
  stop_signal = true;
}

auto main(int argc, char* argv[]) -> int {
  cout << "C++ Version: " << __cplusplus << endl;
  // Set CPUs to performance mode:
  int res;
  for (int i = 0; i < get_nprocs_conf(); i++) {
    string command =
        "sudo cpufreq-set -c " + to_string(i) + " -r -g performance";
    res = system(command.c_str());
  }
  // Set process priority
  setpriority(PRIO_PROCESS, 0, -20);

  libconfig::Config cfg;
  // Try to read configuration file
  try {
    // User-specific config file
    cfg.readFile(argv[1]);
    cout << "Using configuration file $PWD/" << argv[1] << endl;
  } catch (const libconfig::FileIOException& fioex) {
    // Default config file
    try {
      string path = "/root/.config/free5GRAN/config/free5GRAN.cfg";
      cfg.readFile(path.c_str());
      cout << "Using configuration file " << path << endl;
    } catch (const libconfig::FileIOException& fioex) {
      // No config file found
      cerr << "Please provide a config file!" << endl;
      return (EXIT_FAILURE);
    }
  } catch (const libconfig::ParseException& pex) {
    // Libconfig parsing error
    cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - "
         << pex.getError() << endl;
    return (EXIT_FAILURE);
  }

  try {
    // Trying to read configuration parameters
    // logging level
    string level;
    if (!cfg.lookupValue("logging", level)) {
      level = "info";
    }
    init_logging(level);
    BOOST_LOG_TRIVIAL(info) << "Initializing free5GRAN";
    // Receiver mode
    string func = cfg.lookup("function");
    // USRP rf serial address
    string rf_address;
    const libconfig::Setting& root = cfg.getRoot();
    if (!root.lookupValue("rf_address", rf_address)) {
      rf_address = "";
    }
    int gain = cfg.lookup("gain");
    // If receiver mode is search cell
    if (func == "CELL_SEARCH") {
      cout << "##  CELL SEARCH  ##" << endl;
      BOOST_LOG_TRIVIAL(info) << "CELL SEARCH";
      // Search cell info
      const libconfig::Setting& cell_info = root["cell_info"];
      double ssb_period;
      int band;
      free5GRAN::band band_obj;
      // SSB period
      if (!root.lookupValue("ssb_period", ssb_period)) {
        // Default ssb period to 20 ms
        ssb_period = 0.02;
      }
      // Search band and check whether or not it is supported
      if (cell_info.lookupValue("band", band)) {
        bool found_band = false;
        for (int j = 0; j < free5GRAN::NUM_SUPPORTED_BANDS; j++) {
          if (free5GRAN::AVAILABLE_BANDS[j].number == band) {
            band_obj = free5GRAN::AVAILABLE_BANDS[j];
            found_band = true;
            BOOST_LOG_TRIVIAL(trace) << "Band n" + to_string(band);
            break;
          }
        }
        if (found_band) {
          double frequency;
          int gscn;
          // Search either cell frequency or GSCN
          if (cell_info.lookupValue("ssb_frequency", frequency) ||
              cell_info.lookupValue("gscn", gscn)) {
            if (!cell_info.lookupValue("ssb_frequency", frequency)) {
              frequency =
                  free5GRAN::phy::signal_processing::compute_freq_from_gscn(
                      gscn);
            }
            // Call search cell function
            search_cell_with_defined_params(frequency, ssb_period, rf_address,
                                            band_obj, gain);
          } else {
            // Missing a parameter
            cerr << "Missing parameters (frequency or GSCN) in config file"
                 << endl;
          }
        } else {
          // Band is not supported
          cout << "BAND not supported" << endl;
        }
      } else {
        // Missing a parameter
        cerr << "Missing parameters (band) in config file" << endl;
      }
    } else if (func == "BAND_SCAN") {
      // Scan band mode
      BOOST_LOG_TRIVIAL(info) << "BAND SCANNING";
      cout << "##  BAND SCAN  ##" << endl;
      // Search for bands
      const libconfig::Setting& bands = root["bands"];
      int count = bands.getLength();
      vector<free5GRAN::band> band_array;
      BOOST_LOG_TRIVIAL(trace) << "Adding bands to scan: ";
      for (int i = 0; i < count; i++) {
        int band_id = bands[i];
        for (int j = 0; j < free5GRAN::NUM_SUPPORTED_BANDS; j++) {
          if (free5GRAN::AVAILABLE_BANDS[j].number == band_id) {
            band_array.push_back(free5GRAN::AVAILABLE_BANDS[j]);
            BOOST_LOG_TRIVIAL(trace) << "Band n" + to_string(band_id);
            break;
          }
        }
      }
      double ssb_period;
      if (!root.lookupValue("ssb_period", ssb_period)) {
        // Default ssb period to 20 ms
        ssb_period = 0.02;
      }
      // call function
      scan_bands(band_array, ssb_period, rf_address, gain);
    }
  } catch (const libconfig::SettingNotFoundException& nfex) {
    // Some parameters are missing
    cerr << "Missing function, RF device address of gain in config file"
         << endl;
  }
  return EXIT_SUCCESS;
}

void scan_bands(vector<free5GRAN::band> BANDS,
                double ssb_period,
                const string& rf_address,
                int input_gain) {
  setenv("UHD_LOG_FASTPATH_DISABLE", "1", 0);
  /*
   * Cell default parameters
   */
  double freq(1000e6);
  double rate(3.84e6);
  double gain(input_gain);
  double bw(3.84e6);

  // Circular buffer size = 100 frames = 100 * 0.01 = 1 ms
  size_t buffer_size = 100;
  // primary_buffer will contain raw frames received by UE
  // frame_buffer will contain cell-synchronized frames after adjusting primary
  // frames
  boost::circular_buffer<free5GRAN::buffer_element> primary_buffer(buffer_size),
      frame_buffer(buffer_size);
  // condition_variable_vector is vector of condition variables for frame
  // reception notifications condition_variable_frame_vector is another vector
  // of condition variables for frame adjustment notifications
  vector<condition_variable> condition_variable_vector(buffer_size),
      condition_variable_frame_vector(buffer_size);

  // semaphore is a semaphore signal for synchronizing exchanges between
  // reception and adjusting threads
  sem_t semaphore;

  // Build a structure that contains all the variables shared between the
  // different threads and processes
  free5GRAN::rf_buffer rf_buff = {
      .primary_buffer = &primary_buffer,
      .frame_buffer = &frame_buffer,
      .cond_var_vec_prim_buffer = &condition_variable_vector,
      .cond_var_vec_frame_buffer = &condition_variable_frame_vector,
      .semaphore = &semaphore,
      .frame_thread_started = false,
  };
  // Initialize semaphore
  sem_init(rf_buff.semaphore, 1, 0);

  /*
   * Find USRP device with input parameters parameters
   */
  free5GRAN::rf_device chosen_device;
  free5GRAN::utils::common_utils::select_rf_device(chosen_device, rf_address);
  if (chosen_device.type.empty()) {
    cout << "Cannot find USRP device ! Exiting..." << endl;
    return;
  } else {
    cout << "Using USRP " << chosen_device.type << " device" << endl;
  }
  /*
   * Create RF device depending on RF type.
   */
  double bandwidth = 30.72e6;
  free5GRAN::rf* rf_device;
  if (chosen_device.type == "b200") {
    rf_device = new free5GRAN::usrp_b200(bandwidth, freq, gain, bandwidth,
                                         chosen_device, &rf_buff);
  } else if (chosen_device.type == "x300") {
    rf_device = new free5GRAN::usrp_x300(bandwidth, freq, gain, bandwidth,
                                         chosen_device, &rf_buff);
  }
#ifdef INCLUDE_N210_OPT
  else if (chosen_device.type == "usrp2") {
    rf_device = new free5GRAN::usrp_usrp2(bandwidth, freq, gain, bandwidth,
                                          chosen_device, &rf_buff);
  }
#endif
  else {
    cout << "Unsupported RF device" << endl;
    return;
  }

  // Request bw because it may have been modified if requested bw not supported
  // by RF device.
  bandwidth = rf_device->getSampleRate();
  cout << "FINAL BANDWIDTH: " << bandwidth << endl;

  // Initialize FFTW plans
  free5GRAN::utils::common_utils::init_fft_plans(bandwidth);

  // CTRL+C handler
  signal(SIGINT, &sigint);

  // Start receiving primary frames
  boost::thread recv_thread(
      [rf_device, &capture0 = stop_signal, capture1 = 0.01 * bandwidth] {
        rf_device->start_loopback_recv(capture0, capture1);
      });

  free5GRAN::band current_band;

  vector<found_cell> found_cells;
  cout << "\n";
  auto start = chrono::high_resolution_clock::now();
  float received_power;
  /*
   * Loop over each supported band
   */
  for (auto& i : BANDS) {
    if (stop_signal)
      break;
    current_band = i;
    BOOST_LOG_TRIVIAL(info) << "###########################################";
    BOOST_LOG_TRIVIAL(info)
        << "Scanning band n" + to_string(current_band.number);
    /*
     * Search a cell by scanning different possible GSCN
     */
    for (int gscn = current_band.min_gscn; gscn <= current_band.max_gscn;
         gscn++) {
      if (stop_signal)
        break;
      /*
       * Reconfigure RF device and instanciate a phy object
       */
      freq = free5GRAN::phy::signal_processing::compute_freq_from_gscn(gscn);
      bool local_stop_signal = false;

      BOOST_LOG_TRIVIAL(info) << "Scanning gscn= " + to_string(gscn) +
                                     " and freq= " + to_string(freq / 1e6) +
                                     " MHz";

      free5GRAN::bandwidth_info band_info;
      if (freq < 3000e6) {
        band_info = free5GRAN::BANDWIDTH_15_KHZ;
      } else {
        band_info = free5GRAN::BANDWIDTH_30_KHZ;
      }
      BOOST_LOG_TRIVIAL(trace) << "Bandwidth informations are: ";
      BOOST_LOG_TRIVIAL(trace) << "SCS: " + to_string(band_info.scs);

      rf_device->setCenterFrequency(freq);
      rf_device->setGain(input_gain);
      // Wait circular buffer to be fully renewed with new RF parameters frames
      usleep(1100000);

      cout << "\r## Searching in band n" + to_string(current_band.number) +
                  " - " + to_string(freq / 1e6) + " MHz" + " - " +
                  to_string((((float)gscn - (float)current_band.min_gscn) /
                             ((float)current_band.max_gscn -
                              (float)current_band.min_gscn)) *
                            100.0) +
                  "% (found " + to_string(found_cells.size()) + " cells)";
      // Compute PBCH FFT size
      int fft_size = (int)bandwidth / band_info.scs;
      // Instanciate a PHY layer
      phy phy_layer(rf_device, ssb_period, fft_size, band_info.scs,
                    current_band, &rf_buff, &stop_signal);

    phy_initialization:
      // sync_object will contain synchronization variables that will be shared
      // between threads
      free5GRAN::synchronization_object sync_object;
      // sync_completed is a condition variable that will be notified when PHY
      // layer has synchronized with cell
      condition_variable sync_completed;
      // Run init function.
      // It will first, synchronize with the cell, then it will decode DCI and
      // SIB1
      boost::thread phy_init([&phy_layer, &sync_object, &sync_completed] {
        phy_layer.init(sync_object, sync_completed);
      });
      // Wait PHY layer for cell synchronization
      mutex m;
      unique_lock<mutex> lk(m);
      sync_completed.wait(lk);
      lk.unlock();
      // Check if PBCH CRC is validated
      if (!sync_object.mib_crc_val) {
        // If not
        // Finish PHY layer processing
        phy_init.join();
        // If required, perform power down-ramping
        if (sync_object.received_power > -2 && rf_device->getGain() > 0) {
          rf_device->setGain(rf_device->getGain() - 10);
          cout << "Decreasing gain to " << rf_device->getGain() << " dB"
               << endl;
          // Retry to synchronize with cell
          if (!stop_signal) {
            goto phy_initialization;
          }
        }
      } else {
        // If CRC validated
        // cont_sync_sem is a semaphore variable for synchronizing adjust and
        // resynchronization threads
        sem_t cont_sync_sem;
        sync_object.cont_sync_sem = &cont_sync_sem;
        sem_init(sync_object.cont_sync_sem, 1, 0);

        // resync_thread will resynchronize with cell continuously
        boost::thread resync_thread([rf_device, &sync_object,
                                     &capture0 = stop_signal,
                                     &local_stop_signal] {
          rf_device->resynchronization(sync_object, capture0,
                                       local_stop_signal);
        });
        // adjust_thread continuously adjusts primary frames into
        // gNodeB-synchronized frames
        boost::thread adjust_thread(
            [rf_device, &sync_object, capture0 = 0.01 * bandwidth,
             &capture1 = stop_signal, &local_stop_signal] {
              rf_device->adjust_frames(sync_object, capture0, capture1,
                                       local_stop_signal);
            });

        // Wait PHY init to finish
        phy_init.join();
        // Stop resync and adjust threads
        local_stop_signal = true;
        resync_thread.join();
        adjust_thread.join();
        // Re-init variables and buffers
        rf_buff.frame_thread_started = false;
        rf_buff.frame_buffer->clear();
        rf_buff.cond_var_vec_frame_buffer->clear();
        // Add cell to found cells
        found_cell new_cell;
        new_cell.band_obj = current_band;
        new_cell.scs = current_band.ssb_pattern.scs;
        new_cell.gscn = gscn;
        // new_cell.phy_obj = phy_layer;
        new_cell.gain = rf_device->getGain();
        found_cells.push_back(new_cell);
        cout << "FOUND CELL" << endl;
      }
    }
  }
  // Stop all threads
  stop_signal = true;
  recv_thread.join();
  free5GRAN::utils::common_utils::destroy_fft_plans();
  auto stop = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Executed in " + to_string(duration.count() / 1e6) + " seconds"
       << endl;
}

void search_cell_with_defined_params(double frequency,
                                     double ssb_period,
                                     const string& rf_address,
                                     free5GRAN::band band,
                                     int input_gain) {
  setenv("UHD_LOG_FASTPATH_DISABLE", "1", 0);
  /*
   * Cell parameters
   */
  double gain(input_gain);
  free5GRAN::bandwidth_info band_info;
  if (frequency < 3000e6) {
    band_info = free5GRAN::BANDWIDTH_15_KHZ;
  } else {
    band_info = free5GRAN::BANDWIDTH_30_KHZ;
  }
  BOOST_LOG_TRIVIAL(trace) << "Bandwidth informations are: ";
  BOOST_LOG_TRIVIAL(trace) << "SCS: " + to_string(band_info.scs);
  /*
   * Instanciate a rf layer instance to provide exchanges with USRP device
   */

  // Circular buffer size = 100 frames = 100 * 0.01 = 1 ms
  size_t buffer_size = 100;
  // primary_buffer will contain raw frames received by UE
  // frame_buffer will contain cell-synchronized frames after adjusting primary
  // frames
  boost::circular_buffer<free5GRAN::buffer_element> primary_buffer(buffer_size),
      frame_buffer(buffer_size);
  // condition_variable_vector is vector of condition variables for frame
  // reception notifications condition_variable_frame_vector is another vector
  // of condition variables for frame adjustment notifications
  vector<condition_variable> condition_variable_vector(buffer_size),
      condition_variable_frame_vector(buffer_size);

  // semaphore is a semaphore signal for synchronizing exchanges between
  // reception and adjusting threads
  sem_t semaphore;

  // Build a structure that contains all the variables shared between the
  // different threads and processes
  free5GRAN::rf_buffer rf_buff = {
      .primary_buffer = &primary_buffer,
      .frame_buffer = &frame_buffer,
      .cond_var_vec_prim_buffer = &condition_variable_vector,
      .cond_var_vec_frame_buffer = &condition_variable_frame_vector,
      .semaphore = &semaphore,
      .frame_thread_started = false,
  };
  // Initialize semaphore
  sem_init(rf_buff.semaphore, 1, 0);
  /*
   * Find USRP device with input parameters parameters
   */
  free5GRAN::rf_device chosen_device;
  free5GRAN::utils::common_utils::select_rf_device(chosen_device, rf_address);
  if (chosen_device.type.empty()) {
    cout << "Cannot find USRP device ! Exiting..." << endl;
    return;
  } else {
    cout << "Using USRP " << chosen_device.type << " device" << endl;
  }
  /*
   * Create RF device depending on RF type.
   */
  double bandwidth = 30.72e6;
  free5GRAN::rf* rf_device;
  if (chosen_device.type == "b200") {
    rf_device = new free5GRAN::usrp_b200(bandwidth, frequency, gain, bandwidth,
                                         chosen_device, &rf_buff);
  } else if (chosen_device.type == "x300") {
    rf_device = new free5GRAN::usrp_x300(bandwidth, frequency, gain, bandwidth,
                                         chosen_device, &rf_buff);
  }
#ifdef INCLUDE_N210_OPT
  else if (chosen_device.type == "usrp2") {
    rf_device = new free5GRAN::usrp_usrp2(bandwidth, frequency, gain, bandwidth,
                                          chosen_device, &rf_buff);
  }
#endif
  else {
    cout << "Unsupported RF device" << endl;
    return;
  }

  cout << "########################## Searching cell ##########################"
       << endl;
  cout << "\n";
  cout << "###### RADIO" << endl;
  cout << "# Frequency: " + to_string(frequency / 1e6) + " MHz" << endl;
  cout << "# SCS: " + to_string(band_info.scs / 1e3) + " kHz" << endl;
  cout << "\n";

  // Request bw because it may have been modified if requested bw not supported
  // by RF device.
  bandwidth = rf_device->getSampleRate();
  cout << "FINAL BANDWIDTH: " << bandwidth / 1e6 << "MHz" << endl;

  // Initialize FFTW plans
  free5GRAN::utils::common_utils::init_fft_plans(bandwidth);

  // CTRL+C handler
  signal(SIGINT, &sigint);

  // Start receiving primary frames
  boost::thread recv_thread(
      [rf_device, &capture0 = stop_signal, capture1 = 0.01 * bandwidth] {
        rf_device->start_loopback_recv(capture0, capture1);
      });

  // Compute PBCH FFT size
  int fft_size = (int)bandwidth / band_info.scs;
  // Instanciate a PHY layer
  phy phy_layer(rf_device, ssb_period, fft_size, band_info.scs, band, &rf_buff,
                &stop_signal);

phy_initialization:
  // sync_object will contain synchronization variables that will be shared
  // between threads
  free5GRAN::synchronization_object sync_object;
  // sync_completed is a condition variable that will be notified when PHY layer
  // has synchronized with cell
  condition_variable sync_completed;
  // Run init function.
  // It will first, synchronize with the cell, then it will decode DCI and SIB1
  boost::thread phy_init([&phy_layer, &sync_object, &sync_completed] {
    phy_layer.init(sync_object, sync_completed);
  });
  // Wait PHY layer for cell synchronization
  mutex m;
  unique_lock<mutex> lk(m);
  sync_completed.wait(lk);
  lk.unlock();
  // Check if PBCH CRC is validated
  if (!sync_object.mib_crc_val) {
    // If not
    // Finish PHY layer processing
    phy_init.join();
    // If required, perform power down-ramping
    if (sync_object.received_power > -2 && rf_device->getGain() > 0) {
      rf_device->setGain(rf_device->getGain() - 5);
      cout << "Decreasing gain to " << rf_device->getGain() << " dB" << endl;
    } else {
      cout << "MIB decoding failed, trying again in 2 sec" << endl;
      usleep(2000000);
    }
    // Retry to synchronize with cell
    if (!stop_signal) {
      goto phy_initialization;
    }
  }
  // If CRC validated
  bool local_stop_signal = false;
  // cont_sync_sem is a semaphore variable for synchronizing adjust and
  // resynchronization threads
  sem_t cont_sync_sem;
  sync_object.cont_sync_sem = &cont_sync_sem;
  sem_init(sync_object.cont_sync_sem, 1, 0);

  // resync_thread will resynchronize with cell continuously
  boost::thread resync_thread(
      [rf_device, &sync_object, &capture0 = stop_signal, &local_stop_signal] {
        rf_device->resynchronization(sync_object, capture0, local_stop_signal);
      });
  // adjust_thread continuously adjusts primary frames into gNodeB-synchronized
  // frames
  boost::thread adjust_thread([rf_device, &sync_object,
                               capture0 = 0.01 * bandwidth,
                               &capture1 = stop_signal, &local_stop_signal] {
    rf_device->adjust_frames(sync_object, capture0, capture1,
                             local_stop_signal);
  });

  // Stop receive, resync and adjust threads
  resync_thread.join();
  adjust_thread.join();
  recv_thread.join();
  free5GRAN::utils::common_utils::destroy_fft_plans();
}

void init_logging(const string& level) {
  boost::log::register_simple_formatter_factory<
      boost::log::trivial::severity_level, char>("Severity");

  boost::log::add_file_log(
      boost::log::keywords::file_name = "/var/log/free5GRAN/free5GRAN.log",
      boost::log::keywords::format =
          "[%TimeStamp%] [%ThreadID%] [%Severity%] %Message%");

  if (level == "trace") {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        boost::log::trivial::trace);
  } else if (level == "debug") {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        boost::log::trivial::debug);
  } else if (level == "info") {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        boost::log::trivial::info);
  } else if (level == "warning") {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        boost::log::trivial::warning);
  } else if (level == "error") {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        boost::log::trivial::error);
  } else {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        boost::log::trivial::fatal);
  }

  boost::log::add_common_attributes();
}
