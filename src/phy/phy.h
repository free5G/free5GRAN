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

#ifndef FREE5GRAN_PHY_H
#define FREE5GRAN_PHY_H

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <condition_variable>
#include <vector>

#include "../../lib/asn1c/nr_rrc/BCCH-DL-SCH-Message.h"
#include "../../lib/phy/bwp/bwp.h"
#include "../../lib/rf/rf.h"
#include "../../lib/variables/common_structures/common_structures.h"

class phy {
 private:
  free5GRAN::rf* rf_device;   //< pointer to the SDR RF layer device
  int common_cp_length,       //< CP length of common symbols
      pci,                    //< cell PCI
      is_extended_cp,         //< Is cell extended CP ?
      i_ssb,                  //< SSB index
      l_max,                  //< Max number of SSB iterations with a SSB period
      frame_size;             //< Number of samples per frame
  double ssb_period;          //< SSB period
  bool cell_confirmed;        //< Is cell confirmed ?
  float freq_offset;          //< Frequency offset between gNodeB and UE in Hz
  free5GRAN::mib mib_object;  //< MIB structure
  free5GRAN::band band_object;                 // Band structure
  free5GRAN::dci_1_0_si_rnti dci_1_0_si_rnti;  //< DCI Format 1_0
  std::chrono::time_point<std::chrono::high_resolution_clock>
      time_first_pss;                     //< Time first PSS
  std::vector<std::complex<float>> buff;  //< Studied buffer
  BCCH_DL_SCH_Message_t* sib1 = nullptr;  //< SIB1 message
  free5GRAN::ss_power_indicator ss_pwr =
      {};                            //< Synchronization signal power structure
  free5GRAN::rf_buffer* rf_buff;     //< Variables shared with RF containing
                                     // primary and frame buffers
  bool* stop_signal;                 //< End program switch
  free5GRAN::phy::bwp* current_bwp;  //< Current BWP object
  vector<free5GRAN::phy::bwp*>
      available_bwps;  //< Vector containing all the BWPs

 public:
  phy(free5GRAN::rf* rf_dev,
      double ssb_period,
      int fft_size,
      int scs,
      free5GRAN::band band_obj,
      free5GRAN::rf_buffer* rf_buff,
      bool* stp_signal);
  phy();
  ~phy();

  void print_cell_info();

  void parse_dci_1_0_si_rnti(vector<int>& dci_bits,
                             int freq_domain_ra_size,
                             free5GRAN::dci_1_0_si_rnti& dci);

  void print_dci_info();

  auto getSib() -> BCCH_DL_SCH_Message_t*;

  void print_sib1();

  auto getSIB1RV() -> int;

  void printSize() {
    std::cout << "BUFFER SIZE PHY: " << rf_buff->primary_buffer->size()
              << std::endl;
  }

  auto init(free5GRAN::synchronization_object& sync_object,
            condition_variable& cond_var_cell_sync) -> int;
};

#endif
