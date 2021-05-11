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

#include "phy.h"

#include <fftw3.h>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

#include "../../lib/asn1c/nr_rrc/BCCH-DL-SCH-Message.h"
#include "../../lib/asn1c/nr_rrc/FrequencyInfoUL-SIB.h"
#include "../../lib/phy/libphy/libphy.h"
#include "../../lib/phy/physical_channel/physical_channel.h"
#include "../../lib/phy/synchronization/synchronization.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include "../../lib/utils/common_utils/common_utils.h"
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/variables/common_structures/common_structures.h"
#include "../../lib/variables/common_variables/common_variables.h"

using namespace std;

phy::phy(free5GRAN::rf* rf_dev,
         double ssb_period,
         int fft_size,
         int scs,
         free5GRAN::band band_obj,
         free5GRAN::rf_buffer* rf_buff,
         bool* stp_signal) {
  /**
   * \fn phy
   * \param[in] rf_dev: RF device. (Only USRP B210 is currently supported)
   * \param[in] ssb_period: SSB periodicity. Default value is 0.02 (20 ms)
   * \param[in] fft_size: FFT/iFFT size. Represents the total number of os
   * subcarriers to be decoded \param[in] scs: Subcarrier spacing \param[in]
   * band_obj: Band object for getting Lmax value
   */
  this->rf_device = rf_dev;
  this->ssb_period = ssb_period;
  this->band_object = band_obj;
  l_max = band_obj.ssb_pattern.l_max;
  this->is_extended_cp = 0;
  common_cp_length = 0;
  this->rf_buff = rf_buff;
  this->stop_signal = stp_signal;
  this->current_bwp =
      new free5GRAN::phy::bwp(0, scs, 20, rf_device->getSampleRate());
  this->available_bwps.push_back(current_bwp);
}

phy::phy() = default;

void phy::print_cell_info() {
  /**
   * \fn print_cell_info
   * \brief Print cells global informations and MIB.
   */
  cout << "\n";
  cout << "###### RADIO" << endl;
  cout << "# SSB Power: " + to_string(ss_pwr.received_power) + " db" << endl;
  cout << "# SS-RSRP: " + to_string(ss_pwr.ss_rsrp) + " dbm" << endl;
  cout << "# SS-RSSI: " + to_string(ss_pwr.ss_rssi) + " dbm" << endl;
  cout << "# SS-RSRQ: " + to_string(ss_pwr.ss_rsrq) + " db" << endl;
  cout << "# SS-SNR: " + to_string(ss_pwr.ss_sinr) + " db" << endl;
  cout << "# Frequency offset: " + to_string(freq_offset) + " Hz" << endl;
  cout << "\n";
  cout << "###### CELL" << endl;
  cout << "## PCI: " + to_string(pci) << endl;
  cout << "## CP: ";
  cout << ((is_extended_cp == 0) ? "Normal" : "Extended") << endl;
  cout << "## I_SSB: " + to_string(i_ssb) << endl;
  cout << "\n";
  cout << "###### MIB" << endl;
  cout << "## Frame number: " + to_string(mib_object.sfn) << endl;
  cout << "## PDCCH configuration: " + to_string(mib_object.pdcch_config)
       << endl;
  cout << "## Subcarrier spacing common: " + to_string(mib_object.scs) << endl;
  cout << "## Cell barred: " + to_string(mib_object.cell_barred) << endl;
  cout << "## DMRS type A position : " +
              to_string(mib_object.dmrs_type_a_position)
       << endl;
  cout << "## k SSB: " + to_string(mib_object.k_ssb) << endl;
  cout << "## Intra freq reselection: " +
              to_string(mib_object.intra_freq_reselection)
       << endl;
  cout << "## CRC ";
  cout << ((mib_object.crc_validated) ? "validated" : "not validated") << endl;
  cout << "\n";
  cout << "####################################################################"
          "###"
       << endl;
  cout << "\n";
}

void phy::print_dci_info() {
  /**
   * \fn print_dci_info
   * \brief Print DCI decoded informations
   */
  cout << "###### DCI" << endl;
  cout << "# RIV: " + to_string(dci_1_0_si_rnti.RIV) << endl;
  cout << "# Time Domain RA: " + to_string(dci_1_0_si_rnti.TD_ra) << endl;
  cout << ((dci_1_0_si_rnti.vrb_prb_interleaving == 0)
               ? "# Non-interleaved VRB to PRB"
               : "# Interleaved VRB to PRB")
       << endl;
  cout << "# Modulation coding scheme: " + to_string(dci_1_0_si_rnti.mcs)
       << endl;
  cout << "# Redudancy version: " + to_string(dci_1_0_si_rnti.rv) << endl;
  cout << ((dci_1_0_si_rnti.si == 0) ? "# SIB1 message" : "# Other SIB message")
       << endl;
  cout << ((dci_1_0_si_rnti.crc_validated) ? "# CRC validated"
                                           : "# CRC not validated")
       << endl;
  cout << "####################################################################"
          "###"
       << endl;
  if (dci_1_0_si_rnti.rv == 1 || dci_1_0_si_rnti.rv == 2) {
    cout << "WARNING: Redudancy version " + to_string(dci_1_0_si_rnti.rv)
         << " is not supported by current decoder. To decode SIB1 data on this "
            "cell, please use CELL_SEARCH function in config and specify the "
            "cell frequency. Retry until redundancy version is not 1 or 2"
         << endl;
    cout << "##################################################################"
            "#####"
         << endl;
  }
  cout << "\n";
}

void phy::parse_dci_1_0_si_rnti(vector<int>& dci_bits,
                                int freq_domain_ra_size,
                                free5GRAN::dci_1_0_si_rnti& dci) {
  /**
   * \fn parse_dci_1_0_si_rnti
   * \brief Parse DCI informations
   * \standard TS 38.212 7.3.1.2.1
   * \param[in] dci_bits: DCI decoded bits
   * \param[in] freq_domain_ra_size: Number of bits used for frequency
   * allocation in DCI \param[out] dci: Filled DCI object
   */

  dci.RIV = 0;
  for (int i = 0; i < freq_domain_ra_size; i++) {
    dci.RIV += dci_bits[i] * pow(2, freq_domain_ra_size - i - 1);
  }
  dci.TD_ra = 0;
  for (int i = 0; i < 4; i++) {
    dci.TD_ra += dci_bits[i + freq_domain_ra_size] * pow(2, 4 - i - 1);
  }

  dci.vrb_prb_interleaving = dci_bits[freq_domain_ra_size + 4];

  dci.mcs = 0;
  for (int i = 0; i < 5; i++) {
    dci.mcs += dci_bits[i + freq_domain_ra_size + 4 + 1] * pow(2, 5 - i - 1);
  }

  dci.rv = 0;
  for (int i = 0; i < 2; i++) {
    dci.rv += dci_bits[i + freq_domain_ra_size + 4 + 1 + 5] * pow(2, 2 - i - 1);
  }

  dci.si = dci_bits[freq_domain_ra_size + 4 + 1 + 5 + 2];
}

auto phy::getSib() -> BCCH_DL_SCH_Message_t* {
  return this->sib1;
}

void phy::print_sib1() {
  asn_fprint(stdout, &asn_DEF_BCCH_DL_SCH_Message, sib1);
}

auto phy::getSIB1RV() -> int {
  return dci_1_0_si_rnti.rv;
}

auto phy::init(free5GRAN::synchronization_object& sync_object,
               condition_variable& cond_var_cell_sync) -> int {
  /**
   * \fn init
   * \brief Adjust primary buffer frames into gNodeB-synchronized frames
   * \details
   * Details:
   * - Get a period of signal
   * - Search SSB and synchronize with cell
   * - Decode PBCH and extract MIB
   * - Search PDCCH and extract DCI
   * - Decode PSDCH and extract SIB1
   *
   * \param[in] sync_object: Structure containing synchronization variables
   * \param[in] cond_var_cell_sync: Condition variable for notifying cell
   * synchronization
   */
  /*
   * Get 1 SSB period of signal
   */
  int number_of_frames = ceil(ssb_period / 0.01);  // = 10ms
  // Create a vector of frames containing 1 SSB period of signal
  vector<free5GRAN::buffer_element> frames_buffer(number_of_frames);
  // Compute the number of samples in a frame
  frame_size = 0.01 * rf_device->getSampleRate();
  // Create a vector containing all the frames for 1 SSB period (consecutively)
  vector<complex<float>> merged_frames(number_of_frames * frame_size);

  mutex m;
  // Loop over all the frames
  for (int i = 0; i < frames_buffer.size(); i++) {
    // Deactivate timeout switch
    bool timeout = false;
    // If the i-th frame is not yet in buffer
    if (rf_buff->primary_buffer->size() < i + 1) {
      unique_lock<mutex> lk(m);
      // Wait for it with a timeout of 2sec
      (*rf_buff->cond_var_vec_prim_buffer)[i].wait_for(lk,
                                                       std::chrono::seconds(2));
      lk.unlock();
    }
    // If the i-th frame is still not in buffer,
    // timeout occured
    if (rf_buff->primary_buffer->size() < i + 1) {
      // Activate timeout switch
      timeout = true;
    }
    // Get the i-th frame
    frames_buffer[i] = (*rf_buff->primary_buffer)[i];
    if (frames_buffer[i].overflow || timeout) {
      // If timeout or overflow in frame, end function witth failure
      cout << "OVERFLOW DETECTED IN PHY" << endl;
      sync_object.mib_crc_val = false;
      // Notify function ended
      cond_var_cell_sync.notify_all();
      return 1;
    }
    // Add the content of i-th frame to the merged_frames buffer
    for (int j = 0; j < frame_size; j++) {
      merged_frames[i * frame_size + j] = frames_buffer[i].buffer[j];
    }
  }
  // Write the content of merged_frames to a file
  /*
  ofstream data2;
  data2.open("merged_frames.txt");
  for (int i = 0; i < number_of_frames * frame_size; i++) {
    data2 << merged_frames[i];
    data2 << "\n";
  }
  data2.close();
   */

  int pss_start_index;
  float received_power;
  auto start = chrono::high_resolution_clock::now();
  // Search SSB, synchronize with cell and decode PBCH and BCH to extract MIB
  // merged_frames is the buffer containing one SSB period of signal
  // pss_start_index is the starting sample of PSS which provides time-domain
  // synchronization received_power is the SSB received power in dB, used for
  // gain up / down ramping
  free5GRAN::phy::signal_processing::synchronize_and_extract_pbch(
      merged_frames, pss_start_index, received_power, this->current_bwp,
      this->pci, this->freq_offset, this->rf_device->getSampleRate(),
      this->i_ssb, this->ss_pwr, this->mib_object, this->l_max);
  ss_pwr.received_power = received_power;
  // If received power greater than -2dB, then there might be saturation and
  // reception will fail
  if (received_power > -2) {
    sync_object.mib_crc_val = false;
    sync_object.received_power = received_power;
    cond_var_cell_sync.notify_all();
    return 1;
  }
  auto end = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(end - start);

  /*
  int num_symbols_per_subframe_pbch =
      free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * (int)(this -> current_bwp
  -> getScs() / 15e3); int cp_lengths_pbch[num_symbols_per_subframe_pbch]; int
  cum_sum_pbch[num_symbols_per_subframe_pbch];

  // Compute cyclic prefix length for PBCH
  free5GRAN::phy::signal_processing::compute_cp_lengths(
      (int)scs / 1e3, bwp -> getFftSize(), is_extended_cp,
  num_symbols_per_subframe_pbch, &cp_lengths_pbch[0], &cum_sum_pbch[0]);
      */

  // Extract the symbol in frame where SSB has been found based on i ssb
  int symbol_in_frame = band_object.ssb_pattern.ssb_start_symbols[this->i_ssb];
  // Compute the index of the first PSS sample in a radio frame based
  int num_samples_before_pss =
      (symbol_in_frame / free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP) *
          (15e3 / this->current_bwp->getScs() * frame_size / 10.0) +
      this->current_bwp
          ->getCumSumCpLengths()[symbol_in_frame %
                                 free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP] +
      mib_object.half_frame_index * frame_size * 0.5;

  // Compute radio frame starting point relatively to the primary frame buffer
  int point_0_index = pss_start_index - num_samples_before_pss;
  int frame_id_point_0;
  if (point_0_index < 0) {
    frame_id_point_0 = frames_buffer[0].frame_id - 1;
  } else {
    frame_id_point_0 = frames_buffer[point_0_index / frame_size].frame_id;
  }
  // Create a sync object
  sync_object = {.frame_id = frame_id_point_0,
                 .sfn = mib_object.sfn,
                 .sync_index = point_0_index % frame_size,
                 .pci = pci,
                 .pss_index = num_samples_before_pss,
                 .common_cp_length = this->current_bwp->getCpLengths()[1],
                 .fft_size_ssb = this->current_bwp->getFftSize(),
                 .mib_crc_val = mib_object.crc_validated,
                 .received_power = received_power,
                 .freq_offset = freq_offset,
                 .ssb_period = ssb_period};
  // Notify main thread that synchronization has been done
  cond_var_cell_sync.notify_all();
  if (!mib_object.crc_validated) {
    return 1;
  } else {
    print_cell_info();
  }

  // Compute mu, the numerology index for PDCCH/PDSCH
  // mu = log2(mib_object.scs / 15);
  // Compute the number of slots per frame for PDCCH/PDSCH
  // num_slots_per_frame = 10 * mib_object.scs / 15;
  // Recompute fft size for PDCCH/PDSCH
  // fft_size = (int)rf_device->getSampleRate() / (mib_object.scs * 1e3);

  current_bwp = new free5GRAN::phy::bwp(1, mib_object.scs * 1e3,
                                        rf_device->getSampleRate());
  free5GRAN::pdcch_t0ss_monitoring_occasions pdcch_ss_mon_occ = free5GRAN::phy::
      signal_processing::compute_pdcch_t0_ss_monitoring_occasions(
          mib_object.pdcch_config, available_bwps[0]->getScs(),
          this->current_bwp->getScs(), i_ssb);
  current_bwp->setNBwpSize(pdcch_ss_mon_occ.n_rb_coreset);
  this->available_bwps.push_back(this->current_bwp);

  free5GRAN::coreset coreset0 = {
      .controlResourceSetId = 0,
      .pdcch_DMRS_ScrambilngID = pci,
      .n_rb_coreset = pdcch_ss_mon_occ.n_rb_coreset,
      .duration = pdcch_ss_mon_occ.n_symb_coreset,
      .isMapped = true,
      .cce_REG_MappingType = {.reg_BundleSize = free5GRAN::NUMBER_REG_PER_CCE /
                                                pdcch_ss_mon_occ.n_symb_coreset,
                              .interleaverSize = 2,
                              .shiftIndex = pci}};
  int n0 = (int)(pdcch_ss_mon_occ.O * pow(2, this->current_bwp->getMu()) +
                 floor(i_ssb * pdcch_ss_mon_occ.M)) %
           this->current_bwp->getNumSlotsPerFrame();
  int sfn_parity =
      (int)((pdcch_ss_mon_occ.O * pow(2, this->current_bwp->getMu()) +
             floor(i_ssb * pdcch_ss_mon_occ.M)) /
            this->current_bwp->getNumSlotsPerFrame()) %
      2;

  BOOST_LOG_TRIVIAL(trace) << "## n0: " + to_string(n0);
  BOOST_LOG_TRIVIAL(trace) << "## ODD/EVEN ?: " + to_string(sfn_parity);

extract_dci_and_sib:
  /*
   * Get one SIB1 periodicity of signal (= 160ms = 16 frames);
   */
  vector<free5GRAN::buffer_element> frames_160;
  bool timeout_frames;
  rf_device->get_n_frame(frames_160, 16, timeout_frames);
  int index_last_frame;
  free5GRAN::buffer_element buff_elem;
  // If no timeout occured, compute the SFN parity where PDCCH and PDSCH can be
  // found
  if (!timeout_frames) {
    if (frames_160[0].frame_id % 2 == sfn_parity) {
      index_last_frame = 0;
    } else {
      index_last_frame = 1;
    }
  }
  bool dci_found = false;
  // Loop over all the frame with SFN parity inside a SIB1 period
  while (!dci_found && index_last_frame < 16 && !timeout_frames) {
    // Get a frame
    buff_elem = frames_160[index_last_frame];
    std::vector<std::complex<float>> frame_data = buff_elem.buffer;

    /*
     * Computing frequency offset of Initial BWP from SSB, based on RB offset
     * and k_ssb and transposing signal to center on current BWP (which is here
     * CORESET0) freq_diff represents the frequency offset between the SSB.
     * available_bwps[0] is the BWP used for SSB transmission
     */
    float freq_diff = 12 * 1e3 * mib_object.scs *
                      (pdcch_ss_mon_occ.n_rb_coreset / 2 -
         (10 * (available_bwps[0]->getScs() / (1e3 * mib_object.scs)) +
          pdcch_ss_mon_occ.offset));
    // freq_diff_k_ssb represents the k ssb offset which is the difference (in
    // 15kHz subcarriers) between the subcarrier 0 of the SSB and the subcarrier
    // 0 of the Common RB where the SSB is starting
    float freq_diff_k_ssb = -15e3 * mib_object.k_ssb;
    free5GRAN::phy::signal_processing::transpose_signal(
        &frame_data, freq_diff + freq_diff_k_ssb, rf_device->getSampleRate(),
        frame_size);

    BOOST_LOG_TRIVIAL(trace) << "## FREQ DIFF 1: " + to_string(freq_diff);
    BOOST_LOG_TRIVIAL(trace)
        << "## FREQ DIFF K SSB: " + to_string(freq_diff_k_ssb);

    /*
     * Logging frame data to text file for plotting
     */
    /*
    ofstream data;
    data.open("frame_buff_data.txt");
    for (int i = 0; i < frame_size; i ++){
      data << frame_data[i];
      data << "\n";
    }
    data.close();
    */

    /*
     * Plotting 4 slots around PDCCH monitoring slots
     */
    /*
    int begin_index =
        (n0 - 1) * frame_size / this -> current_bwp -> getNumSlotsPerFrame();
    begin_index = max(begin_index, 0);

    ofstream data2;
    data2.open("moniroting_slots.txt");
    for (int i = 0; i < 4 * frame_size / this -> current_bwp ->
    getNumSlotsPerFrame(); i++) { data2 << frame_data[i + begin_index]; data2 <<
    "\n";
    }
    data2.close();
    */

    int monitoring_slot, freq_domain_ra_size;
    vector<int> dci_decoded_bits;
    bool validated;
    // Search PDCCH and DCI for SIB1
    free5GRAN::phy::signal_processing::blind_search_pdcch(
        validated, frame_data, this->current_bwp, coreset0, n0,
        pdcch_ss_mon_occ.first_symb_index, monitoring_slot, dci_decoded_bits,
        freq_domain_ra_size, frame_size);

    if (validated) {
      parse_dci_1_0_si_rnti(dci_decoded_bits, freq_domain_ra_size,
                            dci_1_0_si_rnti);
      if (dci_1_0_si_rnti.rv == 0 || dci_1_0_si_rnti.rv == 3) {
        dci_found = true;
      }
      dci_1_0_si_rnti.crc_validated = validated;
      if (dci_found) {
        print_dci_info();
        // Extract SIB1
        // extract_pdsch(frame_data, this->current_bwp, n0 + monitoring_slot);
        vector<int> pdsch_bits;
        bool pdsch_validated;
        free5GRAN::phy::signal_processing::extract_pdsch(
            frame_data, this->current_bwp, n0 + monitoring_slot, pdsch_bits,
            mib_object, pdsch_validated, dci_1_0_si_rnti, frame_size,
            rf_device->getSampleRate(), pci);
        if (pdsch_validated) {
          int bytes_size = (int)ceil(pdsch_bits.size() / 8.0);
          uint8_t dl_sch_bytes[bytes_size];
          for (int i = 0; i < pdsch_bits.size(); i++) {
            if (i % 8 == 0) {
              dl_sch_bytes[i / 8] = 0;
            }
            dl_sch_bytes[i / 8] += pdsch_bits[i] * pow(2, 8 - (i % 8) - 1);
          }
          asn_dec_rval_t dec_rval = asn_decode(
              nullptr, ATS_UNALIGNED_BASIC_PER, &asn_DEF_BCCH_DL_SCH_Message,
              (void**)&sib1, dl_sch_bytes, bytes_size);
          if (dec_rval.code == RC_OK) {
            BOOST_LOG_TRIVIAL(trace) << "SIB1 parsing succeeded";
          } else {
            BOOST_LOG_TRIVIAL(trace) << "SIB1 parsing failed";
          }
          print_sib1();
        }
      }
    }
    // Increment by two to focus on frames with the right SFN parity
    index_last_frame += 2;
  }
  // If dci successfully decoded
  if (!dci_found) {
    cout << "SIB1 data could not be decoded ! Waiting 2 sec and trying again"
         << endl;
    // If failed, wait 2 sec and retry DCI and SIB1 extraction
    if (!*stop_signal) {
      usleep(2000000);
      goto extract_dci_and_sib;
    }
  }
  return 0;
}

phy::~phy() {
  for (auto bwp : available_bwps) {
    delete bwp;
  }
}
