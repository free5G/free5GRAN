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

#include <semaphore.h>

#include <fftw3.h>
#include <boost/circular_buffer.hpp>
#include <complex>
#include <condition_variable>
#include <string>
#include <vector>

#ifndef FREE5GRAN_COMMON_STRUCTURES_H
#define FREE5GRAN_COMMON_STRUCTURES_H

using namespace std;

namespace free5GRAN {

using ssb_pattern_t = struct ssb_pattern_ {
  int scs, l_max;
  vector<int> ssb_start_symbols;
};
/*
 * Define structure to store NR bands information
 */
using band = struct band_ {
  int number, min_gscn, max_gscn;
  ssb_pattern_t ssb_pattern;
};

/*
 * From TS38.331 6.2.2
 * cellBarred: barred means the cell is barred, as defined in TS 38.304 [20].
 * dmrs-TypeA-Position: Position of (first) DM-RS for downlink (see 38.211,
 * section 7.4.1.1.1) and uplink (see 38.211, section 6.4.1.1.3).
 * intraFreqReselection: Controls cell selection/reselection to intra-frequency
 * cells when the highest ranked cell is barred, or treated as barred by the UE,
 * as specified in TS 38.304 [20}. pdcch-ConfigSIB1: See TS 38.213 [13].
 * Determines a common ControlResourceSet (CORESET) a common search space and
 * necessary PDCCH parameters. If the field ssb-SubcarrierOffset indicates that
 * SIB1 is not present, the field pdcch-ConfigSIB1 indicate the frequency
 * positions where the UE may find SS/PBCH block with SIB1 or the frequency
 * range where the network does not provide SS/PBCH block with SIB1 (see
 * TS 38.213 [13], section 13). ssb-SubcarrierOffset: Corresponds to kSSB (see
 * TS 38.213 [13]), which is the frequency domain offset between SSB and the
 * overall resource block grid in number of subcarriers. (See 38.211). The value
 * range of this field may be extended by an additional most significant bit
 * encoded within PBCH as specified in 38.213 [13]. This field may indicate that
 * this beam does not provide SIB1 and that there is hence no common CORESET
 * (see TS 38.213 [13], section 13). In this case, the field pdcch- ConfigSIB1
 * may indicate the frequency positions where the UE may (not) find a SS/PBCH
 * with a control resource set and search space for SIB1 (see 38.213 [13],
 * section 13). subCarrierSpacingCommon: Subcarrier spacing for SIB1, Msg.2/4
 * for initial access and broadcast SI-messages. If the UE acquires this MIB on
 * a carrier frequency <6GHz, the value scs15or60 corresponds to 15 Khz and the
 * value scs30or120 corresponds to 30 kHz. If the UE acquires this MIB on a
 * carrier frequency >6GHz, the value scs15or60 corresponds to 60 Khz and the
 * value scs30or120 corresponds to 120 kHz. systemFrameNumber: The 6 most
 * significant bit (MSB) of the 10-bit System Frame Number. The 4 LSB of the SFN
 * are conveyed in the PBCH transport block as part of transport_channel coding
 * (i.e. outside the MIB encoding).
 */
using mib = struct mib_ {
  int sfn, pdcch_config, scs, cell_barred, dmrs_type_a_position, k_ssb,
      intra_freq_reselection, half_frame_index;
  bool crc_validated;
};

/*
 * DCI Format 1_0 with SI-RNTI
 */
using dci_1_0_si_rnti = struct dci_1_0_si_rnti_ {
  int RIV, TD_ra, vrb_prb_interleaving, mcs, rv, si;
  bool crc_validated;
};

using bandwidth_info = struct bandwidth_info_ { int scs; };

/*
 * Defining structure for storing Type0-PDCCH search space config & monitoring
 * occasions (TS 38.213 13)
 */
using pdcch_t0ss_monitoring_occasions =
    struct pdcch_t0ss_monitoring_occasions_ {
  int multiplexing_pattern, n_rb_coreset, n_symb_coreset, offset, O,
      num_ss_slots, first_symb_index;
  float M;
  int n0, sfn_parity, monitoring_slot;
};

using pdcch_type0_search_space = struct pdcch_type0_search_space_ {
  float M;
  int O, num_ss_slots, first_symb_index, n0;
};

using coreset = struct coreset_ {
  int controlResourceSetId, precoderGranularity, pdcch_DMRS_ScrambilngID,
      n_rb_coreset, duration;
  vector<int> frequencyDomainResources;
  bool isMapped;
  struct cce_REG_MappingType_t {
    int reg_BundleSize, interleaverSize, shiftIndex;
  };
  cce_REG_MappingType_t cce_REG_MappingType;
};

using ss_power_indicator = struct ss_power_indicator_ {
  float ss_rsrp, ss_rssi, ss_rsrq, ss_sinr, received_power;
};

using rf_device = struct rf_device_ { string type, serial, subdev, ref; };

using buffer_element = struct buffer_element_ {
  size_t frame_id;
  vector<complex<float>> buffer;
  bool overflow;
};

using rf_buffer = struct rf_buffer_ {
  boost::circular_buffer<buffer_element>*primary_buffer, *frame_buffer;
  vector<condition_variable>*cond_var_vec_prim_buffer,
      *cond_var_vec_frame_buffer;
  sem_t* semaphore;
  bool frame_thread_started;
};

using synchronization_object = struct synchronization_object_ {
  int frame_id, sfn, sync_index, pci, pss_index, common_cp_length, fft_size_ssb;
  bool mib_crc_val;
  float received_power, freq_offset;
  double ssb_period;
  sem_t* cont_sync_sem;
};

using fft_t = struct fft_ {
  int fft_size;
  fftw_complex *fft_in, *fft_out;
  fftw_plan fft_plan;
  ~fft_() {
    fftw_destroy_plan(fft_plan);
    fftw_free(fft_in);
    fftw_free(fft_out);
  }
};

using bwp = struct bwp_ {
  int id, scs, n_bwp_size, mu, num_slots_per_frame, num_symbols_per_subframe;
  vector<int> cp_lengths, cum_sum_cp_lengths;
  fft_t fft;
};

// Define Global variables with previously-defined types
/*
 * 5G NR bands
 */
extern band BAND_N_78;
extern band BAND_N_7;
extern band BAND_N_8;
extern band BAND_N_1;
extern band BAND_N_2;
extern band BAND_N_3;
extern band BAND_N_28;

/*
 * Bandwidth parameters for PSS and SSB
 */
extern bandwidth_info BANDWIDTH_15_KHZ;

extern bandwidth_info BANDWIDTH_30_KHZ;

extern ssb_pattern_t SSB_PATTERN_CASE_A_FR1_INF_3_GHZ;
extern ssb_pattern_t SSB_PATTERN_CASE_A_FR1_SUP_3_GHZ;
extern ssb_pattern_t SSB_PATTERN_CASE_B_FR1_INF_3_GHZ;
extern ssb_pattern_t SSB_PATTERN_CASE_B_FR1_SUP_3_GHZ;
extern ssb_pattern_t SSB_PATTERN_CASE_C_FR1_INF_3_GHZ;
extern ssb_pattern_t SSB_PATTERN_CASE_C_FR1_SUP_3_GHZ;

}  // namespace free5GRAN

#endif