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

#include <string>

#ifndef FREE5GRAN_COMMON_STRUCTURES_H
#define FREE5GRAN_COMMON_STRUCTURES_H



namespace free5GRAN{
    /*
     * Define structure to store NR bands information
     */
    typedef struct band_ {
        int number, scs, min_gscn, max_gscn, l_max, *ssb_symbols;
    } band;

    /*
     * From TS38.331 6.2.2
     * cellBarred: barred means the cell is barred, as defined in TS 38.304 [20].
     * dmrs-TypeA-Position: Position of (first) DM-RS for downlink (see 38.211, section 7.4.1.1.1) and uplink (see 38.211, section 6.4.1.1.3).
     * intraFreqReselection: Controls cell selection/reselection to intra-frequency cells when the highest ranked cell is barred, or treated as barred by the UE, as specified in TS 38.304 [20}.
     * pdcch-ConfigSIB1: See TS 38.213 [13]. Determines a common ControlResourceSet (CORESET) a common search space and necessary PDCCH parameters. If the field ssb-SubcarrierOffset indicates that SIB1 is not present, the field pdcch-ConfigSIB1 indicate the frequency positions where the UE may find SS/PBCH block with SIB1 or the frequency range where the network does not provide SS/PBCH block with SIB1 (see TS 38.213 [13], section 13).
     * ssb-SubcarrierOffset: Corresponds to kSSB (see TS 38.213 [13]), which is the frequency domain offset between SSB and the overall resource block grid in number of subcarriers. (See 38.211).
     * The value range of this field may be extended by an additional most significant bit encoded within PBCH as specified in 38.213 [13].
     * This field may indicate that this beam does not provide SIB1 and that there is hence no common CORESET (see TS 38.213 [13], section 13). In this case, the field pdcch- ConfigSIB1 may indicate the frequency positions where the UE may (not) find a SS/PBCH with a control resource set and search space for SIB1 (see 38.213 [13], section 13).
     * subCarrierSpacingCommon: Subcarrier spacing for SIB1, Msg.2/4 for initial access and broadcast SI-messages. If the UE acquires this MIB on a carrier frequency <6GHz, the value scs15or60 corresponds to 15 Khz and the value scs30or120 corresponds to 30 kHz. If the UE acquires this MIB on a carrier frequency >6GHz, the value scs15or60 corresponds to 60 Khz and the value scs30or120 corresponds to 120 kHz.
     * systemFrameNumber: The 6 most significant bit (MSB) of the 10-bit System Frame Number. The 4 LSB of the SFN are conveyed in the PBCH transport block as part of transport_channel coding (i.e. outside the MIB encoding).
     */
    typedef struct mib_ {
        int sfn, pdcch_config, scs, cell_barred, dmrs_type_a_position, k_ssb, intra_freq_reselection;
    } mib;

    /*
     * DCI Format 1_0 with SI-RNTI
     */
    typedef struct dci_1_0_si_rnti_ {
        int RIV, TD_ra, vrb_prb_interleaving, mcs, rv, si;
    } dci_1_0_si_rnti;

    typedef struct bandwidth_info_ {
        int scs;
    } bandwidth_info;

    /*
     * Defining structure for storing Type0-PDCCH search space config & monitoring occasions (TS 38.213 13)
     */
    typedef struct pdcch_t0ss_monitoring_occasions_ {
        int multiplexing_pattern, n_rb_coreset, n_symb_coreset, offset, O, num_ss_slots, first_symb_index;
        float M;
        int n0, sfn_parity, monitoring_slot;
    } pdcch_t0ss_monitoring_occasions;


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
    extern bandwidth_info  BANDWIDTH_15_KHZ;

    extern bandwidth_info  BANDWIDTH_30_KHZ;

}

#endif