
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */



#ifndef _RX_MPDU_INFO_H_
#define _RX_MPDU_INFO_H_
#if !defined(__ASSEMBLER__)
#endif

#include "rxpt_classify_info.h"

#define NUM_OF_DWORDS_RX_MPDU_INFO 23

struct rx_mpdu_info {
    struct            rxpt_classify_info                       rxpt_classify_info_details;
             uint32_t rx_reo_queue_desc_addr_31_0     : 32;
             uint32_t rx_reo_queue_desc_addr_39_32    :  8,
                      receive_queue_number            : 16,
                      pre_delim_err_warning           :  1,
                      first_delim_err                 :  1,
                      reserved_2a                     :  6;
             uint32_t pn_31_0                         : 32;
             uint32_t pn_63_32                        : 32;
             uint32_t pn_95_64                        : 32;
             uint32_t pn_127_96                       : 32;
             uint32_t epd_en                          :  1,
                      all_frames_shall_be_encrypted   :  1,
                      encrypt_type                    :  4,
                      wep_key_width_for_variable_key  :  2,
                      __reserved_g_0003                        :  2,
                      bssid_hit                       :  1,
                      bssid_number                    :  4,
                      tid                             :  4,
                      reserved_7a                     : 13;
             uint32_t peer_meta_data                  : 32;
             uint32_t rxpcu_mpdu_filter_in_category   :  2,
                      sw_frame_group_id               :  7,
                      ndp_frame                       :  1,
                      phy_err                         :  1,
                      phy_err_during_mpdu_header      :  1,
                      protocol_version_err            :  1,
                      ast_based_lookup_valid          :  1,
                      reserved_9a                     :  2,
                      phy_ppdu_id                     : 16;
             uint32_t ast_index                       : 16,
                      sw_peer_id                      : 16;
             uint32_t mpdu_frame_control_valid        :  1,
                      mpdu_duration_valid             :  1,
                      mac_addr_ad1_valid              :  1,
                      mac_addr_ad2_valid              :  1,
                      mac_addr_ad3_valid              :  1,
                      mac_addr_ad4_valid              :  1,
                      mpdu_sequence_control_valid     :  1,
                      mpdu_qos_control_valid          :  1,
                      mpdu_ht_control_valid           :  1,
                      frame_encryption_info_valid     :  1,
                      mpdu_fragment_number            :  4,
                      more_fragment_flag              :  1,
                      reserved_11a                    :  1,
                      fr_ds                           :  1,
                      to_ds                           :  1,
                      encrypted                       :  1,
                      mpdu_retry                      :  1,
                      mpdu_sequence_number            : 12;
             uint32_t key_id_octet                    :  8,
                      new_peer_entry                  :  1,
                      decrypt_needed                  :  1,
                      decap_type                      :  2,
                      rx_insert_vlan_c_tag_padding    :  1,
                      rx_insert_vlan_s_tag_padding    :  1,
                      strip_vlan_c_tag_decap          :  1,
                      strip_vlan_s_tag_decap          :  1,
                      pre_delim_count                 : 12,
                      ampdu_flag                      :  1,
                      bar_frame                       :  1,
                      raw_mpdu                        :  1,
                      reserved_12                     :  1;
             uint32_t mpdu_length                     : 14,
                      first_mpdu                      :  1,
                      mcast_bcast                     :  1,
                      ast_index_not_found             :  1,
                      ast_index_timeout               :  1,
                      power_mgmt                      :  1,
                      non_qos                         :  1,
                      null_data                       :  1,
                      mgmt_type                       :  1,
                      ctrl_type                       :  1,
                      more_data                       :  1,
                      eosp                            :  1,
                      fragment_flag                   :  1,
                      order                           :  1,
                      u_apsd_trigger                  :  1,
                      encrypt_required                :  1,
                      directed                        :  1,
                      amsdu_present                   :  1,
                      reserved_13                     :  1;
             uint32_t mpdu_frame_control_field        : 16,
                      mpdu_duration_field             : 16;
             uint32_t mac_addr_ad1_31_0               : 32;
             uint32_t mac_addr_ad1_47_32              : 16,
                      mac_addr_ad2_15_0               : 16;
             uint32_t mac_addr_ad2_47_16              : 32;
             uint32_t mac_addr_ad3_31_0               : 32;
             uint32_t mac_addr_ad3_47_32              : 16,
                      mpdu_sequence_control_field     : 16;
             uint32_t mac_addr_ad4_31_0               : 32;
             uint32_t mac_addr_ad4_47_32              : 16,
                      mpdu_qos_control_field          : 16;
             uint32_t mpdu_ht_control_field           : 32;
};

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_REO_DESTINATION_INDICATION_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_REO_DESTINATION_INDICATION_LSB 0
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_REO_DESTINATION_INDICATION_MASK 0x0000001f

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_LMAC_PEER_ID_MSB_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_LMAC_PEER_ID_MSB_LSB 5
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_LMAC_PEER_ID_MSB_MASK 0x00000060

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_USE_FLOW_ID_TOEPLITZ_CLFY_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_USE_FLOW_ID_TOEPLITZ_CLFY_LSB 7
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_USE_FLOW_ID_TOEPLITZ_CLFY_MASK 0x00000080

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_UCAST_DATA_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_UCAST_DATA_LSB 8
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_UCAST_DATA_MASK 0x00000100

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_MCAST_DATA_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_MCAST_DATA_LSB 9
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_MCAST_DATA_MASK 0x00000200

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_1000_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_1000_LSB 10
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_PKT_SELECTION_FP_1000_MASK 0x00000400

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RXDMA0_SOURCE_RING_SELECTION_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RXDMA0_SOURCE_RING_SELECTION_LSB 11
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RXDMA0_SOURCE_RING_SELECTION_MASK 0x00001800

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RXDMA0_DESTINATION_RING_SELECTION_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RXDMA0_DESTINATION_RING_SELECTION_LSB 13
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RXDMA0_DESTINATION_RING_SELECTION_MASK 0x00006000

#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RESERVED_0B_OFFSET 0x00000000
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RESERVED_0B_LSB    15
#define RX_MPDU_INFO_0_RXPT_CLASSIFY_INFO_DETAILS_RESERVED_0B_MASK   0xffff8000

#define RX_MPDU_INFO_1_RX_REO_QUEUE_DESC_ADDR_31_0_OFFSET            0x00000004
#define RX_MPDU_INFO_1_RX_REO_QUEUE_DESC_ADDR_31_0_LSB               0
#define RX_MPDU_INFO_1_RX_REO_QUEUE_DESC_ADDR_31_0_MASK              0xffffffff

#define RX_MPDU_INFO_2_RX_REO_QUEUE_DESC_ADDR_39_32_OFFSET           0x00000008
#define RX_MPDU_INFO_2_RX_REO_QUEUE_DESC_ADDR_39_32_LSB              0
#define RX_MPDU_INFO_2_RX_REO_QUEUE_DESC_ADDR_39_32_MASK             0x000000ff

#define RX_MPDU_INFO_2_RECEIVE_QUEUE_NUMBER_OFFSET                   0x00000008
#define RX_MPDU_INFO_2_RECEIVE_QUEUE_NUMBER_LSB                      8
#define RX_MPDU_INFO_2_RECEIVE_QUEUE_NUMBER_MASK                     0x00ffff00

#define RX_MPDU_INFO_2_PRE_DELIM_ERR_WARNING_OFFSET                  0x00000008
#define RX_MPDU_INFO_2_PRE_DELIM_ERR_WARNING_LSB                     24
#define RX_MPDU_INFO_2_PRE_DELIM_ERR_WARNING_MASK                    0x01000000

#define RX_MPDU_INFO_2_FIRST_DELIM_ERR_OFFSET                        0x00000008
#define RX_MPDU_INFO_2_FIRST_DELIM_ERR_LSB                           25
#define RX_MPDU_INFO_2_FIRST_DELIM_ERR_MASK                          0x02000000

#define RX_MPDU_INFO_2_RESERVED_2A_OFFSET                            0x00000008
#define RX_MPDU_INFO_2_RESERVED_2A_LSB                               26
#define RX_MPDU_INFO_2_RESERVED_2A_MASK                              0xfc000000

#define RX_MPDU_INFO_3_PN_31_0_OFFSET                                0x0000000c
#define RX_MPDU_INFO_3_PN_31_0_LSB                                   0
#define RX_MPDU_INFO_3_PN_31_0_MASK                                  0xffffffff

#define RX_MPDU_INFO_4_PN_63_32_OFFSET                               0x00000010
#define RX_MPDU_INFO_4_PN_63_32_LSB                                  0
#define RX_MPDU_INFO_4_PN_63_32_MASK                                 0xffffffff

#define RX_MPDU_INFO_5_PN_95_64_OFFSET                               0x00000014
#define RX_MPDU_INFO_5_PN_95_64_LSB                                  0
#define RX_MPDU_INFO_5_PN_95_64_MASK                                 0xffffffff

#define RX_MPDU_INFO_6_PN_127_96_OFFSET                              0x00000018
#define RX_MPDU_INFO_6_PN_127_96_LSB                                 0
#define RX_MPDU_INFO_6_PN_127_96_MASK                                0xffffffff

#define RX_MPDU_INFO_7_EPD_EN_OFFSET                                 0x0000001c
#define RX_MPDU_INFO_7_EPD_EN_LSB                                    0
#define RX_MPDU_INFO_7_EPD_EN_MASK                                   0x00000001

#define RX_MPDU_INFO_7_ALL_FRAMES_SHALL_BE_ENCRYPTED_OFFSET          0x0000001c
#define RX_MPDU_INFO_7_ALL_FRAMES_SHALL_BE_ENCRYPTED_LSB             1
#define RX_MPDU_INFO_7_ALL_FRAMES_SHALL_BE_ENCRYPTED_MASK            0x00000002

#define RX_MPDU_INFO_7_ENCRYPT_TYPE_OFFSET                           0x0000001c
#define RX_MPDU_INFO_7_ENCRYPT_TYPE_LSB                              2
#define RX_MPDU_INFO_7_ENCRYPT_TYPE_MASK                             0x0000003c

#define RX_MPDU_INFO_7_WEP_KEY_WIDTH_FOR_VARIABLE_KEY_OFFSET         0x0000001c
#define RX_MPDU_INFO_7_WEP_KEY_WIDTH_FOR_VARIABLE_KEY_LSB            6
#define RX_MPDU_INFO_7_WEP_KEY_WIDTH_FOR_VARIABLE_KEY_MASK           0x000000c0

#define RX_MPDU_INFO_7_BSSID_HIT_OFFSET                              0x0000001c
#define RX_MPDU_INFO_7_BSSID_HIT_LSB                                 10
#define RX_MPDU_INFO_7_BSSID_HIT_MASK                                0x00000400

#define RX_MPDU_INFO_7_BSSID_NUMBER_OFFSET                           0x0000001c
#define RX_MPDU_INFO_7_BSSID_NUMBER_LSB                              11
#define RX_MPDU_INFO_7_BSSID_NUMBER_MASK                             0x00007800

#define RX_MPDU_INFO_7_TID_OFFSET                                    0x0000001c
#define RX_MPDU_INFO_7_TID_LSB                                       15
#define RX_MPDU_INFO_7_TID_MASK                                      0x00078000

#define RX_MPDU_INFO_7_RESERVED_7A_OFFSET                            0x0000001c
#define RX_MPDU_INFO_7_RESERVED_7A_LSB                               19
#define RX_MPDU_INFO_7_RESERVED_7A_MASK                              0xfff80000

#define RX_MPDU_INFO_8_PEER_META_DATA_OFFSET                         0x00000020
#define RX_MPDU_INFO_8_PEER_META_DATA_LSB                            0
#define RX_MPDU_INFO_8_PEER_META_DATA_MASK                           0xffffffff

#define RX_MPDU_INFO_9_RXPCU_MPDU_FILTER_IN_CATEGORY_OFFSET          0x00000024
#define RX_MPDU_INFO_9_RXPCU_MPDU_FILTER_IN_CATEGORY_LSB             0
#define RX_MPDU_INFO_9_RXPCU_MPDU_FILTER_IN_CATEGORY_MASK            0x00000003

#define RX_MPDU_INFO_9_SW_FRAME_GROUP_ID_OFFSET                      0x00000024
#define RX_MPDU_INFO_9_SW_FRAME_GROUP_ID_LSB                         2
#define RX_MPDU_INFO_9_SW_FRAME_GROUP_ID_MASK                        0x000001fc

#define RX_MPDU_INFO_9_NDP_FRAME_OFFSET                              0x00000024
#define RX_MPDU_INFO_9_NDP_FRAME_LSB                                 9
#define RX_MPDU_INFO_9_NDP_FRAME_MASK                                0x00000200

#define RX_MPDU_INFO_9_PHY_ERR_OFFSET                                0x00000024
#define RX_MPDU_INFO_9_PHY_ERR_LSB                                   10
#define RX_MPDU_INFO_9_PHY_ERR_MASK                                  0x00000400

#define RX_MPDU_INFO_9_PHY_ERR_DURING_MPDU_HEADER_OFFSET             0x00000024
#define RX_MPDU_INFO_9_PHY_ERR_DURING_MPDU_HEADER_LSB                11
#define RX_MPDU_INFO_9_PHY_ERR_DURING_MPDU_HEADER_MASK               0x00000800

#define RX_MPDU_INFO_9_PROTOCOL_VERSION_ERR_OFFSET                   0x00000024
#define RX_MPDU_INFO_9_PROTOCOL_VERSION_ERR_LSB                      12
#define RX_MPDU_INFO_9_PROTOCOL_VERSION_ERR_MASK                     0x00001000

#define RX_MPDU_INFO_9_AST_BASED_LOOKUP_VALID_OFFSET                 0x00000024
#define RX_MPDU_INFO_9_AST_BASED_LOOKUP_VALID_LSB                    13
#define RX_MPDU_INFO_9_AST_BASED_LOOKUP_VALID_MASK                   0x00002000

#define RX_MPDU_INFO_9_RESERVED_9A_OFFSET                            0x00000024
#define RX_MPDU_INFO_9_RESERVED_9A_LSB                               14
#define RX_MPDU_INFO_9_RESERVED_9A_MASK                              0x0000c000

#define RX_MPDU_INFO_9_PHY_PPDU_ID_OFFSET                            0x00000024
#define RX_MPDU_INFO_9_PHY_PPDU_ID_LSB                               16
#define RX_MPDU_INFO_9_PHY_PPDU_ID_MASK                              0xffff0000

#define RX_MPDU_INFO_10_AST_INDEX_OFFSET                             0x00000028
#define RX_MPDU_INFO_10_AST_INDEX_LSB                                0
#define RX_MPDU_INFO_10_AST_INDEX_MASK                               0x0000ffff

#define RX_MPDU_INFO_10_SW_PEER_ID_OFFSET                            0x00000028
#define RX_MPDU_INFO_10_SW_PEER_ID_LSB                               16
#define RX_MPDU_INFO_10_SW_PEER_ID_MASK                              0xffff0000

#define RX_MPDU_INFO_11_MPDU_FRAME_CONTROL_VALID_OFFSET              0x0000002c
#define RX_MPDU_INFO_11_MPDU_FRAME_CONTROL_VALID_LSB                 0
#define RX_MPDU_INFO_11_MPDU_FRAME_CONTROL_VALID_MASK                0x00000001

#define RX_MPDU_INFO_11_MPDU_DURATION_VALID_OFFSET                   0x0000002c
#define RX_MPDU_INFO_11_MPDU_DURATION_VALID_LSB                      1
#define RX_MPDU_INFO_11_MPDU_DURATION_VALID_MASK                     0x00000002

#define RX_MPDU_INFO_11_MAC_ADDR_AD1_VALID_OFFSET                    0x0000002c
#define RX_MPDU_INFO_11_MAC_ADDR_AD1_VALID_LSB                       2
#define RX_MPDU_INFO_11_MAC_ADDR_AD1_VALID_MASK                      0x00000004

#define RX_MPDU_INFO_11_MAC_ADDR_AD2_VALID_OFFSET                    0x0000002c
#define RX_MPDU_INFO_11_MAC_ADDR_AD2_VALID_LSB                       3
#define RX_MPDU_INFO_11_MAC_ADDR_AD2_VALID_MASK                      0x00000008

#define RX_MPDU_INFO_11_MAC_ADDR_AD3_VALID_OFFSET                    0x0000002c
#define RX_MPDU_INFO_11_MAC_ADDR_AD3_VALID_LSB                       4
#define RX_MPDU_INFO_11_MAC_ADDR_AD3_VALID_MASK                      0x00000010

#define RX_MPDU_INFO_11_MAC_ADDR_AD4_VALID_OFFSET                    0x0000002c
#define RX_MPDU_INFO_11_MAC_ADDR_AD4_VALID_LSB                       5
#define RX_MPDU_INFO_11_MAC_ADDR_AD4_VALID_MASK                      0x00000020

#define RX_MPDU_INFO_11_MPDU_SEQUENCE_CONTROL_VALID_OFFSET           0x0000002c
#define RX_MPDU_INFO_11_MPDU_SEQUENCE_CONTROL_VALID_LSB              6
#define RX_MPDU_INFO_11_MPDU_SEQUENCE_CONTROL_VALID_MASK             0x00000040

#define RX_MPDU_INFO_11_MPDU_QOS_CONTROL_VALID_OFFSET                0x0000002c
#define RX_MPDU_INFO_11_MPDU_QOS_CONTROL_VALID_LSB                   7
#define RX_MPDU_INFO_11_MPDU_QOS_CONTROL_VALID_MASK                  0x00000080

#define RX_MPDU_INFO_11_MPDU_HT_CONTROL_VALID_OFFSET                 0x0000002c
#define RX_MPDU_INFO_11_MPDU_HT_CONTROL_VALID_LSB                    8
#define RX_MPDU_INFO_11_MPDU_HT_CONTROL_VALID_MASK                   0x00000100

#define RX_MPDU_INFO_11_FRAME_ENCRYPTION_INFO_VALID_OFFSET           0x0000002c
#define RX_MPDU_INFO_11_FRAME_ENCRYPTION_INFO_VALID_LSB              9
#define RX_MPDU_INFO_11_FRAME_ENCRYPTION_INFO_VALID_MASK             0x00000200

#define RX_MPDU_INFO_11_MPDU_FRAGMENT_NUMBER_OFFSET                  0x0000002c
#define RX_MPDU_INFO_11_MPDU_FRAGMENT_NUMBER_LSB                     10
#define RX_MPDU_INFO_11_MPDU_FRAGMENT_NUMBER_MASK                    0x00003c00

#define RX_MPDU_INFO_11_MORE_FRAGMENT_FLAG_OFFSET                    0x0000002c
#define RX_MPDU_INFO_11_MORE_FRAGMENT_FLAG_LSB                       14
#define RX_MPDU_INFO_11_MORE_FRAGMENT_FLAG_MASK                      0x00004000

#define RX_MPDU_INFO_11_RESERVED_11A_OFFSET                          0x0000002c
#define RX_MPDU_INFO_11_RESERVED_11A_LSB                             15
#define RX_MPDU_INFO_11_RESERVED_11A_MASK                            0x00008000

#define RX_MPDU_INFO_11_FR_DS_OFFSET                                 0x0000002c
#define RX_MPDU_INFO_11_FR_DS_LSB                                    16
#define RX_MPDU_INFO_11_FR_DS_MASK                                   0x00010000

#define RX_MPDU_INFO_11_TO_DS_OFFSET                                 0x0000002c
#define RX_MPDU_INFO_11_TO_DS_LSB                                    17
#define RX_MPDU_INFO_11_TO_DS_MASK                                   0x00020000

#define RX_MPDU_INFO_11_ENCRYPTED_OFFSET                             0x0000002c
#define RX_MPDU_INFO_11_ENCRYPTED_LSB                                18
#define RX_MPDU_INFO_11_ENCRYPTED_MASK                               0x00040000

#define RX_MPDU_INFO_11_MPDU_RETRY_OFFSET                            0x0000002c
#define RX_MPDU_INFO_11_MPDU_RETRY_LSB                               19
#define RX_MPDU_INFO_11_MPDU_RETRY_MASK                              0x00080000

#define RX_MPDU_INFO_11_MPDU_SEQUENCE_NUMBER_OFFSET                  0x0000002c
#define RX_MPDU_INFO_11_MPDU_SEQUENCE_NUMBER_LSB                     20
#define RX_MPDU_INFO_11_MPDU_SEQUENCE_NUMBER_MASK                    0xfff00000

#define RX_MPDU_INFO_12_KEY_ID_OCTET_OFFSET                          0x00000030
#define RX_MPDU_INFO_12_KEY_ID_OCTET_LSB                             0
#define RX_MPDU_INFO_12_KEY_ID_OCTET_MASK                            0x000000ff

#define RX_MPDU_INFO_12_NEW_PEER_ENTRY_OFFSET                        0x00000030
#define RX_MPDU_INFO_12_NEW_PEER_ENTRY_LSB                           8
#define RX_MPDU_INFO_12_NEW_PEER_ENTRY_MASK                          0x00000100

#define RX_MPDU_INFO_12_DECRYPT_NEEDED_OFFSET                        0x00000030
#define RX_MPDU_INFO_12_DECRYPT_NEEDED_LSB                           9
#define RX_MPDU_INFO_12_DECRYPT_NEEDED_MASK                          0x00000200

#define RX_MPDU_INFO_12_DECAP_TYPE_OFFSET                            0x00000030
#define RX_MPDU_INFO_12_DECAP_TYPE_LSB                               10
#define RX_MPDU_INFO_12_DECAP_TYPE_MASK                              0x00000c00

#define RX_MPDU_INFO_12_RX_INSERT_VLAN_C_TAG_PADDING_OFFSET          0x00000030
#define RX_MPDU_INFO_12_RX_INSERT_VLAN_C_TAG_PADDING_LSB             12
#define RX_MPDU_INFO_12_RX_INSERT_VLAN_C_TAG_PADDING_MASK            0x00001000

#define RX_MPDU_INFO_12_RX_INSERT_VLAN_S_TAG_PADDING_OFFSET          0x00000030
#define RX_MPDU_INFO_12_RX_INSERT_VLAN_S_TAG_PADDING_LSB             13
#define RX_MPDU_INFO_12_RX_INSERT_VLAN_S_TAG_PADDING_MASK            0x00002000

#define RX_MPDU_INFO_12_STRIP_VLAN_C_TAG_DECAP_OFFSET                0x00000030
#define RX_MPDU_INFO_12_STRIP_VLAN_C_TAG_DECAP_LSB                   14
#define RX_MPDU_INFO_12_STRIP_VLAN_C_TAG_DECAP_MASK                  0x00004000

#define RX_MPDU_INFO_12_STRIP_VLAN_S_TAG_DECAP_OFFSET                0x00000030
#define RX_MPDU_INFO_12_STRIP_VLAN_S_TAG_DECAP_LSB                   15
#define RX_MPDU_INFO_12_STRIP_VLAN_S_TAG_DECAP_MASK                  0x00008000

#define RX_MPDU_INFO_12_PRE_DELIM_COUNT_OFFSET                       0x00000030
#define RX_MPDU_INFO_12_PRE_DELIM_COUNT_LSB                          16
#define RX_MPDU_INFO_12_PRE_DELIM_COUNT_MASK                         0x0fff0000

#define RX_MPDU_INFO_12_AMPDU_FLAG_OFFSET                            0x00000030
#define RX_MPDU_INFO_12_AMPDU_FLAG_LSB                               28
#define RX_MPDU_INFO_12_AMPDU_FLAG_MASK                              0x10000000

#define RX_MPDU_INFO_12_BAR_FRAME_OFFSET                             0x00000030
#define RX_MPDU_INFO_12_BAR_FRAME_LSB                                29
#define RX_MPDU_INFO_12_BAR_FRAME_MASK                               0x20000000

#define RX_MPDU_INFO_12_RAW_MPDU_OFFSET                              0x00000030
#define RX_MPDU_INFO_12_RAW_MPDU_LSB                                 30
#define RX_MPDU_INFO_12_RAW_MPDU_MASK                                0x40000000

#define RX_MPDU_INFO_12_RESERVED_12_OFFSET                           0x00000030
#define RX_MPDU_INFO_12_RESERVED_12_LSB                              31
#define RX_MPDU_INFO_12_RESERVED_12_MASK                             0x80000000

#define RX_MPDU_INFO_13_MPDU_LENGTH_OFFSET                           0x00000034
#define RX_MPDU_INFO_13_MPDU_LENGTH_LSB                              0
#define RX_MPDU_INFO_13_MPDU_LENGTH_MASK                             0x00003fff

#define RX_MPDU_INFO_13_FIRST_MPDU_OFFSET                            0x00000034
#define RX_MPDU_INFO_13_FIRST_MPDU_LSB                               14
#define RX_MPDU_INFO_13_FIRST_MPDU_MASK                              0x00004000

#define RX_MPDU_INFO_13_MCAST_BCAST_OFFSET                           0x00000034
#define RX_MPDU_INFO_13_MCAST_BCAST_LSB                              15
#define RX_MPDU_INFO_13_MCAST_BCAST_MASK                             0x00008000

#define RX_MPDU_INFO_13_AST_INDEX_NOT_FOUND_OFFSET                   0x00000034
#define RX_MPDU_INFO_13_AST_INDEX_NOT_FOUND_LSB                      16
#define RX_MPDU_INFO_13_AST_INDEX_NOT_FOUND_MASK                     0x00010000

#define RX_MPDU_INFO_13_AST_INDEX_TIMEOUT_OFFSET                     0x00000034
#define RX_MPDU_INFO_13_AST_INDEX_TIMEOUT_LSB                        17
#define RX_MPDU_INFO_13_AST_INDEX_TIMEOUT_MASK                       0x00020000

#define RX_MPDU_INFO_13_POWER_MGMT_OFFSET                            0x00000034
#define RX_MPDU_INFO_13_POWER_MGMT_LSB                               18
#define RX_MPDU_INFO_13_POWER_MGMT_MASK                              0x00040000

#define RX_MPDU_INFO_13_NON_QOS_OFFSET                               0x00000034
#define RX_MPDU_INFO_13_NON_QOS_LSB                                  19
#define RX_MPDU_INFO_13_NON_QOS_MASK                                 0x00080000

#define RX_MPDU_INFO_13_NULL_DATA_OFFSET                             0x00000034
#define RX_MPDU_INFO_13_NULL_DATA_LSB                                20
#define RX_MPDU_INFO_13_NULL_DATA_MASK                               0x00100000

#define RX_MPDU_INFO_13_MGMT_TYPE_OFFSET                             0x00000034
#define RX_MPDU_INFO_13_MGMT_TYPE_LSB                                21
#define RX_MPDU_INFO_13_MGMT_TYPE_MASK                               0x00200000

#define RX_MPDU_INFO_13_CTRL_TYPE_OFFSET                             0x00000034
#define RX_MPDU_INFO_13_CTRL_TYPE_LSB                                22
#define RX_MPDU_INFO_13_CTRL_TYPE_MASK                               0x00400000

#define RX_MPDU_INFO_13_MORE_DATA_OFFSET                             0x00000034
#define RX_MPDU_INFO_13_MORE_DATA_LSB                                23
#define RX_MPDU_INFO_13_MORE_DATA_MASK                               0x00800000

#define RX_MPDU_INFO_13_EOSP_OFFSET                                  0x00000034
#define RX_MPDU_INFO_13_EOSP_LSB                                     24
#define RX_MPDU_INFO_13_EOSP_MASK                                    0x01000000

#define RX_MPDU_INFO_13_FRAGMENT_FLAG_OFFSET                         0x00000034
#define RX_MPDU_INFO_13_FRAGMENT_FLAG_LSB                            25
#define RX_MPDU_INFO_13_FRAGMENT_FLAG_MASK                           0x02000000

#define RX_MPDU_INFO_13_ORDER_OFFSET                                 0x00000034
#define RX_MPDU_INFO_13_ORDER_LSB                                    26
#define RX_MPDU_INFO_13_ORDER_MASK                                   0x04000000

#define RX_MPDU_INFO_13_U_APSD_TRIGGER_OFFSET                        0x00000034
#define RX_MPDU_INFO_13_U_APSD_TRIGGER_LSB                           27
#define RX_MPDU_INFO_13_U_APSD_TRIGGER_MASK                          0x08000000

#define RX_MPDU_INFO_13_ENCRYPT_REQUIRED_OFFSET                      0x00000034
#define RX_MPDU_INFO_13_ENCRYPT_REQUIRED_LSB                         28
#define RX_MPDU_INFO_13_ENCRYPT_REQUIRED_MASK                        0x10000000

#define RX_MPDU_INFO_13_DIRECTED_OFFSET                              0x00000034
#define RX_MPDU_INFO_13_DIRECTED_LSB                                 29
#define RX_MPDU_INFO_13_DIRECTED_MASK                                0x20000000

#define RX_MPDU_INFO_13_AMSDU_PRESENT_OFFSET                         0x00000034
#define RX_MPDU_INFO_13_AMSDU_PRESENT_LSB                            30
#define RX_MPDU_INFO_13_AMSDU_PRESENT_MASK                           0x40000000

#define RX_MPDU_INFO_13_RESERVED_13_OFFSET                           0x00000034
#define RX_MPDU_INFO_13_RESERVED_13_LSB                              31
#define RX_MPDU_INFO_13_RESERVED_13_MASK                             0x80000000

#define RX_MPDU_INFO_14_MPDU_FRAME_CONTROL_FIELD_OFFSET              0x00000038
#define RX_MPDU_INFO_14_MPDU_FRAME_CONTROL_FIELD_LSB                 0
#define RX_MPDU_INFO_14_MPDU_FRAME_CONTROL_FIELD_MASK                0x0000ffff

#define RX_MPDU_INFO_14_MPDU_DURATION_FIELD_OFFSET                   0x00000038
#define RX_MPDU_INFO_14_MPDU_DURATION_FIELD_LSB                      16
#define RX_MPDU_INFO_14_MPDU_DURATION_FIELD_MASK                     0xffff0000

#define RX_MPDU_INFO_15_MAC_ADDR_AD1_31_0_OFFSET                     0x0000003c
#define RX_MPDU_INFO_15_MAC_ADDR_AD1_31_0_LSB                        0
#define RX_MPDU_INFO_15_MAC_ADDR_AD1_31_0_MASK                       0xffffffff

#define RX_MPDU_INFO_16_MAC_ADDR_AD1_47_32_OFFSET                    0x00000040
#define RX_MPDU_INFO_16_MAC_ADDR_AD1_47_32_LSB                       0
#define RX_MPDU_INFO_16_MAC_ADDR_AD1_47_32_MASK                      0x0000ffff

#define RX_MPDU_INFO_16_MAC_ADDR_AD2_15_0_OFFSET                     0x00000040
#define RX_MPDU_INFO_16_MAC_ADDR_AD2_15_0_LSB                        16
#define RX_MPDU_INFO_16_MAC_ADDR_AD2_15_0_MASK                       0xffff0000

#define RX_MPDU_INFO_17_MAC_ADDR_AD2_47_16_OFFSET                    0x00000044
#define RX_MPDU_INFO_17_MAC_ADDR_AD2_47_16_LSB                       0
#define RX_MPDU_INFO_17_MAC_ADDR_AD2_47_16_MASK                      0xffffffff

#define RX_MPDU_INFO_18_MAC_ADDR_AD3_31_0_OFFSET                     0x00000048
#define RX_MPDU_INFO_18_MAC_ADDR_AD3_31_0_LSB                        0
#define RX_MPDU_INFO_18_MAC_ADDR_AD3_31_0_MASK                       0xffffffff

#define RX_MPDU_INFO_19_MAC_ADDR_AD3_47_32_OFFSET                    0x0000004c
#define RX_MPDU_INFO_19_MAC_ADDR_AD3_47_32_LSB                       0
#define RX_MPDU_INFO_19_MAC_ADDR_AD3_47_32_MASK                      0x0000ffff

#define RX_MPDU_INFO_19_MPDU_SEQUENCE_CONTROL_FIELD_OFFSET           0x0000004c
#define RX_MPDU_INFO_19_MPDU_SEQUENCE_CONTROL_FIELD_LSB              16
#define RX_MPDU_INFO_19_MPDU_SEQUENCE_CONTROL_FIELD_MASK             0xffff0000

#define RX_MPDU_INFO_20_MAC_ADDR_AD4_31_0_OFFSET                     0x00000050
#define RX_MPDU_INFO_20_MAC_ADDR_AD4_31_0_LSB                        0
#define RX_MPDU_INFO_20_MAC_ADDR_AD4_31_0_MASK                       0xffffffff

#define RX_MPDU_INFO_21_MAC_ADDR_AD4_47_32_OFFSET                    0x00000054
#define RX_MPDU_INFO_21_MAC_ADDR_AD4_47_32_LSB                       0
#define RX_MPDU_INFO_21_MAC_ADDR_AD4_47_32_MASK                      0x0000ffff

#define RX_MPDU_INFO_21_MPDU_QOS_CONTROL_FIELD_OFFSET                0x00000054
#define RX_MPDU_INFO_21_MPDU_QOS_CONTROL_FIELD_LSB                   16
#define RX_MPDU_INFO_21_MPDU_QOS_CONTROL_FIELD_MASK                  0xffff0000

#define RX_MPDU_INFO_22_MPDU_HT_CONTROL_FIELD_OFFSET                 0x00000058
#define RX_MPDU_INFO_22_MPDU_HT_CONTROL_FIELD_LSB                    0
#define RX_MPDU_INFO_22_MPDU_HT_CONTROL_FIELD_MASK                   0xffffffff

#endif
