
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



#ifndef _RX_MPDU_END_H_
#define _RX_MPDU_END_H_
#if !defined(__ASSEMBLER__)
#endif

#define NUM_OF_DWORDS_RX_MPDU_END 2

struct rx_mpdu_end {
             uint32_t rxpcu_mpdu_filter_in_category   :  2,
                      sw_frame_group_id               :  7,
                      reserved_0                      :  7,
                      phy_ppdu_id                     : 16;
             uint32_t reserved_1a                     : 11,
                      unsup_ktype_short_frame         :  1,
                      rx_in_tx_decrypt_byp            :  1,
                      overflow_err                    :  1,
                      mpdu_length_err                 :  1,
                      tkip_mic_err                    :  1,
                      decrypt_err                     :  1,
                      unencrypted_frame_err           :  1,
                      pn_fields_contain_valid_info    :  1,
                      fcs_err                         :  1,
                      msdu_length_err                 :  1,
                      rxdma0_destination_ring         :  2,
                      rxdma1_destination_ring         :  2,
                      decrypt_status_code             :  3,
                      rx_bitmap_not_updated           :  1,
                      reserved_1b                     :  3;
};

#define RX_MPDU_END_0_RXPCU_MPDU_FILTER_IN_CATEGORY_OFFSET           0x00000000
#define RX_MPDU_END_0_RXPCU_MPDU_FILTER_IN_CATEGORY_LSB              0
#define RX_MPDU_END_0_RXPCU_MPDU_FILTER_IN_CATEGORY_MASK             0x00000003

#define RX_MPDU_END_0_SW_FRAME_GROUP_ID_OFFSET                       0x00000000
#define RX_MPDU_END_0_SW_FRAME_GROUP_ID_LSB                          2
#define RX_MPDU_END_0_SW_FRAME_GROUP_ID_MASK                         0x000001fc

#define RX_MPDU_END_0_RESERVED_0_OFFSET                              0x00000000
#define RX_MPDU_END_0_RESERVED_0_LSB                                 9
#define RX_MPDU_END_0_RESERVED_0_MASK                                0x0000fe00

#define RX_MPDU_END_0_PHY_PPDU_ID_OFFSET                             0x00000000
#define RX_MPDU_END_0_PHY_PPDU_ID_LSB                                16
#define RX_MPDU_END_0_PHY_PPDU_ID_MASK                               0xffff0000

#define RX_MPDU_END_1_RESERVED_1A_OFFSET                             0x00000004
#define RX_MPDU_END_1_RESERVED_1A_LSB                                0
#define RX_MPDU_END_1_RESERVED_1A_MASK                               0x000007ff

#define RX_MPDU_END_1_UNSUP_KTYPE_SHORT_FRAME_OFFSET                 0x00000004
#define RX_MPDU_END_1_UNSUP_KTYPE_SHORT_FRAME_LSB                    11
#define RX_MPDU_END_1_UNSUP_KTYPE_SHORT_FRAME_MASK                   0x00000800

#define RX_MPDU_END_1_RX_IN_TX_DECRYPT_BYP_OFFSET                    0x00000004
#define RX_MPDU_END_1_RX_IN_TX_DECRYPT_BYP_LSB                       12
#define RX_MPDU_END_1_RX_IN_TX_DECRYPT_BYP_MASK                      0x00001000

#define RX_MPDU_END_1_OVERFLOW_ERR_OFFSET                            0x00000004
#define RX_MPDU_END_1_OVERFLOW_ERR_LSB                               13
#define RX_MPDU_END_1_OVERFLOW_ERR_MASK                              0x00002000

#define RX_MPDU_END_1_MPDU_LENGTH_ERR_OFFSET                         0x00000004
#define RX_MPDU_END_1_MPDU_LENGTH_ERR_LSB                            14
#define RX_MPDU_END_1_MPDU_LENGTH_ERR_MASK                           0x00004000

#define RX_MPDU_END_1_TKIP_MIC_ERR_OFFSET                            0x00000004
#define RX_MPDU_END_1_TKIP_MIC_ERR_LSB                               15
#define RX_MPDU_END_1_TKIP_MIC_ERR_MASK                              0x00008000

#define RX_MPDU_END_1_DECRYPT_ERR_OFFSET                             0x00000004
#define RX_MPDU_END_1_DECRYPT_ERR_LSB                                16
#define RX_MPDU_END_1_DECRYPT_ERR_MASK                               0x00010000

#define RX_MPDU_END_1_UNENCRYPTED_FRAME_ERR_OFFSET                   0x00000004
#define RX_MPDU_END_1_UNENCRYPTED_FRAME_ERR_LSB                      17
#define RX_MPDU_END_1_UNENCRYPTED_FRAME_ERR_MASK                     0x00020000

#define RX_MPDU_END_1_PN_FIELDS_CONTAIN_VALID_INFO_OFFSET            0x00000004
#define RX_MPDU_END_1_PN_FIELDS_CONTAIN_VALID_INFO_LSB               18
#define RX_MPDU_END_1_PN_FIELDS_CONTAIN_VALID_INFO_MASK              0x00040000

#define RX_MPDU_END_1_FCS_ERR_OFFSET                                 0x00000004
#define RX_MPDU_END_1_FCS_ERR_LSB                                    19
#define RX_MPDU_END_1_FCS_ERR_MASK                                   0x00080000

#define RX_MPDU_END_1_MSDU_LENGTH_ERR_OFFSET                         0x00000004
#define RX_MPDU_END_1_MSDU_LENGTH_ERR_LSB                            20
#define RX_MPDU_END_1_MSDU_LENGTH_ERR_MASK                           0x00100000

#define RX_MPDU_END_1_RXDMA0_DESTINATION_RING_OFFSET                 0x00000004
#define RX_MPDU_END_1_RXDMA0_DESTINATION_RING_LSB                    21
#define RX_MPDU_END_1_RXDMA0_DESTINATION_RING_MASK                   0x00600000

#define RX_MPDU_END_1_RXDMA1_DESTINATION_RING_OFFSET                 0x00000004
#define RX_MPDU_END_1_RXDMA1_DESTINATION_RING_LSB                    23
#define RX_MPDU_END_1_RXDMA1_DESTINATION_RING_MASK                   0x01800000

#define RX_MPDU_END_1_DECRYPT_STATUS_CODE_OFFSET                     0x00000004
#define RX_MPDU_END_1_DECRYPT_STATUS_CODE_LSB                        25
#define RX_MPDU_END_1_DECRYPT_STATUS_CODE_MASK                       0x0e000000

#define RX_MPDU_END_1_RX_BITMAP_NOT_UPDATED_OFFSET                   0x00000004
#define RX_MPDU_END_1_RX_BITMAP_NOT_UPDATED_LSB                      28
#define RX_MPDU_END_1_RX_BITMAP_NOT_UPDATED_MASK                     0x10000000

#define RX_MPDU_END_1_RESERVED_1B_OFFSET                             0x00000004
#define RX_MPDU_END_1_RESERVED_1B_LSB                                29
#define RX_MPDU_END_1_RESERVED_1B_MASK                               0xe0000000

#endif
