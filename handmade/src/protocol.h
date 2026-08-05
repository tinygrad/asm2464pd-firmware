#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#define FW_INFO_REQUEST                    0xF4
#define FW_INFO_SIZE                       16
#define FW_PROTOCOL_MAJOR                  1
#define FW_PROTOCOL_MINOR                  0
#define FW_REVISION                        8UL

#define FW_CAP_XDATA                       (1UL << 0)
#define FW_CAP_PCIE_TLP                    (1UL << 1)
#define FW_CAP_SRAM_DMA                    (1UL << 2)
#define FW_CAP_PCIE_POWER                  (1UL << 3)
#define FW_CAP_PCIE_POWER_POST_TRAIN       (1UL << 4)
#define FW_CAP_HW_STATUS                   (1UL << 5)
#define FW_CAP_USB3_DIRECT                 (1UL << 6)
#define FW_CAP_SRAM_STREAM                 (1UL << 7)
#define FW_CAPABILITIES                    (FW_CAP_XDATA | FW_CAP_PCIE_TLP | FW_CAP_SRAM_DMA | FW_CAP_PCIE_POWER | \
                                            FW_CAP_PCIE_POWER_POST_TRAIN | FW_CAP_HW_STATUS | FW_CAP_USB3_DIRECT | \
                                            FW_CAP_SRAM_STREAM)

#endif
