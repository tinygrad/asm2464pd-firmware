#ifndef PAGE1_REGS_H
#define PAGE1_REGS_H

/*
 * page1_regs.h — named offsets for the page-1 (DPX=1) register windows.
 *
 * These are OFFSETS ONLY. Access still goes through the per-access banking
 * macros in sb.h (SB_RD/SB_WR, SBTX_RD/SBTX_WR, SBP2_RD, P12_RD/P12_WR,
 * P1_RD/P1_WR) which toggle DPX around each MOVX — the only correct model
 * given that these routines interleave page-0 and page-1 XDATA in the same
 * block. Naming the offset argument is a pure preprocessor substitution, so
 * the compiled binary is unchanged (verify: fast_check.sh -> IDENTICAL).
 *
 * Register semantics are from the RE + hardware-captured project notes.
 * Names marked "inferred" are best-effort from call context; the hex suffix
 * is kept so an unproven guess stays honest and greppable against Ghidra.
 */

/* ---- Sideband transport regs: SB_RD/SB_WR, 0x2800 window ---- */
#define SB_ADP0_CTRL        0x00u   /* adapter/link arm (inferred): .6 arm, .7 go */
#define SB_ADP1_CTRL        0x01u   /* inferred */
#define SB_ADP0_EN          0x04u   /* .0 enable (inferred) */
#define SB_ROUTEROP_COMMIT  0x06u   /* write 0x01 to post a router-op */
#define SB_DESC_COUNT_GO    0x0Cu   /* descriptor count in low bits; |0x03 kicks */
#define SB_DESC_CMD         0x15u   /* descriptor / router-op opcode */
#define SB_USB_MODE         0x1Cu   /* .0 USB (vs USB4) mode */
#define SB_CH_ROUTE_LO(p)   (0x20u + (p))   /* port route-id lo (p0=0x20, p1=0x21) */
#define SB_CH_ROUTE_HI(p)   (0x22u + (p))   /* port route-id hi (p0=0x22, p1=0x23) */
#define SB_TRANSPORT_STAT   0x24u   /* .0 edge toggle, .2:1 active port */
#define SB_ROUTEROP_EVENT   0x26u   /* .1 router-op pending (W1C), .2 L0-dis, .4 L1-dis */
#define SB_CONN_EDGE(p)     (0x28u + 2u*(p)) /* per-port connect edge: p0=0x28, p1=0x2A; .3 edge, W1C */
#define SB_CONNECT_EVENT    0x2Cu   /* .0 connect, .1 disconnect (W1C) */
#define SB_CONNECT_STATE    0x2Du   /* .0/.1 latched connect state */
#define SB_LINK_REINIT_50   0x50u   /* inferred */
#define SB_LINK_REINIT_5A   0x5Au   /* inferred */
#define SB_LANE_PRESENT(l)  ((l) ? 0x60u : 0x56u)  /* lane-present latch (inferred) */
#define SB_CONNECT_PRESENT_57 0x57u /* inferred */
#define SB_CONNECT_PRESENT_61 0x61u /* inferred */
#define SB_CL0_ACK          0x64u   /* .0 L0, .1 L1 CL0-latched */
#define SB_BOND_EVENT       0x66u   /* bond/lane events (W1C), see BOND_EVT_* below */
#define SB_LINK_EDGE(p)     (0x81u + 2u*(p)) /* per-port link edge: p2=0x81, p3=0x83; .3 edge, W1C */
#define SB_CL0_EVENT        0x9Eu   /* CL0/lane-train events (W1C), see CL0_EVT_* below */
#define SB_LANE_CL(l)       (0xA0u + (l))   /* lane CL state, low nibble; 2 == CL0/bonded (L0=0xA0, L1=0xA1) */
#define SB_CH2_ROUTE_LO(l)  (0xA4u + (l))   /* port2/3 route-id lo (0xA4/0xA5) */
#define SB_CH2_ROUTE_HI(l)  (0xA6u + (l))   /* port2/3 route-id hi (0xA6/0xA7) */
#define SB_KEYSTONE_BA      0xBAu
#define SB_KEYSTONE_BD      0xBDu
#define SB_PORT_SVC         0xC9u   /* per-port W1C: bits7:4 request, bits3:0 ack (idx = 0x10<<ch) */
#define SB_PEER_CL0         0xD4u   /* .5 peer reached CL0 */
#define SB_ROUTE_ACK        0xD8u   /* write 0x02 to ack a route */
#define SB_ROUTE_GATE       0xEDu   /* .7 route gate (inferred) */
#define SB_EVENT_CLEAR_F6   0xF6u   /* read-to-clear (inferred) */
#define SB_LANESEL          0x40u   /* lane-descriptor select (inferred) */
#define SB_RATE_STROBE      0x65u   /* lane-rate strobe: .6 arm, .7 go (inferred) */
#define SB_RATE_HI(l)       (0x6Au + 2u*(l)) /* per-lane rate hi (0x6A/0x6C, inferred) */
#define SB_RATE_LO(l)       (0x6Bu + 2u*(l)) /* per-lane rate lo (0x6B/0x6D, inferred) */
#define SB_WIDTH_LO         0x74u   /* lane-width cfg lo (inferred) */
#define SB_WIDTH_HI         0x75u   /* lane-width cfg hi (inferred) */

/* SB_BOND_EVENT (0x66) bits */
#define BOND_EVT_BONDED     0x01u
#define BOND_EVT_L0_ABR2    0x04u
#define BOND_EVT_L0_FAIL    0x08u
#define BOND_EVT_L1_ABR2    0x20u
#define BOND_EVT_L1_FAIL    0x40u
/* SB_CL0_EVENT (0x9E) bits */
#define CL0_EVT_L0          0x01u
#define CL0_EVT_L1          0x02u
/* SB_ROUTEROP_EVENT (0x26) bits */
#define ROUTEROP_EVT_PENDING 0x02u

/* ---- SB TX window: SBTX_RD/SBTX_WR, 0x2900 ---- */
#define SBTX_DESC_TYPE      0x00u
#define SBTX_DESC_DIR       0x01u   /* dir bit .7 + width nibble */
#define SBTX_DESC_BODY      0x02u   /* body base; body[i] = SBTX_DESC_BODY + i */

/* ---- Host connect descriptor plane: SBP2_RD, 0x2A00 + (port<<8) ---- */
#define SBP2_DESC_TYPE      0x00u
#define SBP2_DESC_LEN       0x01u   /* .6:0 len, .7 dir */
#define SBP2_DESC_BODY      0x02u

/* ---- Router config-space / descriptor engine (CS_25): P12_RD/P12_WR, 0x1200 ---- */
#define DE_LANESEL          0x34u   /* CS index / data-in cursor */
#define DE_CTRL             0x35u   /* engine control cursor */
#define DE_OPCODE           0x36u
#define DE_COMMIT           0x37u   /* .7 commit */
#define DE_KICK             0x38u   /* .0 busy/kick */
#define DE_WR(i)            (0x3Cu + (i))  /* write data quad  0x3C..0x3F */
#define DE_RD(i)            (0x40u + (i))  /* readback quad    0x40..0x43 */
#define DE_TRANSPORT(i)     (0x4Cu + (i))  /* transport reinit 0x4C..0x4F (inferred) */
#define DE_ENG_RESET_03     0x03u
#define DE_ENG_DATA_BASE    0x12u   /* 9-byte init block 0x12..0x1A */
#define DE_ENG_RESET_7A     0x7Au   /* inferred */
#define DE_ENG_RESET_8F     0x8Fu   /* inferred */
#define DE_ENG_RESET_90     0x90u   /* inferred */

/* ---- Raw page-1 tunnel/link/event regs: P1_RD/P1_WR (arbitrary DPX=1 addr) ---- */
#define P1_PORT_CTRL_0000   0x0000u          /* .1 SB assert (inferred) */
#define P1_LANE_FLIP(i)     (0x0100u + (i))  /* lane orient/flip + link ctrl 0x0100..0x0102 */
#define P1_ROUTE_ACK        0x0109u          /* .0 in-band route ack */
#define P1_LANE_EN_010B     0x010Bu          /* lane enable (inferred) */
/* USB4 tunnel / link / descriptor-engine mirror regs (inferred; hex-suffixed). */
#define P1_ADP_LINK_CFG_1206   0x1206u       /* adapter link cfg (.5 lane-width) */
#define P1_DESC_CTRL_1235      0x1235u       /* page-1 descriptor-engine ctrl (mirrors DE_CTRL) */
#define P1_DESC_CMD_1236       0x1236u       /* page-1 descriptor-engine command */
#define P1_DESC_COMMIT_1237    0x1237u       /* page-1 descriptor-engine commit (mirrors DE_COMMIT) */
#define P1_DESC_RESULT_1243    0x1243u       /* page-1 descriptor result; .7 = width gate */
#define P1_LINK_PHY_CFG_1267   0x1267u       /* link-PHY reconfig */
#define P1_TUNNEL_PHY_1285     0x1285u       /* tunnel-PHY finalize (hi-nibble 0x30) */
#define P1_TUNNEL_PHY_CTRL_1334 0x1334u      /* tunnel-PHY finalize control */
#define P1_TUNNEL_PHY_CFG_1335  0x1335u      /* tunnel-PHY finalize config */
#define P1_TUNNEL_PHY_134D     0x134Du       /* tunnel-PHY finalize (write 0x04) */
#define P1_XPORT_LANE_EVT_1404 0x1404u       /* transport lane event (lane-gated) */
#define P1_XPORT_LANE_EVT_1405 0x1405u       /* transport lane event (skip-lane path) */
#define P1_XPORT_TRIG_1511     0x1511u       /* transport reinit trigger (write 0x01) */
#define P1_XPORT_RESET_1802    0x1802u       /* transport-layer reset strobe (bits 1-4) */
#define P1_RXPLL_CFG_1808      0x1808u       /* RXPLL cfg-trigger context */
#define P1_PCIE_LINK_1835      0x1835u       /* PCIe link bring-up */
#define P1_PCIE_LANE_SLOT(l)   (0x78AFu + 0x100u*(l)) /* per-lane PCIe slot-enable .7 (0x78AF..0x7BAF) */
/* 0x1203/0x121E/0x1406/0x1407/0x1507/0x1508/0x1602/0x1603 are already named
 * P1_USB4_* in registers.h — reuse those. */

#endif /* PAGE1_REGS_H */
