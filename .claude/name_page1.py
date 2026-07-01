#!/usr/bin/env python3
"""Rewrite bare-hex page-1 register-macro args to page1_regs.h names.
Precise: only replaces the FIRST arg of SB/SBTX/SBP2/P12/P1 _RD/_WR/_CLR/_SET
calls, keyed per family. md5 gate validates values. Usage: name_page1.py <file>..."""
import re, sys

MAP = {
 'SB': {
  '0x00':'SB_ADP0_CTRL','0x01':'SB_ADP1_CTRL','0x04':'SB_ADP0_EN','0x06':'SB_ROUTEROP_COMMIT',
  '0x0C':'SB_DESC_COUNT_GO','0x15':'SB_DESC_CMD','0x1C':'SB_USB_MODE',
  '0x20':'SB_CH_ROUTE_LO(0)','0x21':'SB_CH_ROUTE_LO(1)','0x22':'SB_CH_ROUTE_HI(0)','0x23':'SB_CH_ROUTE_HI(1)',
  '0x24':'SB_TRANSPORT_STAT','0x26':'SB_ROUTEROP_EVENT','0x28':'SB_CONN_EDGE(0)','0x2A':'SB_CONN_EDGE(1)',
  '0x2C':'SB_CONNECT_EVENT','0x2D':'SB_CONNECT_STATE',
  '0x50':'SB_LINK_REINIT_50','0x5A':'SB_LINK_REINIT_5A','0x56':'SB_LANE_PRESENT(0)','0x60':'SB_LANE_PRESENT(1)',
  '0x57':'SB_CONNECT_PRESENT_57','0x61':'SB_CONNECT_PRESENT_61',
  '0x64':'SB_CL0_ACK','0x66':'SB_BOND_EVENT','0x81':'SB_LINK_EDGE(0)','0x83':'SB_LINK_EDGE(1)',
  '0x9E':'SB_CL0_EVENT','0xA0':'SB_LANE_CL(0)','0xA1':'SB_LANE_CL(1)',
  '0xA4':'SB_CH2_ROUTE_LO(0)','0xA5':'SB_CH2_ROUTE_LO(1)','0xA6':'SB_CH2_ROUTE_HI(0)','0xA7':'SB_CH2_ROUTE_HI(1)',
  '0xBA':'SB_KEYSTONE_BA','0xBD':'SB_KEYSTONE_BD','0xC9':'SB_PORT_SVC','0xD4':'SB_PEER_CL0',
  '0xD8':'SB_ROUTE_ACK','0xED':'SB_ROUTE_GATE','0xF6':'SB_EVENT_CLEAR_F6',
  '0x40':'SB_LANESEL','0x65':'SB_RATE_STROBE',
  '0x6A':'SB_RATE_HI(0)','0x6C':'SB_RATE_HI(1)','0x6B':'SB_RATE_LO(0)','0x6D':'SB_RATE_LO(1)',
  '0x74':'SB_WIDTH_LO','0x75':'SB_WIDTH_HI',
 },
 'SBTX': {'0':'SBTX_DESC_TYPE','1':'SBTX_DESC_DIR','2':'SBTX_DESC_BODY',
          '0x00':'SBTX_DESC_TYPE','0x01':'SBTX_DESC_DIR','0x02':'SBTX_DESC_BODY'},
 'SBP2': {'0':'SBP2_DESC_TYPE','1':'SBP2_DESC_LEN',
          '0x00':'SBP2_DESC_TYPE','0x01':'SBP2_DESC_LEN'},
 'P12': {
  '0x34':'DE_LANESEL','0x35':'DE_CTRL','0x36':'DE_OPCODE','0x37':'DE_COMMIT','0x38':'DE_KICK',
  '0x3C':'DE_WR(0)','0x3D':'DE_WR(1)','0x3E':'DE_WR(2)','0x3F':'DE_WR(3)',
  '0x40':'DE_RD(0)','0x41':'DE_RD(1)','0x42':'DE_RD(2)','0x43':'DE_RD(3)',
  '0x4C':'DE_TRANSPORT(0)','0x4D':'DE_TRANSPORT(1)','0x4E':'DE_TRANSPORT(2)','0x4F':'DE_TRANSPORT(3)',
  '0x03':'DE_ENG_RESET_03','0x7A':'DE_ENG_RESET_7A','0x8F':'DE_ENG_RESET_8F','0x90':'DE_ENG_RESET_90',
 },
 'P1': {'0x0100':'P1_LANE_FLIP(0)','0x0101':'P1_LANE_FLIP(1)','0x0102':'P1_LANE_FLIP(2)','0x0109':'P1_ROUTE_ACK',
  '0x0000':'P1_PORT_CTRL_0000','0x010B':'P1_LANE_EN_010B',
  '0x1206':'P1_ADP_LINK_CFG_1206','0x1235':'P1_DESC_CTRL_1235','0x1236':'P1_DESC_CMD_1236',
  '0x1237':'P1_DESC_COMMIT_1237','0x1243':'P1_DESC_RESULT_1243','0x1267':'P1_LINK_PHY_CFG_1267',
  '0x1285':'P1_TUNNEL_PHY_1285','0x1334':'P1_TUNNEL_PHY_CTRL_1334','0x1335':'P1_TUNNEL_PHY_CFG_1335',
  '0x134D':'P1_TUNNEL_PHY_134D','0x1404':'P1_XPORT_LANE_EVT_1404','0x1405':'P1_XPORT_LANE_EVT_1405',
  '0x1511':'P1_XPORT_TRIG_1511','0x1802':'P1_XPORT_RESET_1802','0x1808':'P1_RXPLL_CFG_1808',
  '0x1835':'P1_PCIE_LINK_1835',
  '0x78AF':'P1_PCIE_LANE_SLOT(0)','0x79AF':'P1_PCIE_LANE_SLOT(1)','0x7AAF':'P1_PCIE_LANE_SLOT(2)','0x7BAF':'P1_PCIE_LANE_SLOT(3)'},
}
FAM = re.compile(r'\b(SB|SBTX|SBP2|P12|P1)_(RD|WR|CLR|SET)\s*\(')

def rewrite(src):
    out=[]; i=0; n=0
    while True:
        m=FAM.search(src,i)
        if not m: out.append(src[i:]); break
        out.append(src[i:m.end()])
        # extract first arg (to top-level comma or close paren)
        j=m.end(); depth=1; arg=''
        while j<len(src) and depth>0:
            ch=src[j]
            if ch=='(':depth+=1
            elif ch==')':
                depth-=1
                if depth==0: break
            if ch==',' and depth==1: break
            arg+=ch; j+=1
        key=arg.strip()
        rep=MAP.get(m.group(1),{}).get(key)
        if rep is not None:
            out.append(arg.replace(key,rep,1)); n+=1
        else:
            out.append(arg)
        i=j
    return ''.join(out), n

for f in sys.argv[1:]:
    s=open(f).read()
    r,n=rewrite(s)
    open(f,'w').write(r)
    print(f"{f}: {n} substitutions")
