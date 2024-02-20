`timescale 1 ps/1 ps
//------------------------------------------------------------------------------
// File       : sitcpxg.v
// Author     : by zhj@ihep.ac.cn
//------------------------------------------------------------------------------
// Description: This is the verilog design for the Ten GigaEthernet TCP/IP core.
//              The block level wrapper for the core is instantiated and the
//              timer circuitry is created.
module sitcpxg #(
  parameter         USE_CHIPSCOPE = 0,
  parameter [31: 0] BASE_IP_ADDR  = 32'hC0A8_0A10,  //192.168.10.16
  parameter [31: 0] MAC_IP_WIDTH  = 3,
  parameter [4 : 0] PHY_ADDRESS   = 5'b1,
  parameter         RxBufferSize  = "LongLong" // Little-endian conversion for rxdata
  // "Byte":8bit width ,"Word":16bit width ,"LongWord":32bit width , "LongLong":64bit width
)(
  // System I/F
  input           RST,            // in : System reset (Sync.)
  output          CLKOUT,         // out: 156.25MHz BUFG clock out
  input           CLK40,          // in : indepondent clock for DRP
  // SiTCP setting
  input   [31: 0] REG_FPGA_VER,   // in : User logic Version(For example, the synthesized date)
  input   [31: 0] REG_FPGA_ID,    // in : User logic ID (We recommend using the lower 4 bytes of the MAC address.)
  input   [MAC_IP_WIDTH-1 :0]   MAC_SELECT,
  input   [MAC_IP_WIDTH-1 :0]   IP_SELECT,
  output          TIM_1US,      // out  : 1 us interval
  output          TIM_10US,     // out  : 10 us interval
  output          TIM_100US,    // out  : 100 us interval
  output          TIM_1MS,      // out  : 1 ms interval
  output          TIM_10MS,     // out  : 10 ms interval
  output          TIM_100MS,    // out  : 100 ms interval
  output          TIM_1S,       // out  : 1 s interval
  output          TIM_1M,       // out  : 1 m interval
  // User I/F
  output          SiTCP_RESET_OUT,// out: System reset for user's module
  // RBCP
  output          RBCP_ACT,       // out: Indicates that bus access is active.
  output  [31: 0] RBCP_ADDR,      // out: Address[31:0]
  output          RBCP_WE,        // out: Write enable
  output  [ 7: 0] RBCP_WD,        // out: Data[7:0]
  output          RBCP_RE,        // out: Read enable
  input           RBCP_ACK,       // in : Access acknowledge
  input   [ 7: 0] RBCP_RD,        // in : Read data[7:0]
  // TCP
  input           USER_SESSION_OPEN_REQ,    // in : Request for opening the new session
  output          USER_SESSION_ESTABLISHED, // out: Establish of a session
  output          USER_SESSION_CLOSE_REQ,   // out: Request for closing session.
  input           USER_SESSION_CLOSE_ACK,   // in : Acknowledge for USER_SESSION_CLOSE_REQ.
  input   [63: 0] USER_TX_D,      // in : Write data
  input   [ 3: 0] USER_TX_B,      // in : Byte length of USER_TX_DATA(Set to 0 if not written)
  output          USER_TX_AFULL,  // out: Request to stop TX
  input   [15: 0] USER_RX_SIZE,   // in : Set a fixed value less than or equal to buffer size -16
  output          USER_RX_CLR_ENB,// out: Receive buffer Clear Enable
  input           USER_RX_CLR_REQ,// in : Receive buffer Clear Request
  input   [15: 0] USER_RX_RADR,   // in : Receive buffer read address in bytes (unused upper bits are set to 0)
  output  [15: 0] USER_RX_WADR,   // out: Receive buffer write address in bytes (lower 3 bits are not connected to memory)
  output  [ 7: 0] USER_RX_WENB,   // out: Receive buffer byte write enable (big endian)
  output  [63: 0] USER_RX_WDAT,   // out: Receive buffer write data (big endian)
  // GT interface
  input           GTREFCLK_P,     // in : Differential +ve of reference clock for MGT: very high quality.
  input           GTREFCLK_N,     // in : Differential -ve of reference clock for MGT: very high quality.
  output          GT_TXP,         // out: Tx signal line
  output          GT_TXN,         //
  input           GT_RXP,         // in : Rx signal line
  input           GT_RXN          //
);

// XGMII I/F
wire  [ 7: 0] xgmii_rxc;
wire  [63: 0] xgmii_rxd;
wire  [ 7: 0] xgmii_txc;
wire  [63: 0] xgmii_txd;
wire          xgmii_clock;

wire rst;
async2sync_reset reset_usrclk(
  .rst_in       (RST),
  .clk          (xgmii_clock),
  .rst_out      (rst)
);
// assign rst = RST;

TIMER #(
  .CLK_FREQ   (8'd156)
)TIMER(
// System
  .CLK        (xgmii_clock),  // in: System clock
  .RST        (rst),          // in: System reset
// Intrrupts
  .TIM_1US    (TIM_1US),      // out: 1 us interval
  .TIM_10US   (TIM_10US),     // out: 10 us interval
  .TIM_100US  (TIM_100US),    // out: 100 us interval
  .TIM_1MS    (TIM_1MS),      // out: 1 ms interval
  .TIM_10MS   (TIM_10MS),     // out: 10 ms interval
  .TIM_100MS  (TIM_100MS),    // out: 100 ms interval
  .TIM_1S     (TIM_1S),       // out: 1 s interval
  .TIM_1M     (TIM_1M)        // out: 1 min interval
);

//------------------------------------------------------------------------------
//  SiTCP library
wire  [47: 0] TCP_SERVER_MAC;
wire  [31: 0] TCP_SERVER_ADDR;
wire  [15: 0] TCP_SERVER_PORT;
wire  [ 7: 0] swap_rx_wenb;
wire  [63: 0] swap_rx_wdat;

SiTCPXG_XC7K_128K_V3 SiTCPXG_XC7K(
  .REG_FPGA_VER             (REG_FPGA_VER),     // in : User logic Version(For example, the synthesized date)
  .REG_FPGA_ID              (REG_FPGA_ID),      // in : User logic ID (We recommend using the lower 4 bytes of the MAC address.)
  // System I/F
  .XGMII_CLOCK              (xgmii_clock),      // in : XGMII clock
  .RSTs                     (rst),              // in : System reset (Sync.)
  .TIM_1US                  (TIM_1US),          // in : 1us interval pulse
  .TIM_1MS                  (TIM_1MS),          // in : 1us interval pulse
  .TIM_1S                   (TIM_1S),           // in : 1s   interval pulse
  // XGMII I/F
  .XGMII_RXC                (xgmii_rxc),        // in : Rx control[7:0]
  .XGMII_RXD                (xgmii_rxd),        // in : Rx data[63:0]
  .XGMII_TXC                (xgmii_txc),        // out: Control bits[7:0]
  .XGMII_TXD                (xgmii_txd),        // out: Data[63:0]
  // 93C46 I/F
  .EEPROM_CS                (),                 // out: Chip select
  .EEPROM_SK                (),                 // out: Serial data clock
  .EEPROM_DI                (),                 // out: Serial write data
  .EEPROM_DO                (1'b0),             // in : Serial read data
  // Configuration parameters
  .FORCE_DEFAULTn           (1'b0),             // in : Force to set default values
  .MY_MAC_ADDR              (),                 // out: My IP MAC Address[47:0]
  .MY_IP_ADDR               (BASE_IP_ADDR+IP_SELECT),  // in : My IP address[31:0]
  .IP_ADDR_DEFAULT          (),                 // out: Default value for MY_IP_ADDR[31:0]
  .MY_TCP_PORT              (16'd24),           // in : My TCP port[15:0]
  .TCP_PORT_DEFAULT         (),                 // out: Default value for my TCP MY_TCP_PORT[15:0]
  .MY_RBCP_PORT             (16'd4660),         // in : My UDP RBCP-port[15:0]
  .RBCP_PORT_DEFAULT        (),                 // out: Default value for my UDP RBCP-port #[15:0]
  .TCP_SERVER_MAC_IN        (TCP_SERVER_MAC),   // in : Client mode, Server MAC address[47:0]
  .TCP_SERVER_MAC_DEFAULT   (TCP_SERVER_MAC),   // out: Default value for the server's MAC address
  .TCP_SERVER_ADDR_IN       (TCP_SERVER_ADDR),  // in : Client mode, Server IP address[31:0]
  .TCP_SERVER_ADDR_DEFAULT  (TCP_SERVER_ADDR),  // out: Default value for the server's IP address[31:0]
  .TCP_SERVER_PORT_IN       (TCP_SERVER_PORT),  // in : Client mode, Server wating port#[15:0]
  .TCP_SERVER_PORT_DEFAULT  (TCP_SERVER_PORT),  // out: Default value for the server port #[15:0]
  // User I/F
  .SiTCP_RESET_OUT          (SiTCP_RESET_OUT),  // out: System reset for user's module
  // RBCP
  .RBCP_ACT                 (RBCP_ACT),         // out: Indicates that bus access is active.
  .RBCP_ADDR                (RBCP_ADDR),        // out: Address[31:0]
  .RBCP_WE                  (RBCP_WE),          // out: Write enable
  .RBCP_WD                  (RBCP_WD),          // out: Data[7:0]
  .RBCP_RE                  (RBCP_RE),          // out: Read enable
  .RBCP_ACK                 (RBCP_ACK),         // in : Access acknowledge
  .RBCP_RD                  (RBCP_RD),          // in : Read data[7:0]
  // TCP
  .USER_SESSION_OPEN_REQ    (USER_SESSION_OPEN_REQ),    // in : Request for opening the new session
  .USER_SESSION_ESTABLISHED (USER_SESSION_ESTABLISHED), // out: Establish of a session
  .USER_SESSION_CLOSE_REQ   (USER_SESSION_CLOSE_REQ),   // out: Request for closing session.
  .USER_SESSION_CLOSE_ACK   (USER_SESSION_CLOSE_ACK),   // in : Acknowledge for USER_SESSION_CLOSE_REQ.
  .USER_TX_D                (USER_TX_D),        // in : Write data
  .USER_TX_B                (USER_TX_B),        // in : Byte length of USER_TX_DATA(Set to 0 if not written)
  .USER_TX_AFULL            (USER_TX_AFULL),    // out: Request to stop TX
  .USER_RX_SIZE             (USER_RX_SIZE),     // in : Set a fixed value less than or equal to buffer size -16
  .USER_RX_CLR_ENB          (USER_RX_CLR_ENB),  // out: Receive buffer Clear Enable
  .USER_RX_CLR_REQ          (USER_RX_CLR_REQ),  // in : Receive buffer Clear Request
  .USER_RX_RADR             (USER_RX_RADR),     // in : Receive buffer read address in bytes (unused upper bits are set to 0)
  .USER_RX_WADR             (USER_RX_WADR),     // out: Receive buffer write address in bytes (lower 3 bits are not connected to memory)
  .USER_RX_WENB             (swap_rx_wenb),     // out: Receive buffer byte write enable (big endian)
  .USER_RX_WDAT             (swap_rx_wdat)      // out: Receive buffer write data (big endian)
);

// Little-endian conversion
generate
  if (RxBufferSize == "LongLong") begin
    assign  USER_RX_WENB[ 7:0]  = swap_rx_wenb[ 7:0];
    assign  USER_RX_WDAT[63:0]  = swap_rx_wdat[63:0];
  end
  else if (RxBufferSize == "LongWord") begin
    assign  USER_RX_WENB[ 3: 0] = swap_rx_wenb[ 7: 4];
    assign  USER_RX_WDAT[31: 0] = swap_rx_wdat[63:32];
    assign  USER_RX_WENB[ 7: 4] = swap_rx_wenb[ 3: 0];
    assign  USER_RX_WDAT[63:32] = swap_rx_wdat[31: 0];
  end
  else if (RxBufferSize == "Word") begin
    assign  USER_RX_WENB[ 1: 0] = swap_rx_wenb[ 7: 6];
    assign  USER_RX_WDAT[15: 0] = swap_rx_wdat[63:48];
    assign  USER_RX_WENB[ 3: 2] = swap_rx_wenb[ 5: 4];
    assign  USER_RX_WDAT[31:16] = swap_rx_wdat[47:32];
    assign  USER_RX_WENB[ 5: 4] = swap_rx_wenb[ 3: 2];
    assign  USER_RX_WDAT[47:32] = swap_rx_wdat[31:16];
    assign  USER_RX_WENB[ 7: 6] = swap_rx_wenb[ 1: 0];
    assign  USER_RX_WDAT[63:48] = swap_rx_wdat[15: 0];
  end
  else if (RxBufferSize == "Byte") begin
    assign  USER_RX_WENB[0] = swap_rx_wenb[7];
    assign  USER_RX_WDAT[ 7: 0] = swap_rx_wdat[63:56];
    assign  USER_RX_WENB[1] = swap_rx_wenb[6];
    assign  USER_RX_WDAT[15: 8] = swap_rx_wdat[55:48];
    assign  USER_RX_WENB[2] = swap_rx_wenb[5];
    assign  USER_RX_WDAT[23:16] = swap_rx_wdat[47:40];
    assign  USER_RX_WENB[3] = swap_rx_wenb[4];
    assign  USER_RX_WDAT[31:24] = swap_rx_wdat[39:32];
    assign  USER_RX_WENB[4] = swap_rx_wenb[3];
    assign  USER_RX_WDAT[39:32] = swap_rx_wdat[31:24];
    assign  USER_RX_WENB[5] = swap_rx_wenb[2];
    assign  USER_RX_WDAT[47:40] = swap_rx_wdat[23:16];
    assign  USER_RX_WENB[6] = swap_rx_wenb[1];
    assign  USER_RX_WDAT[55:48] = swap_rx_wdat[15: 8];
    assign  USER_RX_WENB[7] = swap_rx_wenb[0];
    assign  USER_RX_WDAT[63:56] = swap_rx_wdat[ 7: 0];
  end
endgenerate

// Status Signals
wire [7 :0] core_status;
wire        resetdone;
wire        qplllock;
wire        intSfpTxDisable;

ten_gig_eth_pcs_pma ten_gig_eth_pcs_pma_i(
  .refclk_p               (GTREFCLK_P),   // input wire refclk_p
  .refclk_n               (GTREFCLK_N),   // input wire refclk_n
  .reset                  (rst),          // input wire reset
  .coreclk_out            (xgmii_clock),  // output wire coreclk_out
  .txp                    (GT_TXP),       // output wire txp
  .txn                    (GT_TXN),       // output wire txn
  .rxp                    (GT_RXP),       // input wire rxp
  .rxn                    (GT_RXN),       // input wire rxn
  .xgmii_txd              (xgmii_txd),    // input wire [63 : 0] xgmii_txd
  .xgmii_txc              (xgmii_txc),    // input wire [7 : 0] xgmii_txc
  .xgmii_rxd              (xgmii_rxd),    // output wire [63 : 0] xgmii_rxd
  .xgmii_rxc              (xgmii_rxc),    // output wire [7 : 0] xgmii_rxc
// MDIO
  .mdc                    (1'b1),         // input wire mdc
  .mdio_in                (1'b1),         // input wire mdio_in
  .mdio_out               (),             // output wire mdio_out
  .mdio_tri               (),             // output wire mdio_tri
  .prtad                  (PHY_ADDRESS),  // input wire [4 : 0] prtad
// SPF+ I/F
  .signal_detect          (1'b1),         // input wire signal_detect
  .tx_fault               (1'b0),         // input wire tx_fault
  .tx_disable             (intSfpTxDisable),  // output wire tx_disable
// drp_interface_ports
  .dclk                   (CLK40),        // input wire dclk  <DRP clock 156MHz>
  .drp_req                (),             // output wire drp_req
  .drp_gnt                (1'b0),         // input wire drp_gnt
// core_gt_drp_interface
  .drp_daddr_i            (16'b0),        // input wire [15 : 0] drp_daddr_i
  .drp_den_i              (1'b0),         // input wire drp_den_i
  .drp_di_i               (16'b0),        // input wire [15 : 0] drp_di_i
  .drp_drpdo_o            (),             // output wire [15 : 0] drp_drpdo_o
  .drp_drdy_o             (),             // output wire drp_drdy_o
  .drp_dwe_i              (1'b0),         // input wire drp_dwe_i
// core_gt_drp_interface
  .drp_daddr_o            (),             // output wire [15 : 0] drp_daddr_o
  .drp_den_o              (),             // output wire drp_den_o
  .drp_di_o               (),             // output wire [15 : 0] drp_di_o
  .drp_drpdo_i            (16'b0),        // input wire [15 : 0] drp_drpdo_i
  .drp_drdy_i             (1'b0),         // input wire drp_drdy_i
  .drp_dwe_o              (),             // output wire drp_dwe_o
//Miscellaneous port
  .core_status            (core_status),  // output wire [7 : 0] core_status
  .sim_speedup_control    (1'b0),         // input wire sim_speedup_control
  .pma_pmd_type           (3'b111),       // input wire for 10GBASE-SR
  .resetdone_out          (resetdone),    // output wire resetdone_out
  .rxrecclk_out           (),             // output wire rxrecclk_out
  .txusrclk_out           (),             // output wire txusrclk_out
  .txusrclk2_out          (),             // output wire txusrclk2_out
  .areset_datapathclk_out (),             // output wire areset_datapathclk_out
  .gttxreset_out          (),             // output wire gttxreset_out
  .gtrxreset_out          (),             // output wire gtrxreset_out
  .txuserrdy_out          (),             // output wire txuserrdy_out
  .reset_counter_done_out (),             // output wire reset_counter_done_out
  .qplllock_out           (qplllock),     // output wire qplllock_out
  .qplloutclk_out         (),             // output wire qplloutclk_out
  .qplloutrefclk_out      ()              // output wire qplloutrefclk_out
);

assign CLKOUT = xgmii_clock;

generate
if (USE_CHIPSCOPE == 1) begin
  wire [255:0] probe0;
  ila256 ila256 (
      .clk(CLKOUT),
      .probe0(probe0)
  );
  // Status Signals
  assign probe0[7 :0] =  core_status;
  assign probe0[8]  =  resetdone;
  assign probe0[9]  =  qplllock;
  assign probe0[10] =  intSfpTxDisable;
  // Other Status
  assign probe0[11] = rst;
  assign probe0[12] = IP_SELECT[0];

  assign probe0[111:13] = 0;

  // XGMII
  assign probe0[119:112] = xgmii_txc;
  assign probe0[127:120] = xgmii_rxc;
  assign probe0[191:128] = xgmii_txd;
  assign probe0[255:192] = xgmii_rxd;
end
endgenerate

//------------------------------------------------------------------------------
endmodule
