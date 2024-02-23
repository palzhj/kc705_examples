`timescale 1ps/1ps
/*******************************************************************************
* System      : u4FCP GbE readout                                           *
* Version     : v 1.1 2024/02/12                                               *
*                                                                              *
* Description : Top Module                                                     *
*                                                                              *
* Designer    : zhj@ihep.ac.cn                                                 *
*                                                                              *
*******************************************************************************/
module top #(
  parameter         USE_CHIPSCOPE = 1,
  parameter [31:0]  SYN_DATE      = 32'h0, // the date of compiling
  parameter [7:0]   FPGA_VER      = 8'h1,         // the code version
  parameter [31:0]  BASE_IP_ADDR  = 32'hC0A8_0A10, // 192.168.10.16
  parameter [4 :0]  PHY_ADDRESS   = 5'b1,
  parameter [3 :0]  I2C_NUM       = 1,
  parameter [3 :0]  SPI_NUM       = 1,
  parameter [3 :0]  UART_NUM      = 1
)(
  input           CPU_RESET,
  input           SYSCLK_P,
  input           SYSCLK_N,
  input   [3 : 0] GPIO_DIP_SW,
  output  [7 : 0] GPIO_LED,
  inout           IIC_SCL_MAIN,
  inout           IIC_SDA_MAIN,
  output          SI5326_RST,
  output			    USER_SMA_GPIO_P,	  // out	: 156.25MHz
	output			    USER_SMA_GPIO_N,	  // out	: 156.25MHz
  // Tranceiver Interface
  // input           SGMIICLK_P,       // Differential +ve of reference clock for MGT: very high quality.
  // input           SGMIICLK_N,       // Differential -ve of reference clock for MGT: very high quality.
  input           SMA_MGT_REFCLK_P,
  input           SMA_MGT_REFCLK_N,
  output          SFP_TX_P,         // Differential +ve of serial transmission from PMA to PMD.
  output          SFP_TX_N,         // Differential -ve of serial transmission from PMA to PMD.
  input           SFP_RX_P,         // Differential +ve for serial reception from PMD to PMA.
  input           SFP_RX_N,         // Differential -ve for serial reception from PMD to PMA.
  input           SFP_LOS           // Input from PMD to indicate presence of optical input.
);

localparam DEBUG_SITCPXG  = 1;
localparam DEBUG_RBCP_REG = 1;

////////////////////////////////////////////////////////////////////////////////
//  Clock
wire clk200_in, clk200_int;
IBUFDS #(
  .DIFF_TERM    ("TRUE")
) IBUFDS_clk200 (
  .O            (clk200_in),
  .I            (SYSCLK_P),
  .IB           (SYSCLK_N)
);
BUFG BUFG_200 (
  .O            (clk200_int),
  .I            (clk200_in)
);

wire clk40_int, clk156_int, clk125_int, locked;
clk_wiz clk_wiz(
  // Clock in ports
  .clk_in1      (clk200_int),
  // Clock out ports
  .clk_out1     (clk40_int),
  .clk_out2     (clk156_int),
  .clk_out3     (clk125_int),
  // Status and control signals
  .resetn       (~CPU_RESET),
  .locked       (locked)
);

OBUFDS #(
  .IOSTANDARD		("LVDS_25"),        // Specify the output I/O standard
  .SLEW			    ("SLOW")            // Specify the output slew rate
) USER_CLK_OBUFDS (
  .O				    (USER_SMA_GPIO_P),  // Diff_p output (connect directly to top-level port)
  .OB				    (USER_SMA_GPIO_N),  // Diff_n output (connect directly to top-level port)
  .I				    (clk156_int)        // Buffer input
);

// An IDELAYCTRL primitive needs to be instantiated for the Fixed Tap Delay mode of the IDELAY.
wire dlyctrl_rdy;
IDELAYCTRL IDELAYCTRL_inst (
  .RDY        (dlyctrl_rdy),  // 1-bit output: Ready output
  .REFCLK     (clk200_int),   // 1-bit input: Reference clock input
  .RST        (CPU_RESET)        // 1-bit input: Active-High reset input. Asynchronous assert, synchronous deassert to REFCLK.
);

////////////////////////////////////////////////////////////////////////////////
// System clock and reset
wire usrclk, rst;
assign usrclk = clk125_int;

async2sync_reset reset_usrclk(
  .rst_in       (~(locked & dlyctrl_rdy)),
  .clk          (usrclk),
  .rst_out      (rst)
);

////////////////////////////////////////////////////////////////////////////////
// SiTCP interface
wire            tim_1s;

wire            tcp_open;
wire            tcp_rst;
wire            tcp_close;

wire  [15:0]  tcp_rx_size;    // Receive buffer size(byte). Caution: Set a value of 4000 or more and (memory size-16) or less
wire          tcp_rx_clr_enb; // Receive buffer Clear Enable
wire          tcp_rx_clr_req; // Receive buffer Clear Request
wire  [15:0]  tcp_rx_radr;    // Receive buffer read address in bytes (unused upper bits are set to 0)
wire  [15:0]  tcp_rx_wadr;    // Receive buffer write address in bytes (lower 3 bits are not connected to memory)
wire  [7 :0]  tcp_rx_wenb;    // Receive buffer byte write enable (big endian)
wire  [63:0]  tcp_rx_wdat;    // Receive buffer write data (big endian)
wire          tcp_tx_afull;   // TX fifo almost full
wire  [63:0]  tcp_tx_d;       // Tx data[63:0]
wire  [3 :0]  tcp_tx_b;       // Byte leng

wire    [31: 0] rbcp_addr;
wire            rbcp_we;
wire    [7 : 0] rbcp_wd;
wire            rbcp_re;
wire            rbcp_act;
wire            rbcp_ack;
wire    [7 : 0] rbcp_rd;

wire clk156;

sitcpxg #(
  .USE_CHIPSCOPE            (DEBUG_SITCPXG & USE_CHIPSCOPE),
  .BASE_IP_ADDR             (BASE_IP_ADDR),
  .MAC_IP_WIDTH             (3),
  .RxBufferSize             ("LongLong")
)sitcpxg_i(
  .RST                      (rst),
  .CLKOUT                   (clk156),
  .CLK40                    (clk40_int),
  .REG_FPGA_VER             (SYN_DATE),
  .REG_FPGA_ID              (32'b0),
  .MAC_SELECT               (0),
  .IP_SELECT                (0),
  .TIM_1US                  (),         // out: 1 us interval
  .TIM_10US                 (),
  .TIM_100US                (),
  .TIM_1MS                  (),         // out: 1 ms interval
  .TIM_10MS                 (),
  .TIM_100MS                (),
  .TIM_1S                   (tim_1s),   // out: 1 s interval
  .TIM_1M                   (),         // out: 1 min interval
  .SiTCP_RESET_OUT          (tcp_rst),
  // UDP
  .RBCP_ADDR                (rbcp_addr),
  .RBCP_WE                  (rbcp_we),
  .RBCP_WD                  (rbcp_wd),
  .RBCP_RE                  (rbcp_re),
  .RBCP_ACT                 (rbcp_act),
  .RBCP_ACK                 (rbcp_ack),
  .RBCP_RD                  (rbcp_rd),
  // TCP
  .USER_SESSION_OPEN_REQ    (1'b0),
  .USER_SESSION_ESTABLISHED (tcp_open),
  .USER_SESSION_CLOSE_REQ   (tcp_close),
  .USER_SESSION_CLOSE_ACK   (tcp_close),
  .USER_TX_D                (tcp_tx_d),
  .USER_TX_B                (tcp_tx_b),
  .USER_TX_AFULL            (tcp_tx_afull),
  .USER_RX_SIZE             (tcp_rx_size),
  .USER_RX_CLR_ENB          (tcp_rx_clr_enb),
  .USER_RX_CLR_REQ          (tcp_rx_clr_req),
  .USER_RX_RADR             (tcp_rx_radr),
  .USER_RX_WADR             (tcp_rx_wadr),
  .USER_RX_WENB             (tcp_rx_wenb),
  .USER_RX_WDAT             (tcp_rx_wdat),
  // PHY
  .GTREFCLK_P               (SMA_MGT_REFCLK_P),
  .GTREFCLK_N               (SMA_MGT_REFCLK_N),
  .GT_TXP                   (SFP_TX_P),
  .GT_TXN                   (SFP_TX_N),
  .GT_RXP                   (SFP_RX_P),
  .GT_RXN                   (SFP_RX_N)
);

////////////////////////////////////////////////////////////////////////////////
//  TCP test
wire [1 :0] tcp_mode;                   // 1: Loopback mode; 2: Test mode; Others: Normal mode
wire [7 :0] tcp_test_tx_rate;           // Transmission data rate in units of 100 Mbps
wire [63:0] tcp_test_num_of_data;       // Number of bytes of transmitted data
wire        tcp_test_data_gen;          // Data transmission enable
wire [2 :0] tcp_test_word_len;          // Word length of test data
wire        tcp_test_select_seq;        // Sequence Data select
wire [31:0] tcp_test_seq_pattern;       // sequence data (The default value is 0x60808040)
wire [23:0] tcp_test_blk_size;          // Transmission block size in bytes
wire        tcp_test_ins_error;         // Data error insertion

wire tcp_mode_loopback;
assign tcp_mode_loopback = (tcp_mode == 2'b01)? 1'b1: 1'b0;

wire tcp_mode_test;
assign tcp_mode_test = (tcp_mode == 2'b10)? 1'b1: 1'b0;

TCP_TEST TCP_TEST_i(
  .CLK156M             (clk156),
  .RSTs                (tcp_rst),
  .TX_RATE             (tcp_test_tx_rate),
  .NUM_OF_DATA         (tcp_test_num_of_data),
  .DATA_GEN            (tcp_test_data_gen),
  .LOOPBACK            (tcp_mode_loopback),
  .WORD_LEN            (tcp_test_word_len),
  .SELECT_SEQ          (tcp_test_select_seq),
  .SEQ_PATTERN         (tcp_test_seq_pattern),
  .BLK_SIZE            (tcp_test_blk_size),
  .INS_ERROR           (tcp_test_ins_error),
  .SiTCPXG_ESTABLISHED (tcp_open),
  .SiTCPXG_RX_SIZE     (tcp_rx_size),
  .SiTCPXG_RX_CLR_ENB  (tcp_rx_clr_enb),
  .SiTCPXG_RX_CLR_REQ  (tcp_rx_clr_req),
  .SiTCPXG_RX_RADR     (tcp_rx_radr),
  .SiTCPXG_RX_WADR     (tcp_rx_wadr),
  .SiTCPXG_RX_WENB     (tcp_rx_wenb),
  .SiTCPXG_RX_WDAT     (tcp_rx_wdat),
  .SiTCPXG_TX_AFULL    (tcp_tx_afull),
  .SiTCPXG_TX_D        (tcp_tx_d),
  .SiTCPXG_TX_B        (tcp_tx_b)
);


////////////////////////////////////////////////////////////////////////////////
//  Register controll

wire [I2C_NUM-1: 0] scl_i, sda_i, scl_o, sda_o, scl_oen, sda_oen;

RBCP_REG #(
  .USE_CHIPSCOPE(DEBUG_RBCP_REG & USE_CHIPSCOPE),
  .SYN_DATE     (SYN_DATE),
  .FPGA_VER     (FPGA_VER),
  .I2C_NUM      (I2C_NUM),
  .SPI_NUM      (SPI_NUM),
  .UART_NUM     (UART_NUM)
)RBCP_REG(
  // System
  .CLK          (clk156),     // in : System clock
  .RST          (tcp_rst),        // in : System reset
  // RBCP I/F
  .RBCP_ACT     (rbcp_act),       // in : Active
  .RBCP_ADDR    (rbcp_addr),      // in : Address[31:0]
  .RBCP_WE      (rbcp_we),        // in : Write enable
  .RBCP_WD      (rbcp_wd),        // in : Write data[7:0]
  .RBCP_RE      (rbcp_re),        // in : Read enable
  .RBCP_RD      (rbcp_rd),        // out  : Read data[7:0]
  .RBCP_ACK     (rbcp_ack),       // out  : Acknowledge
  // User IO
  .VP_IN        (),
  .VN_IN        (),
  .SCL          (scl_i),
  .SCL_OEN      (scl_oen),
  .SCL_O        (scl_o),
  .SDA          (sda_i),
  .SDA_OEN      (sda_oen),
  .SDA_O        (sda_o),
  .SCK          (),
  .MOSI_O       (),
  .MISO_I       (1'b0),
  .UART_RX      (1'b1),
  .UART_TX      (),
  .i_fpga_dna                   (64'b0),
  .o_tcp_mode                   (tcp_mode),
  .o_tcp_test_tx_rate           (tcp_test_tx_rate),
  .o_tcp_test_num_of_data       (tcp_test_num_of_data),
  .o_tcp_test_data_gen          (tcp_test_data_gen),
  .o_tcp_test_word_len          (tcp_test_word_len),
  .o_tcp_test_select_seq        (tcp_test_select_seq),
  .o_tcp_test_seq_pattern       (tcp_test_seq_pattern),
  .o_tcp_test_blk_size          (tcp_test_blk_size),
  .o_tcp_test_ins_error_trigger (tcp_test_ins_error)
);

assign scl_i[0] = IIC_SCL_MAIN;
assign IIC_SCL_MAIN = scl_oen[0] ? 1'bz: scl_o[0];

assign sda_i[0] = IIC_SDA_MAIN;
assign IIC_SDA_MAIN = sda_oen[0] ? 1'bz: sda_o[0];

//////////////////////////////////////////////////////////////////////////////
// Debug
reg ledr;
always @(posedge clk156)
  if(tim_1s) ledr <= ~ledr;

// assign BLED_B = ledr;
// assign GLED_B = ~tcp_open;
// assign RLED_B = ~rst;

assign GPIO_LED[0] = ledr;
assign GPIO_LED[1] = tcp_open;
assign GPIO_LED[2] = 1'b0;

endmodule
