`timescale 1ps/1ps
/*******************************************************************************
* System      : uRTM GbE readout                                           *
* Version     : v 1.0 2023/09/30                                               *
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
  // Tranceiver Interface
  input           SGMIICLK_P,       // Differential +ve of reference clock for MGT: very high quality.
  input           SGMIICLK_N,       // Differential -ve of reference clock for MGT: very high quality.
  output          SFP_TX_P,         // Differential +ve of serial transmission from PMA to PMD.
  output          SFP_TX_N,         // Differential -ve of serial transmission from PMA to PMD.
  input           SFP_RX_P,         // Differential +ve for serial reception from PMD to PMA.
  input           SFP_RX_N,         // Differential -ve for serial reception from PMD to PMA.
  input           SFP_LOS           // Input from PMD to indicate presence of optical input.
);

localparam DEBUG_SITCP    = 1;
localparam DEBUG_RBCP_REG = 0;

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

wire clk40_int, clk100_int, clk125_int, locked;
clk_wiz clk_wiz(
  // Clock in ports
  .clk_in1      (clk200_int),
  // Clock out ports
  .clk_out1     (clk40_int),
  .clk_out2     (clk100_int),
  .clk_out3     (clk125_int),
  // Status and control signals
  .resetn       (~CPU_RESET),
  .locked       (locked)
);

// An IDELAYCTRL primitive needs to be instantiated for the Fixed Tap Delay
// mode of the IDELAY.
wire dlyctrl_rdy;
IDELAYCTRL dlyctrl (
  .RDY          (dlyctrl_rdy),
  .REFCLK       (clk200_int),
  .RST          (CPU_RESET)
);

assign SI5326_RST = 1'b1;

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
wire            tcp_error;
wire            tcp_rst;
wire            tcp_close;

wire            tcp_rx_wr;
wire    [7 : 0] tcp_rx_data;
wire    [15: 0] tcp_rx_wc;

wire            tcp_tx_wr;
wire    [7 : 0] tcp_tx_data;
wire            tcp_tx_full;

wire    [31: 0] rbcp_addr;
wire            rbcp_we;
wire    [7 : 0] rbcp_wd;
wire            rbcp_re;
wire            rbcp_act;
wire            rbcp_ack;
wire    [7 : 0] rbcp_rd;

SITCP #(
  .USE_CHIPSCOPE(DEBUG_SITCP & USE_CHIPSCOPE),
  .BASE_IP_ADDR (BASE_IP_ADDR),
  .PHY_ADDRESS  (PHY_ADDRESS),
  .MAC_IP_WIDTH (4)
)sitcp(
  .RST          (rst),
  .USRCLK       (clk125_int),
  .CLK40        (clk40_int),
  .CLKOUT       (),
  .MAC_SELECT   (0),
  .IP_SELECT    (0),
  .TIM_1US      (),         // out: 1 us interval
  .TIM_10US     (),
  .TIM_100US    (),
  .TIM_1MS      (),         // out: 1 ms interval
  .TIM_10MS     (),
  .TIM_100MS    (),
  .TIM_1S       (tim_1s),   // out: 1 s interval
  .TIM_1M       (),         // out: 1 min interval
  // TCP
  .TCP_OPEN     (tcp_open),
  .TCP_ERROR    (tcp_error),
  .TCP_RST      (tcp_rst),
  .TCP_CLOSE    (tcp_close),
  .TCP_RX_WC    (tcp_rx_wc[15:0]),
  .TCP_RX_WR    (tcp_rx_wr),
  .TCP_RX_DATA  (tcp_rx_data),
  .TCP_TX_FULL  (tcp_tx_full),
  .TCP_TX_WR    (tcp_tx_wr),
  .TCP_TX_DATA  (tcp_tx_data),
  // UDP
  .RBCP_ADDR    (rbcp_addr),
  .RBCP_WE      (rbcp_we),
  .RBCP_WD      (rbcp_wd),
  .RBCP_RE      (rbcp_re),
  .RBCP_ACT     (rbcp_act),
  .RBCP_ACK     (rbcp_ack),
  .RBCP_RD      (rbcp_rd),
  // PHY
  // .PHY_RSTn     (PHY_RST_B),
  // .RGMII_TXC    (PHY_TX_CLK),
  // .RGMII_TXD    (PHY_TX_D),
  // .RGMII_TX_CTL (PHY_TX_CTRL),
  // .RGMII_RXC    (PHY_RX_CLK),
  // .RGMII_RXD    (PHY_RX_D),
  // .RGMII_RX_CTL (PHY_RX_CTRL),
  // .MDC          (PHY_MDC),
  // .MDIO         (PHY_MDIO)
  .GT_REFCLKP              (SGMIICLK_P),
  .GT_REFCLKN              (SGMIICLK_N),
  .GT_TXP                (SFP_TX_P),
  .GT_TXN                (SFP_TX_N),
  .GT_RXP                (SFP_RX_P),
  .GT_RXN                (SFP_RX_N),
  .GT_DETECT             (~SFP_LOS)  // Input from PMD to indicate presence of optical input.
);

////////////////////////////////////////////////////////////////////////////////
//  TCP loopback FIFO
wire sitcpFifoEmpty, sitcpFifoRe;
assign tcp_rx_wc[15:11] = 5'b11111;

sitcp_fifo sitcp_fifo(
  .clk          (clk125_int),
  .srst         (~tcp_open),
  .data_count   (tcp_rx_wc[10:0]),
  .full         (),
  .wr_en        (tcp_rx_wr),
  .din          (tcp_rx_data[7:0]),
  .empty        (sitcpFifoEmpty),
  .rd_en        (sitcpFifoRe),
  .dout         (tcp_tx_data[7:0]),
  .valid        (tcp_tx_wr)
);

assign  sitcpFifoRe = ~tcp_tx_full & ~sitcpFifoEmpty;

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
  .CLK          (clk125_int),     // in : System clock
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
  .UART_TX      ()
);

assign scl_i[0] = IIC_SCL_MAIN;
assign IIC_SCL_MAIN = scl_oen[0] ? 1'bz: scl_o[0];

assign sda_i[0] = IIC_SDA_MAIN;
assign IIC_SDA_MAIN = sda_oen[0] ? 1'bz: sda_o[0];

////////////////////////////////////////////////////////////////////////////////
// Debug
reg ledr;
always @(posedge clk125_int)
  if(tim_1s) ledr <= ~ledr;

// assign BLED_B = ledr;
// assign GLED_B = ~tcp_open;
// assign RLED_B = ~tcp_error;

assign GPIO_LED[0] = ledr;
assign GPIO_LED[1] = ~tcp_open;
assign GPIO_LED[2] = ~tcp_error;

endmodule
