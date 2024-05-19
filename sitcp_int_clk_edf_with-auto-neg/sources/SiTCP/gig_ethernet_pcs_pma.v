// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2022.2 (lin64) Build 3671981 Fri Oct 14 04:59:54 MDT 2022
// Date        : Sun May 19 23:33:03 2024
// Host        : iheper-GTR7-U22 running 64-bit Ubuntu 22.04.4 LTS
// Command     : write_verilog -mode synth_stub /home/iheper/Downloads/ver.v
// Design      : gig_ethernet_pcs_pma
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7k325tffg900-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module gig_ethernet_pcs_pma(gtgrefclk, gtrefclk_out, gtrefclk_bufg_out, 
  txp, txn, rxp, rxn, resetdone, userclk_out, userclk2_out, rxuserclk_out, rxuserclk2_out, 
  independent_clock_bufg, pma_reset_out, mmcm_locked_out, gmii_txd, gmii_tx_en, gmii_tx_er, 
  gmii_rxd, gmii_rx_dv, gmii_rx_er, gmii_isolate, configuration_vector, an_interrupt, 
  an_adv_config_vector, an_restart_config, status_vector, reset, signal_detect, 
  gt0_qplloutclk_out, gt0_qplloutrefclk_out)
/* synthesis syn_black_box black_box_pad_pin="gtgrefclk,gtrefclk_out,gtrefclk_bufg_out,txp,txn,rxp,rxn,resetdone,userclk_out,userclk2_out,rxuserclk_out,rxuserclk2_out,independent_clock_bufg,pma_reset_out,mmcm_locked_out,gmii_txd[7:0],gmii_tx_en,gmii_tx_er,gmii_rxd[7:0],gmii_rx_dv,gmii_rx_er,gmii_isolate,configuration_vector[4:0],an_interrupt,an_adv_config_vector[15:0],an_restart_config,status_vector[15:0],reset,signal_detect,gt0_qplloutclk_out,gt0_qplloutrefclk_out" */;
  input gtgrefclk;
  output gtrefclk_out;
  output gtrefclk_bufg_out;
  output txp;
  output txn;
  input rxp;
  input rxn;
  output resetdone;
  output userclk_out;
  output userclk2_out;
  output rxuserclk_out;
  output rxuserclk2_out;
  input independent_clock_bufg;
  output pma_reset_out;
  output mmcm_locked_out;
  input [7:0]gmii_txd;
  input gmii_tx_en;
  input gmii_tx_er;
  output [7:0]gmii_rxd;
  output gmii_rx_dv;
  output gmii_rx_er;
  output gmii_isolate;
  input [4:0]configuration_vector;
  output an_interrupt;
  input [15:0]an_adv_config_vector;
  input an_restart_config;
  output [15:0]status_vector;
  input reset;
  input signal_detect;
  output gt0_qplloutclk_out;
  output gt0_qplloutrefclk_out;
endmodule
