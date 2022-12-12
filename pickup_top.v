//////////////////////////////////////////////////////////////////////////////
// This design is confidential and proprietary of Yaoting, All Rights Reserved.
//////////////////////////////////////////////////////////////////////////////
// Version:         1.0
// Filename:        pickup_top .v
// Date Created:    12/07/2022
// Device:          ZYNQ7020
// IDE:             Vivado 2018.3
// Purpose:         Design for LVDS interface
//////////////////////////////////////////////////////////////////////////////

`timescale 1ps/1ps

module pickup_top 
# (
    parameter CHANNELS = 2,      // input data pairs.
    parameter DATA_MODE = "DDR", // DDR or SDR.
    parameter PER_BIT   = 6,     // serial bits per channel per cycle.
    parameter MAXBIT = 12,       // MAXBIT = CHANNELS * PER_BIT.
    parameter C_TYPE = "SPI"     // config type :SPI 3-wire or 4-wire.

    )
(
    input    reset,
    input    clk,                                 // sample clock.
    input    enable,                              // enable bit setting.
    input    [CHANNELS-1 : 0]  channel_data,      // input lvds channels.

    input    clkin_p,                           // data T/R bit clock positive.
    input    clkin_n,                           // data T/R bit clock negative.
    input    frameclk_p,                          // cycle period.
    input    frameclk_n,

    output   [MAXBIT:1] received_data,            // output FIFO data per cycle.
    output   [2*MAXBIT/8-1 :0] read_addr
)





endmodule