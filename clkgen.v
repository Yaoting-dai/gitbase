//////////////////////////////////////////////////////////////////////////////
// This design is confidential and proprietary of Yaoting, All Rights Reserved.
//////////////////////////////////////////////////////////////////////////////
// Version:         1.0
// Filename:        clk_gen .v
// Date Created:    12/07/2022
// Device:          ZYNQ7020/ZYNQ7030/ZYNQ7035+
// IDE:             Vivado 2018.3+
// Purpose:         Design for LVDS interface clk timing.
//////////////////////////////////////////////////////////////////////////////

`timescale 1ps/1ps

module clkgen # (
      parameter real    CLKIN_PERIOD = 6.600,        // Clock period (ns) of input clock on clkin_p
      parameter         CLK_PATTERN  = 7'b110_0011,   // Clock pattern for alignment
   )
   (
      input             clkin_p,              // Clock input LVDS P-side
      input             clkin_n,              // Clock input LVDS N-side
      input             reset,                // Asynchronous interface reset

      output            rx_clkdiv2,           // RX clock div2 output
      output            rx_clkdiv6,           // RX clock div6 output

      output            cmt_locked,           // PLL/MMCM locked output
      output     [4:0]  rx_wr_addr,           // RX write_address output
      output            rx_reset,             // RX reset output
      output reg        rx_ready,             // RX ready output

      output            px_clk,               // Pixel clock output
      output     [4:0]  px_rd_addr,           // Pixel read address output
      output reg [2:0]  px_rd_seq,            // Pixel read sequence output
      output reg        px_ready              // Pixel data ready output
   );

wire       px_reset;        // pixel asynchronous reset
wire       clkin_p_i;       
wire       clkin_n_i;       
wire       mmcm_px;         // Mixed Mode clock Manager clock output
wire       mmcm_div2;       // Mixed Mode clock Manager clock output

reg  [3:0] rx_reset_sync;
reg  [3:0] px_reset_sync;
//-----------------------------------------------------------------------------
// Clock input global diff buffer
BUFG 
//-----------------------------------------------------------------------------
// Clock input global diff buffer
IBUFGDS_DIFF_OUT # ( .DIFF_TERM   (DIFF_TERM) )
   clk_input (
      .I                (clkin_p),
      .IB               (clkin_n),
      .O                (clkin_p_i),
      .OB               (clkin_n_i)
   );
//-----------------------------------------------------------------------------
// Instantitate a MMCM
generate
begin
   MMCME3_BASE # (
         .BANDWIDTH          ("OPTIMIZED"),             // String Vaule, "OPTIMIZED","HIGH","LOW"; default "OPTIMIZED"
         .CLKFBOUT_MULT_F    (6*VCO_MULTIPLIER),        // Spicifies the amount to multiply all CLKOUT clock outputs if a different frequency is desired;2.000 to 64.000,default 5.000
         .CLKFBOUT_PHASE     (0.0),                     // Specifies the phase offset in degrees of the clock feedback output; -360.000 to 360.000,default 0.000
         .CLKIN1_PERIOD      (CLKIN_PERIOD),            // Specifies the input period in ns to the MMCM CLKIN1 input; 0.000 to 100.000 nS
         .CLKOUT0_DIVIDE_F   (2*VCO_MULTIPLIER),        // Specified the amount to divide the associated CLKOUT clock output if a different frequency is desired; 1.000 to 128.000, default 1.000
         .CLKOUT4_CASCADE    ("FALSE"),                 // Cascades the output divider (counter) CLKOUT6 into the input of the CLKOUT4 divider for an output clock divider that is greater than 128; String,"FALSE","TRUE",default "FALSE"
         .DIVCLK_DIVIDE      (1),                       // Specifies the division ratio for all output clocks with respect to the input clock;1 to 106,default 1
         .REF_JITTER1(0.0),                             // Reference input jitter in UI (0.000-0.999)    
         .STARTUP_WAIT("FALSE"),                        // Delays DONE until MMCM is locked (FALSE, TRUE)
         .CLKOUT0_DUTY_CYCLE (0.5),                     // Specifies the duty cycle for CLKOUT0;0.001 to 0.999,default 0.5
         .CLKOUT1_DUTY_CYCLE (),
         .CLKOUT2_DUTY_CYCLE (),
         .CLKOUT3_DUTY_CYCLE (),
         .CLKOUT4_DUTY_CYCLE (),
         .CLKOUT5_DUTY_CYCLE (),
         .CLKOUT6_DUTY_CYCLE (),
         .CLKOUT0_PHASE      (0.0),                     //Specifies the offset for CLKOUT0;-360.000 to 360.000,default 0.000
         .CLKOUT1_PHASE      (),
         .CLKOUT2_PHASE      (),
         .CLKOUT3_PHASE      (),
         .CLKOUT4_PHASE      (),
         .CLKOUT5_PHASE      (),
         .CLKOUT6_PHASE      (),
         .CLKOUT1_DIVIDE     (),                        // Specifies the amount to divide CLKOUT1 to create a different frequency;1 to 128,default 1
         .CLKOUT2_DIVIDE     (),
         .CLKOUT3_DIVIDE     (),
         .CLKOUT4_DIVIDE     (),
         .CLKOUT5_DIVIDE     (),
         .CLKOUT6_DIVIDE     (),
   // Programmable Inversion Attributes: Specifies built-in programmable inversion on specific pins    
         .IS_CLKFBIN_INVERTED(1'b0),                    // Optional inversion for CLKFBIN
         .IS_CLKIN1_INVERTED(1'b0),                     // Optional inversion for CLKIN1    
         .IS_PWRDWN_INVERTED(1'b0),                     // Optional inversion for PWRDWN    
         .IS_RST_INVERTED(1'b0)                        // Optional inversion for RST
    )
   mmcm0 (
   // Input    
         .CLKIN1         (clkin_p_i),     // General clock input
         .CLKFBIN        (px_clk),        // Feedback clock pin to the MMCM
         .PWRDWN         (1'b0),          // Powers down instantiated but unused MMCMs
         .RST            (reset),         // Asynchronous reset signal 
   // Output
         .CLKOUT0        (mmcm_div2),     // CLKOUT0 output
         .CLKOUT0B       (),              // Inverted CLKOUT0 output
         .CLKOUT1        (),              // CLKOUT1 output
         .CLKOUT1B       (),              // Inverted CLKOUT1 output
         .CLKOUT2        (),              // CLKOUT2 output
         .CLKOUT2B       (),              // Inverted CLKOUT2 output
         .CLKOUT3        (),              // CLKOUT3 output
         .CLKOUT3B       (),              // Inverted CLKOUT3 output
         .CLKOUT4        (),              // CLKOUT4 output
         .CLKOUT5        (),              // CLKOUT5 output
         .CLKOUT6        (),              // CLKOUT6 output
         .CLKFBOUT       (mmcm_px),       // Dedicated MMCM feedback clock output
         .CLKFBOUTB      (),              // Inverted CLKFBOUT output
         .LOCKED         (cmt_locked)     // MMCM nust be rest after LOCKED is desserted
    );
end
endgenerate
//-----------------------------------------------------------------------------
// Global Clock Buffers
BUFG  px     (.I(mmcm_px),      .O(px_clk)) ;

BUFG  div2   (.I(mmcm_div2), .O(rx_clkdiv2)) ;

BUFGCE_DIV  # ( .BUFGCE_DIVIDE(3))
      clkdiv8 (
       .I(mmcm_div2),
       .CLR(!cmt_locked),
       .CE(1'b1),
       .O(rx_clkdiv6)
);
//-----------------------------------------------------------------------------
// Asynchronous reset for px_ready output
always @ (posedge px_clk or posedge px_reset) begin
   if (px_reset) begin
      px_ready   <= 3'h0;
   end
   else begin
      px_ready   <= px_ready_int;
   end
end
//-----------------------------------------------------------------------------
// Clock Master Side ISERDES
ISERDESE3 #(
       .DATA_WIDTH     (6),
       .FIFO_ENABLE    ("FALSE"),
       .FIFO_SYNC_MODE ("FALSE"),
       .SIM_DEVICE     (SIM_DEVICE)
   )
   iserdes_m (
       .D              (),
       .RST            (rx_reset),
       .CLK            ( rx_clkdiv2),
       .CLK_B          (~rx_clkdiv2),
       .CLKDIV         ( rx_clkdiv6),
       .Q              (Mstr_Data),
       .FIFO_RD_CLK    (1'b0),
       .FIFO_RD_EN     (1'b0),
       .FIFO_EMPTY     (),
       .INTERNAL_DIVCLK()
   );
//-----------------------------------------------------------------------------
// Clock Slave Side ISERDES
ISERDESE3 #(
       .DATA_WIDTH     (6),
       .FIFO_ENABLE    ("FALSE"),
       .FIFO_SYNC_MODE ("FALSE"),
       .SIM_DEVICE     (SIM_DEVICE)
   )
   iserdes_s (
       .D              (),
       .RST            (rx_reset),
       .CLK            ( rx_clkdiv2),
       .CLK_B          (~rx_clkdiv2),
       .CLKDIV         ( rx_clkdiv6),
       .Q              (Slve_Data_inv),
       .FIFO_RD_CLK    (1'b0),
       .FIFO_RD_EN     (1'b0),
       .FIFO_EMPTY     (),
       .INTERNAL_DIVCLK()
   );
//-----------------------------------------------------------------------------
// Synchronize locked to rx_clkdiv6
always @ (posedge rx_clkdiv6 or negedge cmt_locked)
begin
   if (!cmt_locked)
       rx_reset_sync <= 4'b1111;
   else
       rx_reset_sync <= {1'b0,rx_reset_sync[3:1]};
end
assign rx_reset = rx_reset_sync[0];
//-----------------------------------------------------------------------------
// Synchronize rx_reset to px_clk
always @ (posedge px_clk or posedge rx_reset)
begin
   if (rx_reset)
       px_reset_sync <= 4'b1111;
   else
       px_reset_sync <= {1'b0,px_reset_sync[3:1]};
end
assign px_reset = px_reset_sync[0];
//-----------------------------------------------------------------------------
//  RX write address counter
always @ (posedge rx_clkdiv6)
begin
    if (rx_reset)
        rx_wr_count <= 5'h4;
    else
        rx_wr_count <= rx_wr_count + 1'b1;
end
assign rx_wr_addr   = rx_wr_count;
//-----------------------------------------------------------------------------
//  Pixel read address counter
always @ (posedge px_clk)
begin
    if (px_reset) begin
       px_rd_count <= 5'h0;
       end
    else if (px_rd_seq != 3'h6) begin
       // Increment counter except when the read sequence is 6
       px_rd_count <= px_rd_count + 1'b1;
    end
end
assign px_rd_addr = px_rd_count;
//-----------------------------------------------------------------------------
// Register data from ISERDES
always @ (posedge rx_clkdiv6)
begin
   rx_wr_data <= Mstr_Data;
end
//-----------------------------------------------------------------------------
// Generate 8 Dual Port Distributed RAMS for FIFO
genvar i;
generate
for (i = 0 ; i < 8 ; i = i+1) begin : bit
  RAM32X1D fifo (
     .D     (rx_wr_data[i]),
     .WCLK  (rx_clkdiv6),
     .WE    (1'b1),
     .A4    (rx_wr_addr[4]),
     .A3    (rx_wr_addr[3]),
     .A2    (rx_wr_addr[2]),
     .A1    (rx_wr_addr[1]),
     .A0    (rx_wr_addr[0]),
     .SPO   (),
     .DPRA4 (px_rd_addr[4]),
     .DPRA3 (px_rd_addr[3]),
     .DPRA2 (px_rd_addr[2]),
     .DPRA1 (px_rd_addr[1]),
     .DPRA0 (px_rd_addr[0]),
     .DPO   (px_rd_curr[i]));
end
endgenerate
//-----------------------------------------------------------------------------
// Store last read pixel data for one cycle, bit 0 is not required
always @ (posedge px_clk)
begin
    px_rd_last[7:1] <= px_rd_curr[7:1];
end
//-----------------------------------------------------------------------------
// Pixel 8-to-7 gearbox
always @ (posedge px_clk)
begin
    if (px_reset) begin
       px_rd_seq <= 3'b0;
       end
    else  begin
       px_rd_seq <= px_rd_seq + px_rd_enable;
       case (px_rd_seq )
         3'h0 : begin
            px_data <= px_rd_curr[6:0];
            end
         3'h1 : begin
            px_data <= {px_rd_curr[5:0], px_rd_last[7]};
            end
         3'h2 : begin
            px_data <= {px_rd_curr[4:0], px_rd_last[7:6]};
            end
         3'h3 : begin
            px_data <= {px_rd_curr[3:0], px_rd_last[7:5]};
            end
         3'h4 : begin
            px_data <= {px_rd_curr[2:0], px_rd_last[7:4]};
            end
         3'h5 : begin
            px_data <= {px_rd_curr[1:0], px_rd_last[7:3]};
            end
         3'h6 : begin
            px_data <= {px_rd_curr[0],   px_rd_last[7:2]};
            end
         3'h7 : begin
            px_data <= {px_rd_last[7:1]};
            end
       endcase
    end
end
//-----------------------------------------------------------------------------
// Synchronize rx_ready to px_clk
always @ (posedge px_clk or negedge rx_ready) //cr993494
begin
    if (!rx_ready) begin
       px_rx_ready_sync <= 4'b0;
       end
    else begin
       px_rx_ready_sync <= {1'b1, px_rx_ready_sync[3:1]};
    end
end
assign px_rx_ready = px_rx_ready_sync[0];
//-----------------------------------------------------------------------------
// Pixel alignment state machine
always @ (posedge px_clk) begin
   if (px_reset) begin
      px_state          <= 3'h1;
      px_rd_enable      <= 1'b0;
      px_ready_int      <= 1'b0;
      px_correct        <= 3'h0;
   end else if (px_rx_ready) begin
      //
      // Pixel alignment state machine
      //
      case (px_state)
         //
         // 0x0 - Check alignment
         //
         3'h0: begin
            if (px_data == CLK_PATTERN) begin
               px_rd_enable <= 1'b1;            // Enable read sequencer
               px_correct   <= px_correct + 1'b1;
               if (px_correct == 3'h7) begin
                  px_state  <= 3'h7;            // 0x7 - End state
               end
            end
            else begin
               px_correct   <= 3'h0;
               px_rd_enable <= 1'b0;            // Disable read sequencer to slip alignment
               px_state     <= px_state + 1'b1; // 0x1 - Re-enable and wait 6 cycles
            end
         end
         //
         // 0x1 - Re-enable read sequencer
         //
         3'h1: begin
            px_rd_enable  <= 1'b1;              // Enable read sequencer
            px_state      <= px_state + 1'b1;   // Increment to next state
         end
         //
         // 0x6 - Return to alignment check
         //
         3'h6: begin
            px_rd_enable  <= 1'b1;              // Enable ready sequencer
            px_state      <= 3'h0;              // 0x0 - Check alignment
         end
         //
         // 0x7 - End state
         //
         3'h7 : begin
            px_rd_enable  <= 1'b1;              // Enable ready sequencer
            px_ready_int  <= 1'b1;              // Assert pixel ready
         end
         //
         // Default state
         //
         default: begin
            px_rd_enable  <= 1'b1;              // Enable read sequencer
            px_state      <= px_state + 1'b1;   // Increment to next state
         end
      endcase
   end
end
endmodule