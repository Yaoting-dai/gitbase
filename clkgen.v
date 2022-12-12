
`timescale 1ps/1ps

module clkgen # (
      parameter real    CLKIN_PERIOD = 6.600,        // Clock period (ns) of input clock on clkin_p
      //parameter real    REF_FREQ     = 300.0,        // Reference clock frequency for idelay control
      parameter         CLK_PATTERN  = 7'b110_0011,   // Clock pattern for alignment
   )
   (
      input             clkin_p,              // Clock input LVDS P-side
      input             clkin_n,              // Clock input LVDS N-side
      input             reset,                // Asynchronous interface reset
      //input             idelay_rdy,           // Asyncrhonous IDELAYCRL ready
      //
      output            rx_clkdiv2,           // RX clock div2 output
      output            rx_clkdiv8,           // RX clock div8 output
      output            cmt_locked,           // PLL/MMCM locked output
      output     [4:0]  rx_wr_addr,           // RX write_address output
      //output reg [8:0]  rx_cntval,            // RX input delay count value output
     // output reg        rx_dlyload,           // RX input delay load output
      output            rx_reset,             // RX reset output
      output reg        rx_ready,             // RX ready output
      //
      output            px_clk,               // Pixel clock output
      output     [4:0]  px_rd_addr,           // Pixel read address output
      output reg [2:0]  px_rd_seq,            // Pixel read sequence output
      output reg        px_ready              // Pixel data ready output
   );

//
// Set VCO multiplier for PLL/MMCM 
//  2  - if clock_period is greater than 600 MHz/7 
//  1  - if clock period is <= 600 MHz/7  
//
localparam VCO_MULTIPLIER = (CLKIN_PERIOD >11.666) ? 2 : 1 ;
//localparam DELAY_VALUE    = ((CLKIN_PERIOD*1000)/7 <= 1250.0) ? (CLKIN_PERIOD*1000)/7 : 1250.0;
//localparam DELAY_VALUE    = (SIM_DEVICE = "ULTRASCALE") ? (((CLKIN_PERIOD*1000)/7 <= 1250.0 ) ? (CLKIN_PERIOD*1000)/7 : 1250.0) : 0;
//localparam DELAY_VALUE    = (((CLKIN_PERIOD*1000)/7 <= 1250.0) && SIM_DEVICE = "ULTRASCALE") ? (CLKIN_PERIOD*1000)/7 : 1250.0;

wire       clkin_p_i;
wire       clkin_n_i;
wire       clkin_p_d;
wire       clkin_n_d;
wire       px_pllmmcm;
wire       rx_pllmmcm_div2;

wire       locked_and_idlyrdy;
reg  [3:0] rx_reset_sync;
reg  [3:0] px_reset_sync;
wire       px_reset;

reg        Mstr_Load;
reg        Mstr_IdlyRst;
reg  [8:0] Mstr_CntVal_In;
wire [8:0] Mstr_CntVal_Out;
wire [7:0] Mstr_Data;
reg        Slve_Load;
reg  [8:0] Slve_CntVal_In;
wire [7:0] Slve_Data;
wire [7:0] Slve_Data_inv;
reg        Slve_Less;

reg  [8:0] BitTime_Val;
reg  [5:0] rx_state;
reg  [7:0] Delay_Change;
reg  [4:0] pd_count;
reg        pd_ovflw_up;
reg        pd_ovflw_down;
wire       PhaseDet_Inc;
wire       PhaseDet_Dec;
reg        MstrCnt_GE_BTval;
reg        MstrCnt_LE_Seven;
reg        MstrCnt_GE_HalfBT;
reg  [7:0] Update_Seq;

reg  [4:0] rx_wr_count;
reg  [7:0] rx_wr_data;

reg  [3:0] px_rx_ready_sync;
wire       px_rx_ready;
reg  [4:0] px_rd_count;
wire [7:0] px_rd_curr;
reg  [7:1] px_rd_last;
reg  [6:0] px_data;
reg  [2:0] px_state;
reg  [2:0] px_correct;
reg        px_rd_enable;
wire [3:0] px_rd_addr_int;
reg        px_ready_int;

//
// Clock input
//
IBUFGDS_DIFF_OUT # ( .DIFF_TERM   (DIFF_TERM) )
   clk_input (
      .I                (clkin_p),
      .IB               (clkin_n),
      .O                (clkin_p_i),
      .OB               (clkin_n_i)
   );

//
// Instantitate a MMCM
//
generate
begin    // use an MMCM
   MMCME3_BASE # (
         .CLKIN1_PERIOD      (CLKIN_PERIOD),
         .BANDWIDTH          ("OPTIMIZED"),
         .CLKFBOUT_MULT_F    (7*VCO_MULTIPLIER),
         .CLKFBOUT_PHASE     (0.0),
         .CLKOUT0_DIVIDE_F   (2*VCO_MULTIPLIER),
         .CLKOUT0_DUTY_CYCLE (0.5),
         .CLKOUT0_PHASE      (0.0),
         .DIVCLK_DIVIDE      (1),
         .REF_JITTER1        (0.100)
      )
      rx_mmcm_adv_inst (
         .CLKFBOUT       (px_pllmmcm),
         .CLKFBOUTB      (),
         .CLKOUT0        (rx_pllmmcm_div2),
         .CLKOUT0B       (),
         .CLKOUT1        (),
         .CLKOUT1B       (),
         .CLKOUT2        (),
         .CLKOUT2B       (),
         .CLKOUT3        (),
         .CLKOUT3B       (),
         .CLKOUT4        (),
         .CLKOUT5        (),
         .CLKOUT6        (),
         .LOCKED         (cmt_locked),
         .CLKFBIN        (px_clk),
         .CLKIN1         (clkin_p_i),
         .PWRDWN         (1'b0),
         .RST            (reset)
     );
   end
endgenerate

//
// Global Clock Buffers
//
BUFG  bg_px     (.I(px_pllmmcm),      .O(px_clk)) ;
BUFG  bg_rxdiv2 (.I(rx_pllmmcm_div2), .O(rx_clkdiv2)) ;
BUFGCE_DIV  # ( .BUFGCE_DIVIDE(4))
   bg_rxdiv8 (
       .I(rx_pllmmcm_div2),
       .CLR(!cmt_locked),
       .CE(1'b1),
       .O(rx_clkdiv8)
   );

//
// Synchronize locked to rx_clkdiv8
//
assign locked_and_idlyrdy = cmt_locked & idelay_rdy;
always @ (posedge rx_clkdiv8 or negedge locked_and_idlyrdy)
begin
   if (!locked_and_idlyrdy)
       rx_reset_sync <= 4'b1111;
   else
       rx_reset_sync <= {1'b0,rx_reset_sync[3:1]};
end
assign rx_reset = rx_reset_sync[0];

//
// Synchronize rx_reset to px_clk
//
always @ (posedge px_clk or posedge rx_reset)
begin
   if (rx_reset)
       px_reset_sync <= 4'b1111;
   else
       px_reset_sync <= {1'b0,px_reset_sync[3:1]};
end
assign px_reset = px_reset_sync[0];

//-----------------------------------------------------------------------------
//
// Clock Master Side ISERDES
//
ISERDESE3 #(
       .DATA_WIDTH     (8),
       .FIFO_ENABLE    ("FALSE"),
       .FIFO_SYNC_MODE ("FALSE"),
       .SIM_DEVICE     (SIM_DEVICE)
   )
   iserdes_m (
       .D              (clkin_p_d),
       .RST            (rx_reset),
       .CLK            ( rx_clkdiv2),
       .CLK_B          (~rx_clkdiv2),
       .CLKDIV         ( rx_clkdiv8),
       .Q              (Mstr_Data),
       .FIFO_RD_CLK    (1'b0),
       .FIFO_RD_EN     (1'b0),
       .FIFO_EMPTY     (),
       .INTERNAL_DIVCLK()
   );


//-----------------------------------------------------------------------------
//
// Clock Slave Side ISERDES
//
ISERDESE3 #(
       .DATA_WIDTH     (8),
       .FIFO_ENABLE    ("FALSE"),
       .FIFO_SYNC_MODE ("FALSE"),
       .SIM_DEVICE     (SIM_DEVICE)
   )
   iserdes_s (
       .D              (clkin_n_d),
       .RST            (rx_reset),
       .CLK            ( rx_clkdiv2),
       .CLK_B          (~rx_clkdiv2),
       .CLKDIV         ( rx_clkdiv8),
       .Q              (Slve_Data_inv),
       .FIFO_RD_CLK    (1'b0),
       .FIFO_RD_EN     (1'b0),
       .FIFO_EMPTY     (),
       .INTERNAL_DIVCLK()
   );

assign Slve_Data = ~Slve_Data_inv;  // Invert slave data


assign PhaseDet_Inc =
        ( Slve_Less & ((~Mstr_Data[0] &  Slve_Data[0] & Mstr_Data[1]) |
                       (~Mstr_Data[1] &  Slve_Data[1] & Mstr_Data[2]) |
                       (~Mstr_Data[2] &  Slve_Data[2] & Mstr_Data[3]) |
                       (~Mstr_Data[3] &  Slve_Data[3] & Mstr_Data[4]) |
                       (~Mstr_Data[4] &  Slve_Data[4] & Mstr_Data[5]) |
                       (~Mstr_Data[5] &  Slve_Data[5] & Mstr_Data[6]) |
                       (~Mstr_Data[6] &  Slve_Data[6] & Mstr_Data[7]))) |
        (~Slve_Less & ((~Mstr_Data[0] &  Slve_Data[1] & Mstr_Data[1]) |
                       (~Mstr_Data[1] &  Slve_Data[2] & Mstr_Data[2]) |
                       (~Mstr_Data[2] &  Slve_Data[3] & Mstr_Data[3]) |
                       (~Mstr_Data[3] &  Slve_Data[4] & Mstr_Data[4]) |
                       (~Mstr_Data[4] &  Slve_Data[5] & Mstr_Data[5]) |
                       (~Mstr_Data[5] &  Slve_Data[6] & Mstr_Data[6]) |
                       (~Mstr_Data[6] &  Slve_Data[7] & Mstr_Data[7])));

assign PhaseDet_Dec =
        ( Slve_Less & ((~Mstr_Data[0] & ~Slve_Data[0] & Mstr_Data[1]) |
                       (~Mstr_Data[1] & ~Slve_Data[1] & Mstr_Data[2]) |
                       (~Mstr_Data[2] & ~Slve_Data[2] & Mstr_Data[3]) |
                       (~Mstr_Data[3] & ~Slve_Data[3] & Mstr_Data[4]) |
                       (~Mstr_Data[4] & ~Slve_Data[4] & Mstr_Data[5]) |
                       (~Mstr_Data[5] & ~Slve_Data[5] & Mstr_Data[6]) |
                       (~Mstr_Data[6] & ~Slve_Data[6] & Mstr_Data[7]))) |
        (~Slve_Less & ((~Mstr_Data[0] & ~Slve_Data[1] & Mstr_Data[1]) |
                       (~Mstr_Data[1] & ~Slve_Data[2] & Mstr_Data[2]) |
                       (~Mstr_Data[2] & ~Slve_Data[3] & Mstr_Data[3]) |
                       (~Mstr_Data[3] & ~Slve_Data[4] & Mstr_Data[4]) |
                       (~Mstr_Data[4] & ~Slve_Data[5] & Mstr_Data[5]) |
                       (~Mstr_Data[5] & ~Slve_Data[6] & Mstr_Data[6]) |
                       (~Mstr_Data[6] & ~Slve_Data[7] & Mstr_Data[7])));

//-----------------------------------------------------------------------------
//
// Alexander Bang Bang Phase Detector
//
always @ (posedge rx_clkdiv8) begin
   if (rx_reset == 1'b1) begin
      pd_count          <= 5'd16;  // Decimal 16
      pd_ovflw_down     <= 1'b0;   //
      pd_ovflw_up       <= 1'b0;   //
   end else begin
      //
      // Code initializes to 16 and then looks for increment/decrement requests
      // Once it is goes past +/-8 implements the phase change and restarts
      //
      if (rx_state != 5'h09 ) begin
         // Hold in reset when not in the Wait and Monitor state
         pd_count      <= 5'd16;
         pd_ovflw_down <= 1'b0;
         pd_ovflw_up   <= 1'b0;
      end else begin
         // Accumulate phase direction
         pd_count <= pd_count + PhaseDet_Inc - PhaseDet_Dec;
         if (pd_count >= (5'd24)) begin
            pd_ovflw_up <= 1'b1;
         end else if (pd_count <= (5'd8)) begin
            pd_ovflw_down <= 1'b1;
         end
      end
   end
end

//-----------------------------------------------------------------------------
//
// Dynamic Alignment State Machine for increments/decrements
//
always @ (posedge rx_clkdiv8) begin
   if (rx_reset) begin
      rx_state          <= 5'h0;
      Mstr_CntVal_In    <= 9'b0;
      Mstr_Load         <= 1'b0;
      Slve_CntVal_In    <= 9'b0;
      Slve_Load         <= 1'b0;
      Slve_Less         <= 1'b0;
      BitTime_Val       <= 9'b0;
      Delay_Change      <= 8'h0;
      MstrCnt_GE_BTval  <= 1'b0;
      MstrCnt_GE_HalfBT <= 1'b0;
      MstrCnt_LE_Seven  <= 1'b0;
      Mstr_IdlyRst      <= 1'b0;
      Update_Seq        <= 8'b11001100;
      rx_cntval         <= 9'b0;
      rx_dlyload        <= 1'b0;
      rx_ready          <= 1'b0;
   end else begin
      //
      // Pre-compute values for timing closure
      //
      MstrCnt_GE_BTval  <= (Mstr_CntVal_In >= BitTime_Val);
      MstrCnt_GE_HalfBT <= (Mstr_CntVal_In >= {1'b0,BitTime_Val[8:1]});
      MstrCnt_LE_Seven  <= (Mstr_CntVal_In <= 9'h007);

      //
      // Massive state machine
      //
      case (rx_state)
         //
         // 0 - Initialization
         //
         5'h00 :  begin
            Mstr_IdlyRst   <= 1'b1;
            rx_state       <= rx_state + 1'b1;      // Increment to next state
         end
         //
         // 1 - Deassert Master Idelay Reset
         //
         5'h01 :  begin
            Mstr_IdlyRst   <= 1'b0;
            rx_state       <= rx_state + 1'b1;      // Increment to next state
         end
         //
         // 7 - Store bit time value 6 cycles after reset complete
         //
         5'h07 :  begin
            if (Mstr_CntVal_Out < 64 ) begin
               rx_state    <= 5'h01;
            end
            else begin
               BitTime_Val    <= Mstr_CntVal_Out;      // Save Bit UI in taps
               rx_state       <= rx_state + 1'b1;      // 0x08 - Store Initial
            end
         end
         //
         // 8 - Store Initial Master and Slave
         //
         5'h08 :  begin
            BitTime_Val    <= {BitTime_Val[8:3],3'b0};  // Truncate to modulo 8
            Mstr_CntVal_In <= 9'b0;                     // Set Master to  0% UI
            Slve_CntVal_In <= {1'b0,BitTime_Val[8:3],2'b0}; // Set Slave  to 50% UI
            rx_state       <= 5'h12;                    // 0x12 - Load delays
         end
         //
         // 9 - Wait and Monitor
         //
         5'h09 :  begin
            if      (pd_ovflw_up   == 1'b1 ) begin
               rx_state <= 5'h10;                     // Increment Active Delay
               Update_Seq <= {1'b1, Update_Seq[7:1]}; // Log 1 for increment
            end
            else if (pd_ovflw_down == 1'b1 ) begin
               rx_state <= 5'h11;                     // Decrement Active Delay
               Update_Seq <= {1'b0, Update_Seq[7:1]}; // Log 0 for decrement
            end
         end
         //
         // 0x10 - Increment Active Delay by 8
         //
         5'h10 : begin
            if (MstrCnt_GE_BTval) begin        // Master is = 1.0UI
                Mstr_CntVal_In <= 9'b0;                    // Set master delay to  0%
                Slve_CntVal_In <= {1'b0,BitTime_Val[8:1]}; // Set slave  delay to 50%
                Slve_Less      <= 1'b0;                    // Set slave greater than master
            end
            else if (MstrCnt_GE_HalfBT) begin  // Master is >= 0.5UI
                Mstr_CntVal_In <= Mstr_CntVal_In + 8;                    // Increment master delay by 8
                Slve_CntVal_In <= Mstr_CntVal_In - BitTime_Val[8:1] + 8; // Set slave delay
                Slve_Less      <= 1'b1;                                  // Set slave less than master
            end
            else begin                         // Master is < 0.5UI
                Mstr_CntVal_In <= Mstr_CntVal_In + 8;                    // Increment master delay by 8
                Slve_CntVal_In <= Mstr_CntVal_In + BitTime_Val[8:1] + 8; // Set slave delay
                Slve_Less      <= 1'b0;                                  // Set slave greater than master
            end
            rx_state <= 5'h12;                 // 0x12 - Load delays
         end
         //
         // 0x11 - Decrement Active Delay by 8
         //
         5'h11 : begin
            if (MstrCnt_LE_Seven) begin          // Master is = 0.0UIx
                Mstr_CntVal_In <= BitTime_Val;             // Set master delay to max
                Slve_CntVal_In <= {1'b0,BitTime_Val[8:1]}; // Set slave  delay to 50%
                Slve_Less      <= 1'b1;                    // Set slave less than master
            end
            else if (MstrCnt_GE_HalfBT && Mstr_CntVal_In < BitTime_Val[8:0] + 8) begin   // Master is >= 0.5UI but IDELAY will underflow so adjust Slve_CntVal_In TSR 975184
               Mstr_CntVal_In <= Mstr_CntVal_In - 8;                    // Decrement master delay by 8
               Slve_CntVal_In <= BitTime_Val[8:0] + Mstr_CntVal_In - BitTime_Val[8:1] -8; // Set slave delay to wrap around to BitTime_Val TSR975184
               Slve_Less      <= 1'b0;                    // TSR975184 When IDELAY is underflowing set Slve_Less to 0
           end
           else if (MstrCnt_GE_HalfBT) begin   // Master is >= 0.5UI
                Mstr_CntVal_In <= Mstr_CntVal_In - 8;                    // Decrement master delay by 8
                Slve_CntVal_In <= Mstr_CntVal_In - BitTime_Val[8:1] - 8; // Set slave delay
                Slve_Less      <= 1'b1;                                  // Set slave less than master
            end
            else begin                          // Master is < 0.5UI
                Mstr_CntVal_In <= Mstr_CntVal_In - 8;                    // Decrement master delay by 8
                Slve_CntVal_In <= Mstr_CntVal_In + BitTime_Val[8:1] - 8; // Set slave delay
                Slve_Less      <= 1'b0;                                  // Set slave greater than master
            end
            rx_state <= 5'h12;                  // 0x12 - Load delays
         end
         //
         // 0x12 - Load Delays
         //
         5'h12 : begin
            Mstr_Load    <= 1'b1;            // Reset previous Master load
            Slve_Load    <= 1'b1;            // Reset previous Slave load
            rx_state     <= rx_state + 1'b1; // 0x13 - Finish delay changes
         end
         //
         // 0x13 - Finish delay changes
         //
         5'h13 : begin
            Mstr_Load    <= 1'b0;            // Reset previous Master load
            Slve_Load    <= 1'b0;            // Reset previous Slave load
            Delay_Change <= 8'hff;           // Load Delay Change
            rx_state     <= rx_state + 1'b1; // 0x14 - Wait for load change to complete
         end
         //
         // 0x14 - Wait for load change to complete
         //
         5'h14 : begin
            if (Delay_Change[0] != 1'b0) begin // Wait 8 cycles for change
                Delay_Change <= {1'b0,Delay_Change[7:1]};
            end
            else begin
               if ( Update_Seq == 8'hAA) begin	 //TSR975184 Removing VCO_MULTIPLIER dependency
                  rx_state <= 5'h15;         // 0x15 - Stop training
               end
               else begin
                  rx_state <= 5'h09;         // 0x09 - Monitor Phase Detector
               end
            end
         end
         //
         // 0x15 - Set Delay Value for broadcast to data lines
         //
         5'h15 : begin
            rx_cntval <= Mstr_CntVal_In;     // Set the delay value for all
            Slve_CntVal_In <= Mstr_CntVal_In;     // Set the delay value for all
            rx_state  <= rx_state + 1'b1;    // 0x16 - Assert Delay Load
         end
         //
         // 0x16 - Assert the Delay Load for broadcast to data lines
         //
         5'h16 : begin
            rx_dlyload <= 1'b1;              // Assert the Load output
            Slve_Load  <= 1'b1;              // Assert the Load output
            rx_state   <= rx_state + 1'b1;   // 0x17 - Deassert Load
         end
         //
         // 0x17 - Deassert Load and Assert Ready
         //
         5'h17 : begin
            rx_dlyload <= 1'b0;              // Deassert the Load output
            Slve_Load  <= 1'b0;              // Assert the Load output
            rx_state   <= 5'h1F;             // 0x1F - End state
            // Stay in the state until reset
         end
         //
         // 0x1F - End State
         //
         5'h1F : begin
            rx_ready <= 1'b1;                // Assert rx_ready and end
         end
         //
         // Default State
         //
         default : begin
            rx_state   <= rx_state + 1'b1;   // Increment to next state
         end
      endcase
   end
end

//-----------------------------------------------------------------------------
//
//  RX write address counter
//
always @ (posedge rx_clkdiv8)
begin
    if (rx_reset)
        rx_wr_count <= 5'h4;
    else
        rx_wr_count <= rx_wr_count + 1'b1;
end
assign rx_wr_addr   = rx_wr_count;

//-----------------------------------------------------------------------------
//
//  Pixel read address counter
//
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

//
// Register data from ISERDES
//
always @ (posedge rx_clkdiv8)
begin
   rx_wr_data <= Mstr_Data;
end
//-----------------------------------------------------------------------------
//
// Generate 8 Dual Port Distributed RAMS for FIFO
//
genvar i;
generate
for (i = 0 ; i < 8 ; i = i+1) begin : bit
  RAM32X1D fifo (
     .D     (rx_wr_data[i]),
     .WCLK  (rx_clkdiv8),
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
//
// Store last read pixel data for one cycle, bit 0 is not required
//
always @ (posedge px_clk)
begin
    px_rd_last[7:1] <= px_rd_curr[7:1];
end

//-----------------------------------------------------------------------------
//
// Pixel 8-to-7 gearbox
//
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
//
// Synchronize rx_ready to px_clk
//
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
//
// Pixel alignment state machine
//
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

// 
// Asynchronous reset for px_ready output
//
always @ (posedge px_clk or posedge px_reset) begin
   if (px_reset) begin
      px_ready   <= 3'h0;
   end
   else begin
      px_ready   <= px_ready_int;
   end
end

endmodule
