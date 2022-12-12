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
      output            rx_clkdiv8,           // RX clock div8 output

      output            cmt_locked,           // PLL/MMCM locked output
      output     [4:0]  rx_wr_addr,           // RX write_address output
      output            rx_reset,             // RX reset output
      output reg        rx_ready,             // RX ready output

      output            px_clk,               // Pixel clock output
      output     [4:0]  px_rd_addr,           // Pixel read address output
      output reg [2:0]  px_rd_seq,            // Pixel read sequence output
      output reg        px_ready              // Pixel data ready output
   );

wire       clkin_p_i;
wire       clkin_n_i;
wire       clkin_p_d;
wire       clkin_n_d;
wire       px_mmcm;
wire       mmcm_div2;
wire       locked_and_idlyrdy;
reg  [3:0] rx_reset_sync;
reg  [3:0] px_reset_sync;
wire       px_reset;

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
         .CLKIN1_PERIOD      (CLKIN_PERIOD),
         .BANDWIDTH          ("OPTIMIZED"),
         .CLKFBOUT_MULT_F    (6*VCO_MULTIPLIER),
         .CLKFBOUT_PHASE     (0.0),
         .CLKOUT0_DIVIDE_F   (2*VCO_MULTIPLIER),

         .CLKOUT0_DUTY_CYCLE (0.5),
         .CLKOUT0_PHASE      (0.0),
         .DIVCLK_DIVIDE      (1),
         .REF_JITTER1        ()
    )
   mmcm0 (
         .CLKFBOUT       (px_mmcm),
         .CLKFBOUTB      (),
         .CLKOUT0        (mmcm_div2),
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
//-----------------------------------------------------------------------------
// Global Clock Buffers
BUFG  px     (.I(px_mmcm),      .O(px_clk)) ;

BUFG  div2   (.I(mmcm_div2), .O(rx_clkdiv2)) ;

BUFGCE_DIV  # ( .BUFGCE_DIVIDE(4))
      clkdiv8 (
       .I(mmcm_div2),
       .CLR(!cmt_locked),
       .CE(1'b1),
       .O(rx_clkdiv8)
);
//-----------------------------------------------------------------------------
// Clock Master Side ISERDES
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
// Clock Slave Side ISERDES

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
// Synchronize locked to rx_clkdiv8
assign locked_and_idlyrdy = cmt_locked & idelay_rdy;
always @ (posedge rx_clkdiv8 or negedge locked_and_idlyrdy)
begin
   if (!locked_and_idlyrdy)
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

always @ (posedge rx_clkdiv8)
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
always @ (posedge rx_clkdiv8)
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