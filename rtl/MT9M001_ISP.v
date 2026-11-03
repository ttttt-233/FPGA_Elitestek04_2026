`timescale 1ns/1ps
// Two line RAMs, synchronous read, 3x3 bilinear demosaic.
// Pattern at the FIRST captured pixel: 0=BGGR, 1=GBRG, 2=GRBG, 3=RGGB.
// Preserve IMAGE_WIDTH*IMAGE_HEIGHT words/frame. The unavailable 3x3
// neighbourhood on the first two rows/columns is padded black.
module MT9M001_ISP #(
    parameter IMAGE_WIDTH=1280, IMAGE_HEIGHT=720,
    parameter [1:0] BAYER_PATTERN=2'd2,
    parameter GAMMA_ENABLE=1,
    parameter R_GAIN_Q8=256, G_GAIN_Q8=256, B_GAIN_Q8=256,
    parameter BLACK_LEVEL=0
)(
    input clk, input rst_n,
    input in_vsync, input in_href, input [7:0] in_bayer,
    output reg out_vsync, output reg out_href, output [15:0] out_rgb565,
    output wire raw_vsync,raw_href,
    output wire [7:0] raw_bayer
);
    reg [7:0] row1_mem[0:IMAGE_WIDTH-1], row2_mem[0:IMAGE_WIDTH-1];
    reg [11:0] x,y;
    reg href_prev, frame_armed;
    reg [7:0] raw_d,bank0_d,bank1_d;
    reg [11:0] x_d,y_d;
    reg de_d,vs_d;
    // Lossless tap of exactly the input samples accepted by this ISP, before
    // demosaic/gain/gamma. Exclude DVP surplus columns/rows using the SAME gate.
    assign raw_vsync=vs_d;
    assign raw_href=de_d;
    assign raw_bayer=raw_d;
    // RAMs deliberately have no reset: row/column guards exclude stale data.
    always @(posedge clk) begin
        if (in_vsync && in_href && x<IMAGE_WIDTH && y<IMAGE_HEIGHT) begin
            bank0_d<=row1_mem[x];
            bank1_d<=row2_mem[x];
            // Alternate write banks each row. Both read ports are registered;
            // no asynchronous RAM read feeds another RAM's write port.
            if (y[0]) row2_mem[x]<=in_bayer;
            else row1_mem[x]<=in_bayer;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x<=0; y<=0; href_prev<=0; frame_armed<=0;
            raw_d<=0; x_d<=0; y_d<=0; de_d<=0; vs_d<=0;
        end else begin
            href_prev<=in_href;
            if (!in_vsync) begin x<=0; y<=0; frame_armed<=1; end
            else begin
                if (in_href) begin
                    if (x<IMAGE_WIDTH) x<=x+1'b1;
                end else x<=0;
                // One increment per LINE_VALID falling edge, not blank clocks.
                if (href_prev && !in_href && y<IMAGE_HEIGHT) y<=y+1'b1;
            end
            raw_d<=in_bayer; x_d<=x; y_d<=y;
            vs_d<=frame_armed && in_vsync;
            de_d<=frame_armed && in_vsync && in_href && x<IMAGE_WIDTH && y<IMAGE_HEIGHT;
        end
    end
    wire [7:0] row1_d=y_d[0] ? bank0_d : bank1_d;
    wire [7:0] row2_d=y_d[0] ? bank1_d : bank0_d;
    reg [7:0] t_left,t_mid,m_left,m_mid,b_left,b_mid;
    reg [7:0] r_next,g_next,b_next,r_d,g_d,b_d;
    reg [9:0] diagonal_sum,cross_sum;
    reg [8:0] horizontal_sum,vertical_sum;
    wire [1:0] phase = {y_d[0],x_d[0]} ^ 2'b11 ^ BAYER_PATTERN;
    always @(*) begin
        diagonal_sum={2'd0,t_left}+{2'd0,row2_d}+{2'd0,b_left}+{2'd0,raw_d};
        cross_sum={2'd0,t_mid}+{2'd0,m_left}+{2'd0,row1_d}+{2'd0,b_mid};
        horizontal_sum={1'b0,m_left}+{1'b0,row1_d};
        vertical_sum={1'b0,t_mid}+{1'b0,b_mid};
        r_next=0; g_next=0; b_next=0;
        if (de_d && x_d>=2 && y_d>=2) begin
            case (phase)
                2'd0: begin b_next=m_mid; g_next=cross_sum[9:2]; r_next=diagonal_sum[9:2]; end
                2'd1: begin g_next=m_mid; b_next=horizontal_sum[8:1]; r_next=vertical_sum[8:1]; end
                2'd2: begin g_next=m_mid; r_next=horizontal_sum[8:1]; b_next=vertical_sum[8:1]; end
                2'd3: begin r_next=m_mid; g_next=cross_sum[9:2]; b_next=diagonal_sum[9:2]; end
            endcase
        end
    end
    reg de_rgb,vs_rgb;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_left<=0; t_mid<=0; m_left<=0; m_mid<=0; b_left<=0; b_mid<=0;
            r_d<=0; g_d<=0; b_d<=0; de_rgb<=0; vs_rgb<=0;
            out_href<=0; out_vsync<=0;
        end else begin
            if (de_d) begin
                t_left<=t_mid; t_mid<=row2_d;
                m_left<=m_mid; m_mid<=row1_d;
                b_left<=b_mid; b_mid<=raw_d;
            end else begin
                t_left<=0; t_mid<=0; m_left<=0; m_mid<=0; b_left<=0; b_mid<=0;
            end
            r_d<=r_next; g_d<=g_next; b_d<=b_next;
            de_rgb<=de_d; vs_rgb<=vs_d;
            out_href<=de_rgb; out_vsync<=vs_rgb;
        end
    end
    Color_Tone_Map #(
        .GAMMA_ENABLE(GAMMA_ENABLE), .R_GAIN_Q8(R_GAIN_Q8),
        .G_GAIN_Q8(G_GAIN_Q8), .B_GAIN_Q8(B_GAIN_Q8), .BLACK_LEVEL(BLACK_LEVEL)
    ) color_pipeline (.clk(clk), .rst_n(rst_n), .r(r_d), .g(g_d), .b(b_d), .rgb565(out_rgb565));
endmodule
