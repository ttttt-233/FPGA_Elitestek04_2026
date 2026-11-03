`timescale 1ns/1ps

module Sobel_Edge_3x3 #(
    parameter ACTIVE_WIDTH=1280, parameter ACTIVE_HEIGHT=720
)(
    input wire clk,input wire rst_n,input wire input_vsync,input wire input_de,
    input wire [11:0] edge_threshold,
    input wire [15:0] input_original_rgb565,
    input wire [7:0] input_original_gray,input wire [7:0] input_filtered_gray,
    output reg output_vsync,output reg output_de,
    output reg [11:0] output_x,output reg [11:0] output_y,
    output reg [15:0] output_original_rgb565,
    output reg [7:0] output_original_gray,output reg [7:0] output_filtered_gray,
    output reg [7:0] output_edge,output reg [11:0] output_magnitude,
    output reg [1:0] output_direction,
    // Signed derivatives for the complete-frame Canny experiment.
    output reg signed [10:0] output_gx,output reg signed [10:0] output_gy
);
    wire wvs,wde,wvalid; wire [11:0] wx,wy;
    wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    wire [15:0] center_rgb;
    wire [7:0] center_original_gray;
    wire [10:0] gxp={3'd0,p02[7:0]}+({3'd0,p12[7:0]}<<1)+{3'd0,p22[7:0]};
    wire [10:0] gxn={3'd0,p00[7:0]}+({3'd0,p10[7:0]}<<1)+{3'd0,p20[7:0]};
    wire [10:0] gyp={3'd0,p20[7:0]}+({3'd0,p21[7:0]}<<1)+{3'd0,p22[7:0]};
    wire [10:0] gyn={3'd0,p00[7:0]}+({3'd0,p01[7:0]}<<1)+{3'd0,p02[7:0]};
    wire [10:0] gx=(gxp>=gxn)?(gxp-gxn):(gxn-gxp);
    wire [10:0] gy=(gyp>=gyn)?(gyp-gyn):(gyn-gyp);
    // Quantized gradient direction for Canny non-maximum suppression:
    // 0=horizontal, 1=down-right/up-left, 2=vertical, 3=up-right/down-left.
    wire [1:0] direction = (gx > (gy << 1)) ? 2'd0 :
                           (gy > (gx << 1)) ? 2'd2 :
                           ((gxp >= gxn) == (gyp >= gyn)) ? 2'd1 : 2'd3;
    wire [11:0] magnitude;
    Gradient_Magnitude_Approx magnitude_approx (
        .abs_x(gx),
        .abs_y(gy),
        .magnitude(magnitude)
    );
    // Convolution needs only filtered gray. Center RGB and optional original
    // gray each use one row; separate instances preserve unused-lane pruning.
    Window_3x3 #(.DATA_WIDTH(8),.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)) window (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data(input_filtered_gray),
        .output_vsync(wvs),.output_de(wde),.window_valid(wvalid),.output_x(wx),.output_y(wy),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),.p20(p20),.p21(p21),.p22(p22));
    Window_Center_Delay #(.DATA_WIDTH(16),.ACTIVE_WIDTH(ACTIVE_WIDTH)) color_delay (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data(input_original_rgb565),.center(center_rgb));
    Window_Center_Delay #(.DATA_WIDTH(8),.ACTIVE_WIDTH(ACTIVE_WIDTH)) original_gray_delay (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data(input_original_gray),.center(center_original_gray));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin output_vsync<=0;output_de<=0;output_x<=0;output_y<=0;output_original_rgb565<=0;output_original_gray<=0;output_filtered_gray<=0;output_edge<=0;output_magnitude<=0;output_direction<=0;output_gx<=0;output_gy<=0; end
        else begin
            output_vsync<=wvs;output_de<=wde;output_x<=wx;output_y<=wy;
            output_original_rgb565<=wvalid?center_rgb:0;
            output_original_gray<=wvalid?center_original_gray:0;
            output_filtered_gray<=wvalid?p11[7:0]:0;
            output_magnitude<=wvalid?magnitude:0;
            output_gx<=wvalid?(gxp-gxn):11'd0;
            output_gy<=wvalid?(gyp-gyn):11'd0;
            output_edge<=(wvalid && magnitude>=edge_threshold)?8'hff:8'h00;
            output_direction<=wvalid?direction:2'd0;
        end
    end
endmodule
