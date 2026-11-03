`timescale 1ns/1ps

// 3x3 median denoising. Keep the Window_3x3 coordinate/border convention:
// output DE covers the full frame, with black pixels until x>=2 and y>=2.
// Center color/gray, coordinates and sync travel with the five-stage sorter.
// Optional Gaussian path shares the same window/RAM and latency. The caller
// holds gaussian_select until input AND output VSYNC are low.
module Median_Filter_3x3 #(
    parameter ACTIVE_WIDTH=1280, parameter ACTIVE_HEIGHT=720,
    parameter RUNTIME_FILTER=0
)(
    input wire clk,input wire rst_n,input wire input_vsync,input wire input_de,
    input wire [15:0] input_rgb565,input wire [7:0] input_gray,
    output wire output_vsync,output wire output_de,
    output wire [11:0] output_x,output wire [11:0] output_y,
    output wire [15:0] output_original_rgb565,
    output wire [7:0] output_original_gray,output wire [7:0] output_filtered_gray,
    input wire gaussian_select
);
    // Must match the five registered stages in Median_9.
    localparam SORT_LATENCY=5;
    wire wvs,wde,wvalid;
    wire [11:0] wx,wy;
    wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    wire [15:0] center_rgb;
    wire [7:0] median_value;
    wire [7:0] gaussian_value;
    reg [SORT_LATENCY-1:0] vsync_pipe,de_pipe,valid_pipe;
    reg [11:0] x_pipe [0:SORT_LATENCY-1], y_pipe [0:SORT_LATENCY-1];
    reg [15:0] rgb_pipe [0:SORT_LATENCY-1];
    reg [7:0] gray_pipe [0:SORT_LATENCY-1];
    integer stage;

    // Gray needs nine neighbors; RGB only needs the previous-row center.
    Window_3x3 #(.DATA_WIDTH(8),.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)) window (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data(input_gray),
        .output_vsync(wvs),.output_de(wde),.window_valid(wvalid),.output_x(wx),.output_y(wy),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),.p20(p20),.p21(p21),.p22(p22));
    Window_Center_Delay #(.DATA_WIDTH(16),.ACTIVE_WIDTH(ACTIVE_WIDTH)) color_delay (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data(input_rgb565),.center(center_rgb));
    Median_9 sorter (
        .clk(clk),.rst_n(rst_n),
        .values({p22[7:0],p21[7:0],p20[7:0],p12[7:0],p11[7:0],p10[7:0],p02[7:0],p01[7:0],p00[7:0]}),
        .median(median_value));

    generate if(RUNTIME_FILTER) begin: g_gaussian
        Gaussian_9 convolution (
            .clk(clk),.rst_n(rst_n),
            .values({p22,p21,p20,p12,p11,p10,p02,p01,p00}),
            .gaussian(gaussian_value));
    end else begin: g_median_only
        assign gaussian_value=8'd0;
    end endgenerate

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vsync_pipe<=0; de_pipe<=0; valid_pipe<=0;
            for(stage=0;stage<SORT_LATENCY;stage=stage+1) begin
                x_pipe[stage]<=0; y_pipe[stage]<=0; rgb_pipe[stage]<=0; gray_pipe[stage]<=0;
            end
        end else begin
            vsync_pipe<={vsync_pipe[SORT_LATENCY-2:0],wvs};
            de_pipe<={de_pipe[SORT_LATENCY-2:0],wde};
            valid_pipe<={valid_pipe[SORT_LATENCY-2:0],wvalid};
            x_pipe[0]<=wx; y_pipe[0]<=wy;
            rgb_pipe[0]<=wvalid?center_rgb:0;
            gray_pipe[0]<=wvalid?p11[7:0]:0;
            for(stage=1;stage<SORT_LATENCY;stage=stage+1) begin
                x_pipe[stage]<=x_pipe[stage-1]; y_pipe[stage]<=y_pipe[stage-1];
                rgb_pipe[stage]<=rgb_pipe[stage-1]; gray_pipe[stage]<=gray_pipe[stage-1];
            end
        end
    end
    assign output_vsync=vsync_pipe[SORT_LATENCY-1];
    assign output_de=de_pipe[SORT_LATENCY-1];
    assign output_x=x_pipe[SORT_LATENCY-1];
    assign output_y=y_pipe[SORT_LATENCY-1];
    assign output_original_rgb565=rgb_pipe[SORT_LATENCY-1];
    assign output_original_gray=gray_pipe[SORT_LATENCY-1];
    assign output_filtered_gray=valid_pipe[SORT_LATENCY-1]?
        ((RUNTIME_FILTER && gaussian_select)?gaussian_value:median_value):8'd0;
endmodule
