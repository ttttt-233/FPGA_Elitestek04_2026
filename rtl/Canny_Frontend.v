`timescale 1ns/1ps

// RGB565 -> selectable median/Gaussian -> signed Sobel -> direction -> NMS
// -> weak/strong candidates. Full hysteresis is a separate frame-memory engine.
// Both filters share a window and five arithmetic stages. Filter selection,
// like the edge algorithm, must stay fixed until the entire pipeline is idle.
// Thresholds must remain fixed for the whole frame. In automatic mode the
// previous frame's NMS magnitude distribution selects the next frame's value.
module Canny_Frontend #(
    parameter ACTIVE_WIDTH=1280, parameter ACTIVE_HEIGHT=720,
    parameter RUNTIME_ALGORITHM=0,
    parameter AUTO_RATE_GUARD_ENABLE=0, parameter RUNTIME_FILTER=0
)(
    input wire clk,rst_n,input_vsync,input_de,
    input wire [15:0] input_rgb565,
    input wire [11:0] threshold_low,threshold_high,
    input wire adaptive_enable,
    input wire adaptive_decrease_allowed,
    input wire [11:0] adaptive_rate_floor_high,
    input wire edge_select,
    output wire output_vsync,output_de,
    output wire [11:0] output_x,output_y,output_magnitude,
    output wire [1:0] output_class,output_direction,
    output wire [15:0] output_rgb565,
    output wire [7:0] output_original_gray,output_filtered_gray,
    output wire [11:0] output_threshold_low,output_threshold_high,
    output wire [19:0] output_scene_samples,
    output wire [11:0] output_scene_high,
    input wire gaussian_select
);
    wire [7:0] red={input_rgb565[15:11],input_rgb565[15:13]};
    wire [7:0] green={input_rgb565[10:5],input_rgb565[10:9]};
    wire [7:0] blue={input_rgb565[4:0],input_rgb565[4:2]};
    wire [15:0] r={8'd0,red},g={8'd0,green},b={8'd0,blue};
    wire [15:0] gray_sum=(r<<6)+(r<<3)+(r<<2)+r+(g<<7)+(g<<4)+(g<<2)+(g<<1)
                        +(b<<4)+(b<<3)+(b<<2)+b;
    reg vs,de;
    reg [15:0] rgb;
    reg [7:0] gray;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin vs<=0;de<=0;rgb<=0;gray<=0;end
        else begin vs<=input_vsync;de<=input_de;rgb<=input_rgb565;gray<=gray_sum[15:8];end
    end
    wire mvs,mde,svs,sde;
    wire [15:0] mrgb,srgb;
    wire [7:0] mogray,mgray,sogray,sgray;
    wire [11:0] sx,sy,mag;
    wire signed [10:0] gx,gy;
    wire [1:0] dir;
    wire [11:0] applied_low,applied_high;
    wire [11:0] nms_magnitude,sobel_magnitude;
    wire [1:0] nms_class,sobel_class;
    assign output_magnitude=(RUNTIME_ALGORITHM && !edge_select)?sobel_magnitude:nms_magnitude;
    assign output_class=(RUNTIME_ALGORITHM && !edge_select)?sobel_class:nms_class;
    Median_Filter_3x3 #(.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT),
        .RUNTIME_FILTER(RUNTIME_FILTER)) median_filter (
        .gaussian_select(gaussian_select),
        .clk(clk),.rst_n(rst_n),.input_vsync(vs),.input_de(de),
        .input_rgb565(rgb),.input_gray(gray),.output_vsync(mvs),.output_de(mde),
        .output_x(),.output_y(),.output_original_rgb565(mrgb),
        .output_original_gray(mogray),.output_filtered_gray(mgray));
    Sobel_Edge_3x3 #(.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)) sobel (
        .clk(clk),.rst_n(rst_n),.input_vsync(mvs),.input_de(mde),.edge_threshold(threshold_high),
        .input_original_rgb565(mrgb),.input_original_gray(mogray),.input_filtered_gray(mgray),
        .output_vsync(svs),.output_de(sde),.output_x(sx),.output_y(sy),
        .output_original_rgb565(srgb),.output_original_gray(sogray),.output_filtered_gray(sgray),
        .output_edge(),.output_magnitude(mag),.output_gx(gx),.output_gy(gy));
    Gradient_Direction gradient_direction(.gradient_x(gx),.gradient_y(gy),.direction(dir));
    Canny_NMS_3x3 #(.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)) nms (
        .clk(clk),.rst_n(rst_n),.input_vsync(svs),.input_de(sde),
        .input_support_valid(sx>=4 && sy>=4),.input_magnitude(mag),.input_direction(dir),
        .input_rgb565(srgb),.input_original_gray(sogray),.input_filtered_gray(sgray),
        .threshold_low(applied_low),.threshold_high(applied_high),
        .output_vsync(output_vsync),.output_de(output_de),.output_x(output_x),.output_y(output_y),
        .output_magnitude(nms_magnitude),.output_class(nms_class),.output_rgb565(output_rgb565),
        .output_direction(output_direction),.output_sobel_magnitude(sobel_magnitude),.output_sobel_class(sobel_class),
        .output_original_gray(output_original_gray),.output_filtered_gray(output_filtered_gray));
    Canny_Adaptive_Threshold #(.RATE_GUARD_ENABLE(AUTO_RATE_GUARD_ENABLE)) adaptive_threshold (
        .clk(clk),.rst_n(rst_n),.sample_vsync(output_vsync),.sample_de(output_de),
        .sample_magnitude(output_magnitude),.adaptive_enable(adaptive_enable),
        .frame_vsync(svs),
        .decrease_allowed(adaptive_decrease_allowed),
        .rate_floor_high(adaptive_rate_floor_high),
        .manual_high(threshold_high),.threshold_low(applied_low),
        .threshold_high(applied_high),.adaptive_high(),
        .scene_samples(output_scene_samples),.scene_high(output_scene_high));
    assign output_threshold_low=applied_low;
    assign output_threshold_high=applied_high;
endmodule
