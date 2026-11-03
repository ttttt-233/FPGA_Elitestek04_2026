`timescale 1ns/1ps

// Streaming Canny edge post-processing.
//
// The Gaussian and Sobel stages live upstream. This block performs
// direction-aware non-maximum suppression followed by double thresholding.
// Weak edges are retained when a strong edge exists in their local 3x3
// neighbourhood. This single-pass local hysteresis is deterministic and
// streaming-friendly; it does not attempt recursive full-frame propagation.
module Canny_Edge_3x3 #(
    parameter ACTIVE_WIDTH = 1280,
    parameter ACTIVE_HEIGHT = 720
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        input_vsync,
    input  wire        input_de,
    input  wire [15:0] input_original_rgb565,
    input  wire [7:0]  input_original_gray,
    input  wire [7:0]  input_filtered_gray,
    input  wire [7:0]  input_sobel_edge,
    input  wire [11:0] input_magnitude,
    input  wire [1:0]  input_direction,
    input  wire [11:0] low_threshold,
    input  wire [11:0] high_threshold,
    output reg         output_vsync,
    output reg         output_de,
    output reg  [11:0] output_x,
    output reg  [11:0] output_y,
    output reg  [15:0] output_original_rgb565,
    output reg  [7:0]  output_original_gray,
    output reg  [7:0]  output_filtered_gray,
    output reg  [7:0]  output_sobel_edge,
    output reg  [1:0]  output_direction,
    output reg  [7:0]  output_canny_edge
);
    localparam NMS_DATA_WIDTH = 54;
    localparam HYST_DATA_WIDTH = 54;

    wire nms_vsync;
    wire nms_de;
    wire nms_valid;
    wire [11:0] nms_x;
    wire [11:0] nms_y;
    wire [NMS_DATA_WIDTH-1:0] n00,n01,n02,n10,n11,n12,n20,n21,n22;

    Window_3x3 #(
        .DATA_WIDTH(NMS_DATA_WIDTH),
        .ACTIVE_WIDTH(ACTIVE_WIDTH),
        .ACTIVE_HEIGHT(ACTIVE_HEIGHT)
    ) nms_window (
        .clk(clk), .rst_n(rst_n),
        .input_vsync(input_vsync), .input_de(input_de),
        .input_data({input_original_rgb565, input_original_gray,
                     input_filtered_gray, input_sobel_edge,
                     input_magnitude, input_direction}),
        .output_vsync(nms_vsync), .output_de(nms_de),
        .window_valid(nms_valid), .output_x(nms_x), .output_y(nms_y),
        .p00(n00), .p01(n01), .p02(n02),
        .p10(n10), .p11(n11), .p12(n12),
        .p20(n20), .p21(n21), .p22(n22)
    );

    wire [11:0] center_magnitude = n11[13:2];
    wire [1:0] center_direction = n11[1:0];
    reg [11:0] compare_a;
    reg [11:0] compare_b;
    always @(*) begin
        case (center_direction)
            2'd0: begin compare_a=n10[13:2]; compare_b=n12[13:2]; end
            2'd1: begin compare_a=n00[13:2]; compare_b=n22[13:2]; end
            2'd2: begin compare_a=n01[13:2]; compare_b=n21[13:2]; end
            default: begin compare_a=n02[13:2]; compare_b=n20[13:2]; end
        endcase
    end
    wire [11:0] nms_magnitude =
        (nms_valid && center_magnitude >= compare_a && center_magnitude >= compare_b) ?
        center_magnitude : 12'd0;

    wire hyst_vsync;
    wire hyst_de;
    wire hyst_valid;
    wire [11:0] hyst_x;
    wire [11:0] hyst_y;
    wire [HYST_DATA_WIDTH-1:0] h00,h01,h02,h10,h11,h12,h20,h21,h22;

    Window_3x3 #(
        .DATA_WIDTH(HYST_DATA_WIDTH),
        .ACTIVE_WIDTH(ACTIVE_WIDTH),
        .ACTIVE_HEIGHT(ACTIVE_HEIGHT)
    ) hysteresis_window (
        .clk(clk), .rst_n(rst_n),
        .input_vsync(nms_vsync), .input_de(nms_de),
        .input_data({center_direction, n11[53:38], n11[37:30], n11[29:22], n11[21:14],
                     nms_magnitude}),
        .output_vsync(hyst_vsync), .output_de(hyst_de),
        .window_valid(hyst_valid), .output_x(hyst_x), .output_y(hyst_y),
        .p00(h00), .p01(h01), .p02(h02),
        .p10(h10), .p11(h11), .p12(h12),
        .p20(h20), .p21(h21), .p22(h22)
    );

    wire center_strong = h11[11:0] >= high_threshold;
    wire center_weak = h11[11:0] >= low_threshold;
    wire neighbour_strong =
        (h00[11:0] >= high_threshold) || (h01[11:0] >= high_threshold) ||
        (h02[11:0] >= high_threshold) || (h10[11:0] >= high_threshold) ||
        (h12[11:0] >= high_threshold) || (h20[11:0] >= high_threshold) ||
        (h21[11:0] >= high_threshold) || (h22[11:0] >= high_threshold);
    wire canny_edge = hyst_valid && center_weak &&
                      (center_strong || neighbour_strong);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_vsync <= 1'b0;
            output_de <= 1'b0;
            output_x <= 12'd0;
            output_y <= 12'd0;
            output_original_rgb565 <= 16'd0;
            output_original_gray <= 8'd0;
            output_filtered_gray <= 8'd0;
            output_sobel_edge <= 8'd0;
            output_canny_edge <= 8'd0;
            output_direction <= 0;
        end else begin
            output_vsync <= hyst_vsync;
            output_de <= hyst_de;
            output_x <= hyst_x;
            output_y <= hyst_y;
            output_original_rgb565 <= hyst_valid ? h11[51:36] : 16'd0;
            output_original_gray <= hyst_valid ? h11[35:28] : 8'd0;
            output_filtered_gray <= hyst_valid ? h11[27:20] : 8'd0;
            output_sobel_edge <= hyst_valid ? h11[19:12] : 8'd0;
            output_direction <= hyst_valid ? h11[53:52] : 2'd0;
            output_canny_edge <= canny_edge ? 8'hff : 8'h00;
        end
    end
endmodule
