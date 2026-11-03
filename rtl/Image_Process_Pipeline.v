`timescale 1ns/1ps

// RGB565 -> grayscale -> median/Gaussian 3x3 -> Sobel -> streaming Canny -> composition.
// RGB565 is carried through every window so color and edges remain aligned.
// MODE: 0=color, 1=filtered gray, 2=selected edge, 3=color/edge split,
//       4=edge overlay, 5=original gray/edge split, 6=gradient direction overlay.
module Image_Process_Pipeline #(
    parameter DATA_WIDTH=16, parameter ACTIVE_WIDTH=1280, parameter ACTIVE_HEIGHT=720,
    parameter [15:0] SOBEL_EDGE_COLOR=16'hf800,
    parameter [15:0] CANNY_EDGE_COLOR=16'h07ff,
    parameter RUNTIME_FILTER=0
)(
    input wire clk,input wire rst_n,input wire input_vsync,input wire input_de,
    input wire [DATA_WIDTH-1:0] input_data,input wire [2:0] mode_select,
    // The caller holds these controls until input and output VSYNC are both low.
    input wire edge_select,
    input wire [11:0] edge_threshold,
    output wire output_vsync,output wire output_de,
    output reg [DATA_WIDTH-1:0] output_data,
    input wire gaussian_select
);
    wire [7:0] red={input_data[15:11],input_data[15:13]};
    wire [7:0] green={input_data[10:5],input_data[10:9]};
    wire [7:0] blue={input_data[4:0],input_data[4:2]};
    wire [15:0] red16={8'd0,red};
    wire [15:0] green16={8'd0,green};
    wire [15:0] blue16={8'd0,blue};
    // BT.601: (77R + 150G + 29B) / 256, implemented by shifts/adds.
    wire [15:0] rw=(red16<<6)+(red16<<3)+(red16<<2)+red16;
    wire [15:0] gw=(green16<<7)+(green16<<4)+(green16<<2)+(green16<<1);
    wire [15:0] bw=(blue16<<4)+(blue16<<3)+(blue16<<2)+blue16;
    wire [15:0] gray_sum=rw+gw+bw;
    reg [15:0] rgb565;
    reg [7:0] gray; reg gray_vsync,gray_de;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin rgb565<=0;gray<=0;gray_vsync<=0;gray_de<=0; end
        else begin rgb565<=input_data;gray<=gray_sum[15:8];gray_vsync<=input_vsync;gray_de<=input_de; end
    end

    wire filtered_vsync,filtered_de; wire [11:0] filtered_x_unused,filtered_y_unused;
    wire [15:0] filtered_original_rgb565;
    wire [7:0] filtered_original_gray,filtered_gray;
    Median_Filter_3x3 #(.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT),
        .RUNTIME_FILTER(RUNTIME_FILTER)) median_filter (
        .gaussian_select(gaussian_select),
        .clk(clk),.rst_n(rst_n),.input_vsync(gray_vsync),.input_de(gray_de),
        .input_rgb565(rgb565),.input_gray(gray),
        .output_vsync(filtered_vsync),.output_de(filtered_de),.output_x(filtered_x_unused),.output_y(filtered_y_unused),
        .output_original_rgb565(filtered_original_rgb565),
        .output_original_gray(filtered_original_gray),.output_filtered_gray(filtered_gray));

    wire sobel_vsync,sobel_de; wire [11:0] sobel_x,sobel_y,sobel_magnitude;
    wire [1:0] sobel_direction;
    wire [15:0] aligned_rgb565;
    wire [7:0] aligned_original,aligned_filtered,sobel_edge;
    Sobel_Edge_3x3 #(.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)) sobel (
        .clk(clk),.rst_n(rst_n),.input_vsync(filtered_vsync),.input_de(filtered_de),
        .edge_threshold(edge_threshold),
        .input_original_rgb565(filtered_original_rgb565),
        .input_original_gray(filtered_original_gray),.input_filtered_gray(filtered_gray),
        .output_vsync(sobel_vsync),.output_de(sobel_de),.output_x(sobel_x),.output_y(sobel_y),
        .output_original_rgb565(aligned_rgb565),
        .output_original_gray(aligned_original),.output_filtered_gray(aligned_filtered),
        .output_edge(sobel_edge),.output_magnitude(sobel_magnitude),
        .output_direction(sobel_direction),.output_gx(),.output_gy());

    wire [1:0] canny_direction;
    wire canny_vsync,canny_de; wire [11:0] canny_x,canny_y;
    wire [15:0] canny_aligned_rgb565;
    wire [7:0] canny_aligned_original,canny_aligned_filtered;
    wire [7:0] canny_aligned_sobel,canny_edge;
    // K2 controls the shared high threshold. Keep Canny's original 1:3
    // low/high relationship so the default remains LOW=32, HIGH=96.
    wire [11:0] canny_low_threshold=edge_threshold/3;
    Canny_Edge_3x3 #(
        .ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)
    ) canny (
        .clk(clk),.rst_n(rst_n),.input_vsync(sobel_vsync),.input_de(sobel_de),
        .input_original_rgb565(aligned_rgb565),
        .input_original_gray(aligned_original),
        .input_filtered_gray(aligned_filtered),
        .input_sobel_edge(sobel_edge),.input_magnitude(sobel_magnitude),
        .input_direction(sobel_direction),.low_threshold(canny_low_threshold),
        .high_threshold(edge_threshold),
        .output_vsync(canny_vsync),.output_de(canny_de),
        .output_x(canny_x),.output_y(canny_y),
        .output_original_rgb565(canny_aligned_rgb565),
        .output_original_gray(canny_aligned_original),
        .output_filtered_gray(canny_aligned_filtered),
        .output_direction(canny_direction),.output_sobel_edge(canny_aligned_sobel),.output_canny_edge(canny_edge));

    function [15:0] gray565;
        input [7:0] value;
        begin gray565={value[7:3],value[7:2],value[7:3]}; end
    endfunction
    wire [7:0] selected_edge=edge_select?canny_edge:canny_aligned_sobel;
    wire [15:0] selected_edge_rgb=gray565(selected_edge);
    wire [15:0] selected_edge_color=edge_select?CANNY_EDGE_COLOR:SOBEL_EDGE_COLOR;

    wire [63:0] direction_palette=64'hffe0_001f_07e0_f800;
    wire [15:0] direction_color=direction_palette[canny_direction*16 +: 16];
    assign output_vsync=canny_vsync;
    assign output_de=canny_de;
    always @(*) begin
        case(mode_select)
            0: output_data=canny_aligned_rgb565;
            1: output_data=gray565(canny_aligned_filtered);
            2: output_data=selected_edge_rgb;
            3: output_data=(canny_x<ACTIVE_WIDTH/2)?canny_aligned_rgb565:selected_edge_rgb;
            4: output_data=(selected_edge!=0)?selected_edge_color:canny_aligned_rgb565;
            5: output_data=(canny_x<ACTIVE_WIDTH/2)?
                           gray565(canny_aligned_original):selected_edge_rgb;
            6: output_data=(selected_edge!=0)?direction_color:canny_aligned_rgb565;
            default: output_data=canny_aligned_rgb565;
        endcase
    end
endmodule
