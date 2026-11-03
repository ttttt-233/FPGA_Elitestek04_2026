`timescale 1ns/1ps

// Magnitude-window NMS and double threshold. This is a frontend, NOT complete
// hysteresis: output_class 0=background, 1=weak candidate, 2=strong seed.
// Deterministic plateau tie: > toward W/NW/N/NE, >= toward the opposite side.
// This removes duplicate equal responses across straight step edges. Discrete
// direction quantization still does not guarantee every contour is one pixel.
// A full valid 3x3 gradient neighborhood is required. Sidebands follow p11.
// Caller holds 0 < threshold_low < threshold_high fixed for the entire frame.
module Canny_NMS_3x3 #(
    parameter ACTIVE_WIDTH=1280, parameter ACTIVE_HEIGHT=720
)(
    input wire clk,rst_n,input_vsync,input_de,input_support_valid,
    input wire [11:0] input_magnitude,
    input wire [1:0] input_direction,
    input wire [15:0] input_rgb565,
    input wire [7:0] input_original_gray,input_filtered_gray,
    input wire [11:0] threshold_low,threshold_high,
    output reg output_vsync,output_de,
    output reg [11:0] output_x,output_y,
    output reg [11:0] output_magnitude,
    output reg [1:0] output_class,
    output reg [1:0] output_direction,
    // Raw Sobel center, aligned to the same RGB/gray and coordinates as NMS.
    output reg [11:0] output_sobel_magnitude,
    output reg [1:0] output_sobel_class,
    output reg [15:0] output_rgb565,
    output reg [7:0] output_original_gray,output_filtered_gray
);
    wire wvs,wde,wvalid;
    wire [11:0] wx,wy;
    // Only magnitude and support need all nine neighbors. Direction, RGB and
    // gray values are center-only: keep one previous row instead of two.
    wire [12:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    wire [33:0] center;
    Window_3x3 #(.DATA_WIDTH(13),.ACTIVE_WIDTH(ACTIVE_WIDTH),.ACTIVE_HEIGHT(ACTIVE_HEIGHT)) window (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data({input_support_valid,input_magnitude}),
        .output_vsync(wvs),.output_de(wde),.window_valid(wvalid),.output_x(wx),.output_y(wy),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),
        .p20(p20),.p21(p21),.p22(p22));
    Canny_NMS_Center_Delay #(.ACTIVE_WIDTH(ACTIVE_WIDTH)) center_delay (
        .clk(clk),.rst_n(rst_n),.input_vsync(input_vsync),.input_de(input_de),
        .input_data({input_rgb565,input_original_gray,input_filtered_gray,input_direction}),
        .center(center));
    wire support=wvalid && p00[12] && p01[12] && p02[12] && p10[12] &&
                 p11[12] && p12[12] && p20[12] && p21[12] && p22[12];
    reg [11:0] neighbor_a,neighbor_b;
    always @(*) begin
        case(center[1:0])
            0: begin neighbor_a=p10[11:0];neighbor_b=p12[11:0];end
            1: begin neighbor_a=p00[11:0];neighbor_b=p22[11:0];end
            2: begin neighbor_a=p01[11:0];neighbor_b=p21[11:0];end
            3: begin neighbor_a=p02[11:0];neighbor_b=p20[11:0];end
        endcase
    end
    wire keep=support && p11[11:0]!=0 && p11[11:0]>neighbor_a && p11[11:0]>=neighbor_b;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            output_vsync<=0;output_de<=0;output_x<=0;output_y<=0;
            output_magnitude<=0;output_class<=0;output_rgb565<=0;
            output_sobel_magnitude<=0;output_sobel_class<=0;output_direction<=0;
            output_original_gray<=0;output_filtered_gray<=0;
        end else begin
            output_vsync<=wvs;output_de<=wde;output_x<=wx;output_y<=wy;
            output_rgb565<=wvalid?center[33:18]:0;
            output_original_gray<=wvalid?center[17:10]:0;
            output_filtered_gray<=wvalid?center[9:2]:0;
            output_direction<=wvalid?center[1:0]:0;
            output_magnitude<=keep?p11[11:0]:0;
            output_sobel_magnitude<=(wvalid && p11[12])?p11[11:0]:0;
            output_sobel_class<=(wvalid && p11[12] && p11[11:0]>=threshold_high)?2'd2:2'd0;
            output_class<=!keep?2'd0:(p11[11:0]>=threshold_high)?2'd2:
                          (p11[11:0]>=threshold_low)?2'd1:2'd0;
        end
    end
endmodule

// Center sideband for a raster of ACTIVE_WIDTH pixels per line. The read and
// write use the same address and clock: READ_FIRST returns the previous row.
// Its read register and two horizontal registers match Window_3x3.p11 exactly
// on valid neighborhoods, including single-cycle horizontal blanking. Nothing
// is added to NMS latency. Uninitialized rows are masked by its window_valid.
module Canny_NMS_Center_Delay #(
    parameter ACTIVE_WIDTH=1280
)(
    input wire clk,rst_n,input_vsync,input_de,
    input wire [33:0] input_data,
    output reg [33:0] center
);
    // Separate the optional original-gray lane so unused outputs can be
    // pruned before RAM packing, including their upstream line storage.
    reg [25:0] color_row_memory[0:ACTIVE_WIDTH-1];
    reg [7:0] gray_row_memory[0:ACTIVE_WIDTH-1];
    reg [25:0] color_q;
    reg [7:0] gray_q;
    wire [33:0] row_q={color_q[25:10],gray_q,color_q[9:0]};
    reg [33:0] right;
    reg [11:0] x_count;
    reg de_q;
    always @(posedge clk) begin
        if(input_vsync && input_de && x_count<ACTIVE_WIDTH) begin
            color_q<=color_row_memory[x_count];
            color_row_memory[x_count]<={input_data[33:18],input_data[9:0]};
            gray_q<=gray_row_memory[x_count];
            gray_row_memory[x_count]<=input_data[17:10];
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin x_count<=0;de_q<=0;right<=0;center<=0;end
        else begin
            if(!input_vsync || !input_de) x_count<=0;
            else if(x_count<ACTIVE_WIDTH-1) x_count<=x_count+1'b1;
            de_q<=input_vsync && input_de;
            if(de_q) begin right<=row_q;center<=right;end
            else begin right<=0;center<=0;end
        end
    end
endmodule
