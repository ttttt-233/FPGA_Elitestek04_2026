`timescale 1ns/1ps

module Video_Timing_Generator #(
    parameter ACTIVE_WIDTH  = 1280,
    parameter ACTIVE_HEIGHT = 720,
    parameter H_FRONT = 110,
    parameter H_SYNC  = 40,
    parameter H_BACK  = 220,
    parameter V_FRONT = 5,
    parameter V_SYNC  = 5,
    parameter V_BACK  = 20
)(
    input  wire        pixel_clk,
    input  wire        rst_n,
    input  wire [15:0] rgb565,
    output reg         pixel_request,
    output reg         hsync,
    output reg         vsync,
    output reg         data_enable,
    output reg         frame_blanking,
    output reg  [23:0] rgb888,
    output reg  [11:0] active_x,
    output reg  [11:0] active_y
);
    localparam H_TOTAL = H_SYNC + H_BACK + ACTIVE_WIDTH + H_FRONT;
    localparam V_TOTAL = V_SYNC + V_BACK + ACTIVE_HEIGHT + V_FRONT;
    localparam [13:0] H_ACTIVE_START = H_SYNC + H_BACK;
    localparam [12:0] V_ACTIVE_START = V_SYNC + V_BACK;
    reg [13:0] h_count;
    reg [12:0] v_count;
    wire [23:0] expanded_rgb = {
        rgb565[15:11], rgb565[15:13],
        rgb565[10:5],  rgb565[10:9],
        rgb565[4:0],   rgb565[4:2]
    };
    wire active_area = (h_count >= H_SYNC+H_BACK) &&
                       (h_count <  H_SYNC+H_BACK+ACTIVE_WIDTH) &&
                       (v_count >= V_SYNC+V_BACK) &&
                       (v_count <  V_SYNC+V_BACK+ACTIVE_HEIGHT);
    wire request_area = (h_count >= H_SYNC+H_BACK-1) &&
                        (h_count <  H_SYNC+H_BACK+ACTIVE_WIDTH-1) &&
                        (v_count >= V_SYNC+V_BACK) &&
                        (v_count <  V_SYNC+V_BACK+ACTIVE_HEIGHT);
    wire [13:0] active_x_offset = h_count - H_ACTIVE_START;
    wire [12:0] active_y_offset = v_count - V_ACTIVE_START;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_count <= 0;
            v_count <= 0;
            hsync <= 1'b0;
            vsync <= 1'b0;
            data_enable <= 1'b0;
            frame_blanking <= 1'b1;
            pixel_request <= 1'b0;
            rgb888 <= 24'd0;
            active_x <= 12'd0;
            active_y <= 12'd0;
        end else begin
            if (h_count == H_TOTAL-1) begin
                h_count <= 0;
                if (v_count == V_TOTAL-1) v_count <= 0;
                else v_count <= v_count + 1'b1;
            end else begin
                h_count <= h_count + 1'b1;
            end

            hsync <= (h_count >= H_SYNC);
            vsync <= (v_count >= V_SYNC);
            data_enable <= active_area;
            frame_blanking <= v_count < V_SYNC+V_BACK || v_count >= V_SYNC+V_BACK+ACTIVE_HEIGHT;
            pixel_request <= request_area;
            rgb888 <= active_area ? expanded_rgb : 24'd0;
            active_x <= active_area ? active_x_offset[11:0] : 12'd0;
            active_y <= active_area ? active_y_offset[11:0] : 12'd0;
        end
    end
endmodule
