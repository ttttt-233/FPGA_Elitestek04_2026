`timescale 1ns/1ps

// Common RGB565 stream switch. All inputs must already be in clk domain.
// Selection changes only between frames, preventing a torn mixed-source frame.
module Video_Source_Selector #(
    parameter DATA_WIDTH = 16,
    parameter DEFAULT_SOURCE = 1'b0
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  select_request,
    input  wire                  source0_vsync,
    input  wire                  source0_de,
    input  wire [DATA_WIDTH-1:0] source0_data,
    input  wire                  source1_vsync,
    input  wire                  source1_de,
    input  wire [DATA_WIDTH-1:0] source1_data,
    output wire                  output_vsync,
    output wire                  output_de,
    output wire [DATA_WIDTH-1:0] output_data
);
    reg active_source;
    reg any_vsync_d;
    wire any_vsync = source0_vsync | source1_vsync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_source <= DEFAULT_SOURCE;
            any_vsync_d <= 1'b0;
        end else begin
            any_vsync_d <= any_vsync;
            if (any_vsync_d && !any_vsync)
                active_source <= select_request;
        end
    end

    assign output_vsync = active_source ? source1_vsync : source0_vsync;
    assign output_de    = active_source ? source1_de    : source0_de;
    assign output_data  = active_source ? source1_data  : source0_data;
endmodule
