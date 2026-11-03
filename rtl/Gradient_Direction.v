`timescale 1ns/1ps

// Four unoriented gradient axes, x right / y down:
// 0: W/E, 1: NW/SE, 2: N/S, 3: NE/SW.
// tan(22.5 degrees) ~= 27146 / 65536. DSP constant products replace the
// shift/add trees without changing comparison boundaries or adding latency.
// Tested against atan2 for every valid signed Sobel pair (-1020..1020).
module Gradient_Direction (
    input wire signed [10:0] gradient_x,
    input wire signed [10:0] gradient_y,
    output reg [1:0] direction
);
    wire [10:0] abs_x=gradient_x[10] ? (~gradient_x+11'd1) : gradient_x;
    wire [10:0] abs_y=gradient_y[10] ? (~gradient_y+11'd1) : gradient_y;
    wire [26:0] x={16'd0,abs_x}, y={16'd0,abs_y};
    (* syn_use_dsp = "yes" *) wire [26:0] x_tan,y_tan;
    assign x_tan=abs_x*15'd27146;
    assign y_tan=abs_y*15'd27146;
    always @(*) begin
        if(abs_x==0 && abs_y==0) direction=2'd0;
        else if((y<<16)<x_tan) direction=2'd0;
        else if((x<<16)<y_tan) direction=2'd2;
        else direction=(gradient_x[10]^gradient_y[10]) ? 2'd3 : 2'd1;
    end
endmodule
