`timescale 1ns/1ps

// Resource-efficient Euclidean gradient-magnitude approximation.
//
//   M = max(abs_x, abs_y)
//   m = min(abs_x, abs_y)
//   magnitude = floor((123*M + 51*m) / 128)
//
// The constant products are factored into shifts and additions/subtractions so
// synthesis does not need a general-purpose multiplier or divider.
module Gradient_Magnitude_Approx (
    input  wire [10:0] abs_x,
    input  wire [10:0] abs_y,
    output wire [11:0] magnitude
);
    wire [10:0] maximum = (abs_x >= abs_y) ? abs_x : abs_y;
    wire [10:0] minimum = (abs_x >= abs_y) ? abs_y : abs_x;

    wire [17:0] maximum_ext = {7'd0, maximum};
    wire [17:0] minimum_ext = {7'd0, minimum};

    // 123*M = 128*M - 5*M
    wire [17:0] maximum_times_5   = (maximum_ext << 2) + maximum_ext;
    wire [17:0] maximum_times_123 = (maximum_ext << 7) - maximum_times_5;

    // 51*m = 48*m + 3*m
    wire [17:0] minimum_times_48 = (minimum_ext << 5) + (minimum_ext << 4);
    wire [17:0] minimum_times_3  = (minimum_ext << 1) + minimum_ext;
    wire [17:0] minimum_times_51 = minimum_times_48 + minimum_times_3;

    // Sobel |Gx| and |Gy| are at most 1020.  The largest numerator is
    // (123 + 51) * 1020 = 177480, which fits in 18 bits.
    wire [17:0] weighted_sum = maximum_times_123 + minimum_times_51;

    // Division by 128 is an exact right shift.  Integer hardware truncates the
    // fractional part; the maximum result is floor(177480 / 128) = 1386.
    assign magnitude = {1'b0, weighted_sum[17:7]};
endmodule
