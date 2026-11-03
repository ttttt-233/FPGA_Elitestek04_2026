`timescale 1ns/1ps

// Median of nine unsigned 8-bit values, packed with sample 0 in bits [7:0].
// Five registered stages; accepts one complete window every clock.
// Sort each row, then median(max(row minima), median(row middles),
// min(row maxima)). This selection network needs 19 comparisons.
module Median_9 (
    input wire clk,
    input wire rst_n,
    input wire [71:0] values,
    output reg [7:0] median
);
    wire [23:0] row_minimum, row_middle, row_maximum;
    reg [7:0] largest_minimum, middle_middle, smallest_maximum;

    function [7:0] min3;
        input [7:0] a,b,c;
        reg [7:0] ab;
        begin ab=(a<b)?a:b; min3=(ab<c)?ab:c; end
    endfunction
    function [7:0] max3;
        input [7:0] a,b,c;
        reg [7:0] ab;
        begin ab=(a>b)?a:b; max3=(ab>c)?ab:c; end
    endfunction
    function [7:0] mid3;
        input [7:0] a,b,c;
        reg [7:0] low_ab, high_ab, capped;
        begin
            low_ab=(a<b)?a:b;
            high_ab=(a<b)?b:a;
            capped=(high_ab<c)?high_ab:c;
            mid3=(low_ab>capped)?low_ab:capped;
        end
    endfunction

    genvar row;
    generate for(row=0; row<3; row=row+1) begin: sort_rows
        wire [7:0] a=values[(row*3)*8+:8];
        wire [7:0] b=values[(row*3+1)*8+:8];
        wire [7:0] c=values[(row*3+2)*8+:8];
        reg [7:0] a1,b1,c1,a2,b2,c2,minimum,middle,maximum;
        assign row_minimum[row*8+:8]=minimum;
        assign row_middle[row*8+:8]=middle;
        assign row_maximum[row*8+:8]=maximum;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                a1<=0; b1<=0; c1<=0;
                a2<=0; b2<=0; c2<=0;
                minimum<=0; middle<=0; maximum<=0;
            end else begin
                // Stage 1: compare positions 0 and 1 in each row.
                a1<=(a<b)?a:b;
                b1<=(a<b)?b:a;
                c1<=c;
                // Stage 2: compare positions 1 and 2.
                a2<=a1;
                b2<=(b1<c1)?b1:c1;
                c2<=(b1<c1)?c1:b1;
                // Stage 3: compare positions 0 and 1 again; rows are sorted.
                minimum<=(a2<b2)?a2:b2;
                middle<=(a2<b2)?b2:a2;
                maximum<=c2;
            end
        end
    end endgenerate

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            largest_minimum<=0; middle_middle<=0; smallest_maximum<=0; median<=0;
        end else begin
            // Stage 4: retain the three candidates that bracket the median.
            largest_minimum<=max3(row_minimum[7:0],row_minimum[15:8],row_minimum[23:16]);
            middle_middle<=mid3(row_middle[7:0],row_middle[15:8],row_middle[23:16]);
            smallest_maximum<=min3(row_maximum[7:0],row_maximum[15:8],row_maximum[23:16]);
            // Stage 5: select the fifth of the original nine values.
            median<=mid3(largest_minimum,middle_middle,smallest_maximum);
        end
    end
endmodule
