`timescale 1ns/1ps

// Binomial Gaussian kernel [1 2 1; 2 4 2; 1 2 1] / 16.
// Five registered stages match Median_9, so both share a single window and
// sideband pipeline. Extend BEFORE adding; max weighted sum is 4080.
// Divide once, truncating the final four bits (no intermediate rounding).
module Gaussian_9 (
    input wire clk, rst_n,
    input wire [71:0] values,
    output reg [7:0] gaussian
);
    reg [9:0] row0, row1, row2;
    reg [10:0] outer_rows, middle_row;
    reg [11:0] total;
    reg [7:0] normalized;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0<=0; row1<=0; row2<=0; outer_rows<=0; middle_row<=0;
            total<=0; normalized<=0; gaussian<=0;
        end else begin
            row0 <= {2'b0,values[7:0]} + {1'b0,values[15:8],1'b0} + {2'b0,values[23:16]};
            row1 <= {2'b0,values[31:24]} + {1'b0,values[39:32],1'b0} + {2'b0,values[47:40]};
            row2 <= {2'b0,values[55:48]} + {1'b0,values[63:56],1'b0} + {2'b0,values[71:64]};
            outer_rows <= {1'b0,row0} + {1'b0,row2};
            middle_row <= {row1,1'b0};
            total <= {1'b0,outer_rows} + {1'b0,middle_row};
            normalized <= total[11:4];
            gaussian <= normalized;
        end
    end
endmodule
