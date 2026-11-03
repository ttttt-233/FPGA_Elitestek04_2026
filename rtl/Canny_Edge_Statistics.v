`timescale 1ns/1ps
// Eight per-frame counts displayed as six decimal digits (999999 cap).
// Serial conversion is off the processing path. Publish takes an atomic
// snapshot; a publish during conversion restarts with the newest frame.
module Canny_Edge_Statistics (
    input wire clk,rst_n,publish,
    input wire [255:0] counts,
    output reg [191:0] data_bcd,
    output reg valid,updated
);
    reg [255:0] snapshot;
    reg [191:0] converted;
    reg [19:0] binary_shift;
    reg [23:0] bcd_shift;
    reg [4:0] bit_index;
    reg [2:0] index;
    reg converting;
    reg [23:0] adjusted;
    integer digit;
    wire [23:0] next_bcd={adjusted[22:0],binary_shift[19]};
    wire [2:0] next_index=index+3'd1;
    wire [31:0] next_value=snapshot[next_index*32 +: 32];
    function [19:0] limited;
        input [31:0] value;
        begin limited=value>999999?20'd999999:value[19:0];end
    endfunction
    always @(*) begin
        adjusted=bcd_shift;
        for(digit=0;digit<6;digit=digit+1)
            if(bcd_shift[digit*4 +: 4]>=5)
                adjusted[digit*4 +: 4]=bcd_shift[digit*4 +: 4]+4'd3;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            snapshot<=0;converted<=0;binary_shift<=0;bcd_shift<=0;
            bit_index<=0;index<=0;converting<=0;data_bcd<=0;valid<=0;updated<=0;
        end else begin
            updated<=0;
            if(publish) begin
                snapshot<=counts;binary_shift<=limited(counts[31:0]);
                bcd_shift<=0;bit_index<=0;index<=0;converting<=1;
            end else if(converting) begin
                binary_shift<={binary_shift[18:0],1'b0};bcd_shift<=next_bcd;
                if(bit_index==19) begin
                    converted[index*24 +: 24]<=next_bcd;
                    if(index==7) begin
                        data_bcd<={next_bcd,converted[167:0]};
                        valid<=1;updated<=1;converting<=0;
                    end else begin
                        index<=index+1'b1;bit_index<=0;
                        binary_shift<=limited(next_value);bcd_shift<=0;
                    end
                end else bit_index<=bit_index+1'b1;
            end
        end
    end
endmodule
