`timescale 1ns/1ps
// Eight live slots. Capture/Playback owns retained media through its lease mask.
// All signals use axi_clk; read selection changes only at display blanking.
module Frame_Buffer_Control #(parameter MEDIA_LEASE_ENABLE=0)(
    input wire clk,reset,write_complete,read_boundary,
    input wire [7:0] media_pin_mask,
    output reg [2:0] write_index,read_index,
    output reg read_valid
);
    reg [2:0] latest_index;
    reg latest_valid;
    wire [2:0] completed_index=write_complete?write_index:latest_index;
    wire completed_valid=write_complete || latest_valid;
    reg [2:0] next_write,candidate;
    reg found_slot;
    integer offset;
    always @(*) begin
        next_write=write_index;candidate=write_index;found_slot=0;
        for(offset=1;offset<8;offset=offset+1) begin
            candidate=write_index+offset[2:0];
            if(!found_slot && candidate!=read_index &&
               (!MEDIA_LEASE_ENABLE || !media_pin_mask[candidate])) begin
                next_write=candidate;found_slot=1;
            end
        end
    end
    always @(posedge clk) begin
        if(reset) begin
            write_index<=0;read_index<=2;latest_index<=0;latest_valid<=0;read_valid<=0;
        end else begin
            if(write_complete) begin
                latest_index<=write_index;latest_valid<=1;write_index<=next_write;
            end
            if(read_boundary && completed_valid) begin
                read_index<=completed_index;read_valid<=1;
            end
        end
    end
endmodule
