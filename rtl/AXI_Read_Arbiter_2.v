`timescale 1ns/1ps
// One outstanding burst, ownership retained through accepted RLAST.
// Display (port 0) has priority at burst boundaries. Address selection is
// registered BEFORE asserting ARVALID, so a late display request cannot change
// an algorithm command already stalled by the DDR command bridge.
module AXI_Read_Arbiter_2 #(parameter integer ROUND_ROBIN=0)(
    input wire clk,rst_n,
    input wire [31:0] p0_addr,p1_addr,
    input wire [7:0] p0_len,p1_len,
    input wire p0_valid,p1_valid,
    output wire p0_ready,p1_ready,
    output wire p0_rvalid,p1_rvalid,
    input wire p0_rready,p1_rready,
    output wire [31:0] araddr,
    output wire [7:0] arlen,
    output wire arvalid,
    input wire arready,rvalid,rlast,
    output wire rready
);
    reg [1:0] state;
    reg owner,last_owner;
    assign araddr=owner?p1_addr:p0_addr;
    assign arlen=owner?p1_len:p0_len;
    assign arvalid=state==1 && (owner?p1_valid:p0_valid);
    assign p0_ready=state==1 && !owner && arready;
    assign p1_ready=state==1 && owner && arready;
    // The physical controller cannot respond before accepting its command.
    // Include the AR handshake cycle to support zero-latency behavioral slaves.
    wire response_phase=state==2 || (state==1 && arvalid && arready);
    assign p0_rvalid=response_phase && !owner && rvalid;
    assign p1_rvalid=response_phase && owner && rvalid;
    assign rready=response_phase && (owner?p1_rready:p0_rready);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin state<=0;owner<=0;last_owner<=1;end
        else case(state)
            0: if(p0_valid || p1_valid) begin
                owner<=ROUND_ROBIN && p0_valid && p1_valid?!last_owner:!p0_valid;
                state<=1;
            end
            1: if(arvalid && arready) begin
                if(rvalid && rready && rlast) begin state<=0;last_owner<=owner;end
                else state<=2;
            end
            2: if(rvalid && rready && rlast) begin state<=0;last_owner<=owner;end
            default: state<=0;
        endcase
    end
endmodule
