`timescale 1ns/1ps
// One outstanding write. Round-robin at burst boundaries; selected owner is
// held from BEFORE AWVALID through B, including address/data backpressure.
// The physical interface always accepts B (there is no BREADY on this board).
module AXI_Write_Arbiter_2 (
    input wire clk,rst_n,
    input wire [31:0] p0_addr,p1_addr,
    input wire [7:0] p0_len,p1_len,
    input wire p0_valid,p1_valid,p0_wvalid,p1_wvalid,p0_wlast,p1_wlast,
    input wire [127:0] p0_data,p1_data,
    output wire p0_ready,p1_ready,p0_wready,p1_wready,p0_bvalid,p1_bvalid,
    output wire [31:0] awaddr,
    output wire [7:0] awlen,
    output wire awvalid,wvalid,wlast,
    output wire [127:0] wdata,
    input wire awready,wready,bvalid
);
    localparam IDLE=0,ADDRESS=1,DATA=2,RESPONSE=3;
    reg [1:0] state;
    reg owner,last_owner;
    assign awaddr=owner?p1_addr:p0_addr;
    assign awlen=owner?p1_len:p0_len;
    assign awvalid=state==ADDRESS && (owner?p1_valid:p0_valid);
    assign p0_ready=state==ADDRESS && !owner && awready;
    assign p1_ready=state==ADDRESS && owner && awready;
    assign wdata=owner?p1_data:p0_data;
    assign wlast=owner?p1_wlast:p0_wlast;
    assign wvalid=state==DATA && (owner?p1_wvalid:p0_wvalid);
    assign p0_wready=state==DATA && !owner && wready;
    assign p1_wready=state==DATA && owner && wready;
    wire response_phase=state==RESPONSE || (state==DATA && wvalid && wready && wlast);
    assign p0_bvalid=response_phase && !owner && bvalid;
    assign p1_bvalid=response_phase && owner && bvalid;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin state<=IDLE;owner<=0;last_owner<=1;end
        else case(state)
            IDLE: if(p0_valid || p1_valid) begin
                owner<=p0_valid && p1_valid?!last_owner:!p0_valid;
                state<=ADDRESS;
            end
            ADDRESS: if(awvalid && awready) state<=DATA;
            DATA: if(wvalid && wready && wlast) begin
                if(bvalid) begin state<=IDLE;last_owner<=owner;end
                else state<=RESPONSE;
            end
            RESPONSE: if(bvalid) begin state<=IDLE;last_owner<=owner;end
            default: state<=IDLE;
        endcase
    end
endmodule
