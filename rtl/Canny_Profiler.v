`timescale 1ns/1ps
// Passive frame timing. Slots 0..4: wait/capture/scan/roots/output; slot 5: total.
// Slots 6..11 count the subset waiting on this client's AXI/DDR interface.
// Publish only successful frames. Failed attempts retain the last good report.
// Three BCD digits per slot, milliseconds floored and saturated at 999.
// INTERVAL_MODE: count the complete interval between successful publications,
// including retries/idle cycles. Current publish clock belongs to the report.
module Canny_Profiler #(parameter integer CLOCK_HZ=96000000,INTERVAL_MODE=0,EXTERNAL_COUNTS=0)(
    input wire clk,rst_n,start,active,publish,
    input wire [2:0] phase,
    input wire awvalid,awready,wvalid,wready,wlast,bvalid,
    input wire arvalid,arready,rvalid,rready,rlast,
    // Optional already-coherent per-frame stage / publication-interval counts.
    // Twelve unsigned 32-bit slots, same order as milliseconds_bcd.
    input wire [383:0] external_counts,
    output reg [143:0] milliseconds_bcd,
    output reg report_valid
);
    localparam [31:0] CYCLES_PER_MS=CLOCK_HZ/1000;
    reg [31:0] counts[0:11];
    reg [31:0] snapshot[0:11];
    reg read_pending,write_response_pending;
    // The two channels may overlap: count a stalled clock only once. Waiting
    // for capture FIFO data or downstream pixel consumption is not DDR wait.
    wire ddr_wait=(arvalid && !arready) ||
        (read_pending && rready && !rvalid) ||
        (awvalid && !awready) || (wvalid && !wready) ||
        (write_response_pending && !bvalid);
    reg converting;
    reg [3:0] index;
    reg [5:0] bit_index;
    reg [31:0] quotient;
    reg [32:0] remainder;
    reg [143:0] converted;
    wire [32:0] shifted={remainder[31:0],quotient[31]};
    wire subtract=shifted>={1'b0,CYCLES_PER_MS};
    wire [31:0] next_quotient={quotient[30:0],subtract};
    wire [9:0] limited_ms=next_quotient>999?10'd999:next_quotient[9:0];
    function [11:0] to_bcd;
        input [9:0] value;
        reg [21:0] shift;
        integer j;
        begin
            shift={12'd0,value};
            for(j=0;j<10;j=j+1) begin
                if(shift[13:10]>=5) shift[13:10]=shift[13:10]+4'd3;
                if(shift[17:14]>=5) shift[17:14]=shift[17:14]+4'd3;
                if(shift[21:18]>=5) shift[21:18]=shift[21:18]+4'd3;
                shift=shift<<1;
            end
            to_bcd=shift[21:10];
        end
    endfunction
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(i=0;i<12;i=i+1) begin counts[i]<=0;snapshot[i]<=0;end
            read_pending<=0;write_response_pending<=0;
            converting<=0;index<=0;bit_index<=0;quotient<=0;remainder<=0;
            converted<=0;milliseconds_bcd<=0;report_valid<=0;
        end else begin
            if(arvalid && arready) read_pending<=1;
            if(rvalid && rready && rlast) read_pending<=0;
            if(wvalid && wready && wlast) write_response_pending<=1;
            if(bvalid) write_response_pending<=0;
            if(start) begin
                for(i=0;i<12;i=i+1) counts[i]<=0;
                read_pending<=0;write_response_pending<=0;
            end else if(active && phase<5) begin
                for(i=0;i<6;i=i+1) if(phase==i || i==5) begin
                    if(counts[i]!=32'hffffffff) counts[i]<=counts[i]+1'b1;
                    if(ddr_wait && counts[i+6]!=32'hffffffff) counts[i+6]<=counts[i+6]+1'b1;
                end
            end
            if(INTERVAL_MODE && publish)
                for(i=0;i<12;i=i+1) counts[i]<=0;
            // Conversion uses a separate snapshot and cannot stall the engine.
            // An unusually fast publisher while busy is ignored, never mixed
            // into an in-flight report. Board frames take far more than 384 clocks.
            if(publish && !converting) begin
                for(i=0;i<12;i=i+1) begin
                    if(EXTERNAL_COUNTS) snapshot[i]<=external_counts[i*32 +: 32];
                    else if(INTERVAL_MODE && active && phase<5 && counts[i]!=32'hffffffff &&
                       ((i<6 && (phase==i || i==5)) ||
                        (i>=6 && ddr_wait && (phase==i-6 || i==11))))
                        snapshot[i]<=counts[i]+1'b1;
                    else snapshot[i]<=counts[i];
                end
                quotient<=EXTERNAL_COUNTS?external_counts[31:0]:
                    counts[0]+{31'd0,(INTERVAL_MODE && active && phase==0 && counts[0]!=32'hffffffff)};
                remainder<=0;bit_index<=0;index<=0;
                converting<=1;
            end else if(converting) begin
                quotient<=next_quotient;
                remainder<=subtract?shifted-{1'b0,CYCLES_PER_MS}:shifted;
                if(bit_index==31) begin
                    converted[index*12 +: 12]<=to_bcd(limited_ms);
                    if(index==11) begin
                        milliseconds_bcd<={to_bcd(limited_ms),converted[131:0]};
                        report_valid<=1;converting<=0;
                    end else begin
                        index<=index+1'b1;quotient<=snapshot[index+1'b1];
                        remainder<=0;bit_index<=0;
                    end
                end else bit_index<=bit_index+1'b1;
            end
        end
    end
    // synthesis translate_off
    initial if(CLOCK_HZ<1000 || CLOCK_HZ%1000!=0)
        $fatal(1,"Profiler clock must be a whole number of kHz");
    // synthesis translate_on
endmodule
