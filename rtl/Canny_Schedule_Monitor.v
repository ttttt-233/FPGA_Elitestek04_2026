`timescale 1ns/1ps
// Passive schedule diagnostics. A report covers WINDOW publication intervals,
// excludes startup, and remains fixed until the next whole window is ready.
// Slots by row (A,B): CAM period/high; CAP request-to-start/start-to-done;
// GAP min/max; IDLE before/after camera start IN THE MAXIMUM GAP;
// ERR cumulative capture/processing errors; SLOW gaps > SLOW_MS/window size.
// CAM/CAP describe the latest completed measurements at that maximum gap.
// Slots 0..7 are cycles converted to ms, 8..11 are counts, all saturate at 999.
module Canny_Schedule_Monitor #(
    parameter integer CLOCK_HZ=96000000,WINDOW=16,SLOW_MS=100
)(
    input wire clk,rst_n,vs,
    input wire cap_start,cap_started,cap_done,cap_error,processing_error,publish,
    input wire [2:0] phase,
    output reg [143:0] data_bcd,
    output reg valid
);
    localparam [31:0] CYCLES_PER_MS=CLOCK_HZ/1000;
    localparam [31:0] SLOW_CYCLES=(CLOCK_HZ/1000)*SLOW_MS;
    (* async_reg = "true" *) reg [2:0] vs_sync;
    reg vs_d,cam_seen,cam_active;
    reg [31:0] cam_timer,high_timer,cam_period,cam_high;
    reg cap_pending,cap_running;
    reg [31:0] cap_timer,cap_arm,cap_wait,cap_duration;
    reg seen;
    reg [31:0] gap,wait_before,wait_after;
    reg [31:0] gap_min,gap_max,peak_before,peak_after;
    reg [31:0] peak_period,peak_high,peak_arm,peak_duration;
    reg [9:0] samples,slow_count,capture_errors,process_errors;
    reg [31:0] snapshot[0:11];
    reg converting;
    reg [3:0] index;
    reg [5:0] bit_index;
    reg [31:0] quotient;
    reg [32:0] remainder;
    reg [143:0] converted;
    function [31:0] inc;
        input [31:0] v;
        begin inc=v==32'hffffffff ? v : v+1'b1;end
    endfunction
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
    wire [31:0] this_gap=inc(gap);
    wire [31:0] this_before=phase==0?inc(wait_before):wait_before;
    wire [31:0] this_after=phase==1?inc(wait_after):wait_after;
    wire new_peak=samples==0 || this_gap>=gap_max;
    wire [31:0] next_min=samples==0 || this_gap<gap_min?this_gap:gap_min;
    wire [9:0] next_slow=slow_count+{9'd0,(this_gap>SLOW_CYCLES)};
    wire [9:0] next_cap_errors=capture_errors+{9'd0,(cap_done && cap_error && capture_errors<999)};
    wire [9:0] next_proc_errors=process_errors+{9'd0,(processing_error && process_errors<999)};
    wire [32:0] shifted={remainder[31:0],quotient[31]};
    wire subtract=shifted>={1'b0,CYCLES_PER_MS};
    wire [31:0] next_quotient={quotient[30:0],subtract};
    wire [9:0] limited_ms=next_quotient>999?10'd999:next_quotient[9:0];
    wire [9:0] limited_count=snapshot[index]>999?10'd999:snapshot[index][9:0];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_sync<=0;vs_d<=0;cam_seen<=0;cam_active<=0;
            cam_timer<=0;high_timer<=0;cam_period<=0;cam_high<=0;
            cap_pending<=0;cap_running<=0;cap_timer<=0;cap_arm<=0;cap_wait<=0;cap_duration<=0;
            seen<=0;gap<=0;wait_before<=0;wait_after<=0;
            gap_min<=0;gap_max<=0;peak_before<=0;peak_after<=0;
            peak_period<=0;peak_high<=0;peak_arm<=0;peak_duration<=0;
            samples<=0;slow_count<=0;capture_errors<=0;process_errors<=0;
            for(i=0;i<12;i=i+1) snapshot[i]<=0;
            converting<=0;index<=0;bit_index<=0;quotient<=0;remainder<=0;
            converted<=0;data_bcd<=0;valid<=0;
        end else begin
            vs_sync<={vs_sync[1:0],vs};vs_d<=vs_sync[2];
            if(cam_seen) cam_timer<=inc(cam_timer);
            if(cam_active) high_timer<=inc(high_timer);
            if(vs_sync[2] && !vs_d) begin
                if(cam_seen) cam_period<=inc(cam_timer);
                cam_seen<=1;cam_active<=1;cam_timer<=0;high_timer<=0;
            end
            if(!vs_sync[2] && vs_d && cam_active) begin
                cam_high<=inc(high_timer);cam_active<=0;
            end
            if(cap_pending) cap_timer<=inc(cap_timer);
            if(cap_start) begin cap_pending<=1;cap_running<=0;cap_timer<=0;end
            if(cap_started && cap_pending) begin
                cap_arm<=inc(cap_timer);cap_running<=1;cap_timer<=0;
            end
            if(cap_done && cap_pending) begin
                if(cap_running) begin cap_wait<=cap_arm;cap_duration<=inc(cap_timer);end
                cap_pending<=0;cap_running<=0;
            end
            capture_errors<=next_cap_errors;process_errors<=next_proc_errors;
            if(seen) begin gap<=this_gap;wait_before<=this_before;wait_after<=this_after;end
            if(publish) begin
                seen<=1;gap<=0;wait_before<=0;wait_after<=0;
                if(seen) begin
                    gap_min<=next_min;slow_count<=next_slow;samples<=samples+1'b1;
                    if(new_peak) begin
                        gap_max<=this_gap;peak_before<=this_before;peak_after<=this_after;
                        peak_period<=cam_period;peak_high<=cam_high;
                        peak_arm<=cap_wait;peak_duration<=cap_duration;
                    end
                    if(samples==WINDOW-1) begin
                        samples<=0;slow_count<=0;
                        // Never stall any producer. Very short artificial windows
                        // can be skipped while conversion is busy, never mixed.
                        if(!converting) begin
                            snapshot[0]<=new_peak?cam_period:peak_period;
                            snapshot[1]<=new_peak?cam_high:peak_high;
                            snapshot[2]<=new_peak?cap_wait:peak_arm;
                            snapshot[3]<=new_peak?cap_duration:peak_duration;
                            snapshot[4]<=next_min;snapshot[5]<=new_peak?this_gap:gap_max;
                            snapshot[6]<=new_peak?this_before:peak_before;
                            snapshot[7]<=new_peak?this_after:peak_after;
                            snapshot[8]<={22'd0,next_cap_errors};snapshot[9]<={22'd0,next_proc_errors};
                            snapshot[10]<={22'd0,next_slow};snapshot[11]<=WINDOW;
                            quotient<=new_peak?cam_period:peak_period;
                            remainder<=0;bit_index<=0;index<=0;converting<=1;
                        end
                    end
                end
            end
            if(converting) begin
                if(index<8) begin
                    quotient<=next_quotient;
                    remainder<=subtract?shifted-{1'b0,CYCLES_PER_MS}:shifted;
                    if(bit_index==31) begin
                        converted[index*12 +: 12]<=to_bcd(limited_ms);
                        index<=index+1'b1;quotient<=snapshot[index+1'b1];
                        remainder<=0;bit_index<=0;
                    end else bit_index<=bit_index+1'b1;
                end else begin
                    converted[index*12 +: 12]<=to_bcd(limited_count);
                    if(index==11) begin
                        data_bcd<={to_bcd(limited_count),converted[131:0]};valid<=1;converting<=0;
                    end else index<=index+1'b1;
                end
            end
        end
    end
    // synthesis translate_off
    initial if(CLOCK_HZ<1000 || CLOCK_HZ%1000!=0 || WINDOW<1 || WINDOW>999 || SLOW_MS<0)
        $fatal(1,"Invalid schedule monitor parameters");
    // synthesis translate_on
endmodule
