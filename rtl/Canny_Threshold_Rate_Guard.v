`timescale 1ns/1ps

// Bounded integral load floor with gain scheduling, dead band, settling and
// anti-windup. This is not a continuous PID. Use measured publication periods,
// not the delayed display FPS. Eight settling results precede eight measured
// intervals. Four unsuccessful applied increases lock further increases until
// persistent scene change or measured recovery. Publication time includes
// shape analysis and other costs that raising an edge threshold cannot fix.
module Canny_Threshold_Rate_Guard #(
    parameter integer CLOCK_HZ=96000000,
    parameter integer TARGET_FPS=30,
    parameter [11:0] MIN_HIGH=12'd16,
    // MAX_HIGH is the manual range; AUTO seeds and automatic rises are capped.
    parameter [11:0] MAX_HIGH=12'd1024,
    parameter [11:0] AUTO_LIMIT_HIGH=12'd256
)(
    input wire clk,rst_n,frame_done,
    input wire adaptive_enable,
    input wire [11:0] applied_high,scene_high,
    input wire [19:0] scene_samples,
    output reg [11:0] rate_floor_high,
    output reg decrease_allowed
);
    localparam integer TIMER_BITS=$clog2(CLOCK_HZ+1);
    // Preserve the original 30/32/28/24 thresholds for the quality profile.
    // The 60 FPS profile uses 60/64/56/48 with identical damping and bounds.
    localparam integer SLOW_PERIOD=CLOCK_HZ/TARGET_FPS;
    localparam integer FAST_SUM=(CLOCK_HZ/(TARGET_FPS*32/30))*8;
    localparam integer SLOW_SUM=(CLOCK_HZ/TARGET_FPS)*8;
    localparam integer MEDIUM_SUM=(CLOCK_HZ/(TARGET_FPS*28/30))*8;
    localparam integer SEVERE_SUM=(CLOCK_HZ/(TARGET_FPS*24/30))*8;
    reg [TIMER_BITS-1:0] interval;
    reg [TIMER_BITS+2:0] measured_sum;
    reg [3:0] count;
    reg seen,enabled_d,fast_window;
    reg [11:0] learned_min,reference_high;
    reg [19:0] reference_samples;
    reg [1:0] scene_change_count;
    reg probe_valid,rise_blocked;
    reg [1:0] ineffective_count;
    reg [TIMER_BITS+2:0] probe_sum;
    reg [11:0] probe_high;
    wire [TIMER_BITS:0] period={1'b0,interval}+1'b1;
    wire [TIMER_BITS+2:0] total=measured_sum+period;
    wire [19:0] sample_difference=(scene_samples>reference_samples)?
        scene_samples-reference_samples:reference_samples-scene_samples;
    wire [11:0] high_difference=(scene_high>reference_high)?
        scene_high-reference_high:reference_high-scene_high;
    wire scene_changed=(sample_difference>=20'd64 && sample_difference>(reference_samples>>2)) ||
        (high_difference>=12'd16 && high_difference>(reference_high>>2));
    wire release_memory=scene_changed && scene_change_count==2;
    wire [11:0] active_min=release_memory?MIN_HIGH:learned_min;
    wire [11:0] base=(rate_floor_high>applied_high)?rate_floor_high:applied_high;
    wire [3:0] step_up=(total>SEVERE_SUM)?4'd8:(total>MEDIUM_SUM)?4'd4:4'd2;
    wire [11:0] down_value=(rate_floor_high>MIN_HIGH+2)?rate_floor_high-12'd2:MIN_HIGH;
    // Compare with the last useful baseline, so small cumulative gains count.
    // A 1/64 period reduction is evidence of benefit, not single-frame jitter.
    wire improved=probe_sum>total && (probe_sum-total)>=(probe_sum>>6);
    wire applied_probe=applied_high>=rate_floor_high && applied_high>probe_high;
    wire begin_probe=!probe_valid || release_memory;
    wire stop_rise=probe_valid && !release_memory && applied_probe &&
                   !improved && ineffective_count==3;
    wire allow_rise=begin_probe || (applied_probe && !stop_rise && (!rise_blocked || improved));
    function [11:0] bounded_auto_add;
        input [11:0] value;
        input [3:0] amount;
        reg [12:0] sum;
        begin
            sum={1'b0,value}+amount;
            bounded_auto_add=(sum>AUTO_LIMIT_HIGH)?AUTO_LIMIT_HIGH:
                             (sum<MIN_HIGH)?MIN_HIGH:sum[11:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            interval<=0;measured_sum<=0;count<=0;seen<=0;enabled_d<=0;
            rate_floor_high<=MIN_HIGH;learned_min<=MIN_HIGH;decrease_allowed<=0;
            fast_window<=0;reference_samples<=0;reference_high<=0;scene_change_count<=0;
            probe_valid<=0;rise_blocked<=0;ineffective_count<=0;probe_sum<=0;probe_high<=0;
        end else begin
            enabled_d<=adaptive_enable;
            if(!adaptive_enable || !enabled_d) begin
                interval<=0;measured_sum<=0;count<=0;seen<=0;
                rate_floor_high<=bounded_auto_add(applied_high,0);learned_min<=MIN_HIGH;
                decrease_allowed<=0;fast_window<=0;scene_change_count<=0;
                reference_samples<=scene_samples;reference_high<=scene_high;
                probe_valid<=0;rise_blocked<=0;ineffective_count<=0;
            end else begin
                if(interval<CLOCK_HZ) interval<=interval+1'b1;
                // No publications: close descent immediately, but never integrate
                // repeatedly from one old result. Timer saturates after one second.
                if(seen && interval>=SLOW_PERIOD) decrease_allowed<=0;
                if(frame_done) begin
                    interval<=0;seen<=1;
                    if(!seen) begin count<=0;measured_sum<=0;end
                    else begin
                        count<=count+1'b1;
                        if(count>=8) measured_sum<=total;
                        if(count==15) begin
                            count<=0;measured_sum<=0;
                            // Independent pre-threshold scene evidence. Three
                            // changed windows release only learned memory, never
                            // jump the current floor or erase it on a fast result.
                            if(release_memory) begin
                                reference_samples<=scene_samples;reference_high<=scene_high;
                                scene_change_count<=0;learned_min<=MIN_HIGH;
                                probe_valid<=0;rise_blocked<=0;ineffective_count<=0;
                            end else if(scene_changed) scene_change_count<=scene_change_count+1'b1;
                            else scene_change_count<=0;
                            if(total>SLOW_SUM) begin
                                decrease_allowed<=0;fast_window<=0;
                                if(stop_rise) rise_blocked<=1;
                                if(allow_rise) begin
                                    rate_floor_high<=bounded_auto_add(base,step_up);
                                    if(begin_probe || improved) begin
                                        probe_valid<=1;probe_sum<=total;probe_high<=base;
                                        ineffective_count<=0;rise_blocked<=0;
                                    end else ineffective_count<=ineffective_count+1'b1;
                                    // Retain same-scene protection against re-entering
                                    // a level known to overload the edge pipeline.
                                    if(bounded_auto_add(base,2)>active_min)
                                        learned_min<=bounded_auto_add(base,2);
                                end
                            end else if(total<=FAST_SUM) begin
                                probe_valid<=0;rise_blocked<=0;ineffective_count<=0;
                                fast_window<=1;
                                if(fast_window) begin
                                    decrease_allowed<=1;
                                    rate_floor_high<=(down_value>active_min)?down_value:active_min;
                                end
                            end else begin
                                probe_valid<=0;rise_blocked<=0;ineffective_count<=0;
                                fast_window<=0;decrease_allowed<=0;
                            end
                        end
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial if(CLOCK_HZ<3200 || CLOCK_HZ>200000000 || MIN_HIGH==0 ||
               MIN_HIGH>=MAX_HIGH || MIN_HIGH>AUTO_LIMIT_HIGH ||
               AUTO_LIMIT_HIGH>MAX_HIGH)
        $fatal(1,"Invalid threshold rate controller parameters");
    // synthesis translate_on
endmodule
