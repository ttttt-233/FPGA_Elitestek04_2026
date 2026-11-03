`timescale 1ns/1ps

// Learn from pre-threshold NMS responses, not surviving edge count.
// During vertical blank scan/clear one of 64 histogram bins per clock.
// HIGH = 3/4 of the lower edge of the upper-12.5% bin (16-level bins).
// This leaves strong seeds on a constant step edge and rejects isolated peaks.
// Publish every four valid frames at frame entry with deadband and 1/4 smoothing.
// Default: restore the image-only policy verified before the FPS controller.
// RATE_GUARD_ENABLE=1 retains the experimental guarded policy for comparison.
// Keep the current frontend's earlier frame boundary to avoid threshold tearing.
// No extra DDR access or image-pipeline stalls. LOW remains HIGH/2.
module Canny_Adaptive_Threshold #(
    parameter [11:0] MIN_HIGH = 12'd16,
    parameter [11:0] MAX_HIGH = 12'd1024,
    parameter [11:0] DEFAULT_HIGH = 12'd96,
    parameter [19:0] MIN_SAMPLES = 20'd64,
    parameter [11:0] DEADBAND = 12'd8,
    parameter [11:0] MAX_STEP = 12'd8,
    parameter [11:0] AUTO_LIMIT_HIGH = 12'd256,
    parameter integer RATE_GUARD_ENABLE = 0
)(
    input wire clk,rst_n,
    input wire sample_vsync,sample_de,
    input wire frame_vsync,
    input wire [11:0] sample_magnitude,
    input wire adaptive_enable,
    input wire decrease_allowed,
    input wire [11:0] rate_floor_high,
    input wire [11:0] manual_high,
    output wire [11:0] threshold_low,
    output wire [11:0] threshold_high,
    output reg [11:0] adaptive_high,
    output reg [19:0] scene_samples,
    output reg [11:0] scene_high
);
    reg [19:0] nonzero_count,scan_samples;
    reg frame_vsync_d;
    reg sample_vsync_d,collecting,scan_busy,scan_valid,found;
    reg [5:0] scan_index;
    reg [20:0] cumulative,target;
    reg [11:0] candidate,estimate_high;
    reg estimate_valid,frame_was_adaptive;
    reg [1:0] update_phase;
    wire frame_start=sample_vsync && !sample_vsync_d;
    wire apply_start=frame_vsync && !frame_vsync_d;
    wire frame_end=!sample_vsync && sample_vsync_d;
    wire sample_valid=sample_de && sample_magnitude>=12'd16;
    wire [5:0] sample_bin=(sample_magnitude>=12'd1024)?6'd63:sample_magnitude[9:4];
    // One synchronous RAM port reads the next sample (or next scan bin),
    // while the other commits the preceding sample. Forward the last write
    // on a read/write collision: runs of identical bins must count every pixel.
    // Valid bits implement logical clearing, including asynchronous reset,
    // without an array reset that would turn the RAM into registers and muxes.
    reg [19:0] histogram [0:63];
    reg [19:0] histogram_q;
    reg [63:0] bin_valid;
    reg sample_pending,pending_bin_valid,last_write_valid;
    reg [5:0] pending_bin,last_write_bin;
    reg [19:0] last_write_value;
    wire accept_sample=sample_valid &&
        (frame_start?!scan_busy:(sample_vsync && collecting));
    wire [5:0] read_bin=scan_busy?(scan_index==0?6'd0:scan_index-6'd1):
        (frame_end && collecting)?6'd63:sample_bin;
    wire [19:0] sample_count=(last_write_valid && last_write_bin==pending_bin)?
        last_write_value:(pending_bin_valid?histogram_q:20'd0);
    wire [19:0] incremented_count=(sample_count==20'hfffff) ? sample_count : sample_count+20'd1;
    wire [19:0] scan_count=!bin_valid[scan_index]?20'd0:
        (last_write_valid && last_write_bin==scan_index)?last_write_value:histogram_q;
    wire [20:0] scan_sum=cumulative+{1'b0,scan_count};
    wire crossed=scan_valid && scan_sum>=target;

    always @(posedge clk) begin
        histogram_q<=histogram[read_bin];
        if(rst_n && sample_pending) histogram[pending_bin]<=incremented_count;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sample_pending<=0;pending_bin<=0;pending_bin_valid<=0;
            last_write_valid<=0;last_write_bin<=0;last_write_value<=0;
        end else begin
            sample_pending<=accept_sample;
            pending_bin<=sample_bin;
            pending_bin_valid<=bin_valid[sample_bin];
            last_write_valid<=sample_pending;
            last_write_bin<=pending_bin;
            last_write_value<=incremented_count;
        end
    end
    genvar bin;
    generate for(bin=0;bin<64;bin=bin+1) begin: g_bin_valid
        always @(posedge clk or negedge rst_n)
            if(!rst_n) bin_valid[bin]<=0;
            else if(scan_busy && scan_index==bin) bin_valid[bin]<=0;
            else if(sample_pending && pending_bin==bin) bin_valid[bin]<=1;
    end endgenerate

    function [11:0] clamp_high;
        input [11:0] value;
        begin
            clamp_high=(value<MIN_HIGH)?MIN_HIGH:(value>MAX_HIGH)?MAX_HIGH:value;
        end
    endfunction
    function [11:0] clamp_auto;
        input [11:0] value;
        begin
            clamp_auto=(value<MIN_HIGH)?MIN_HIGH:
                       (value>AUTO_LIMIT_HIGH)?AUTO_LIMIT_HIGH:value;
        end
    endfunction
    function [11:0] bin_high;
        input [5:0] bin;
        reg [11:0] value;
        begin
            // bin * 16 * 3/4. Use the lower bound, never the bin centre.
            value=({6'd0,bin}<<3)+({6'd0,bin}<<2);
            bin_high=clamp_high(value);
        end
    endfunction
    function [11:0] smooth_high;
        input [11:0] current_value,new_value;
        reg [12:0] difference,step;
        begin
            difference=(new_value>current_value)?
                {1'b0,new_value}-{1'b0,current_value}:{1'b0,current_value}-{1'b0,new_value};
            step=(difference+13'd3)>>2;
            if(step>{1'b0,MAX_STEP}) step={1'b0,MAX_STEP};
            if(difference<={1'b0,DEADBAND}) smooth_high=current_value;
            else if(new_value>current_value) smooth_high=current_value+step[11:0];
            else smooth_high=current_value-step[11:0];
        end
    endfunction

    reg [11:0] applied_high;
    wire update_estimate=!scan_busy && estimate_valid && update_phase==3;
    wire [11:0] next_estimate=update_estimate?smooth_high(adaptive_high,estimate_high):adaptive_high;
    wire [11:0] floor_value=clamp_auto(rate_floor_high);
    wire [11:0] image_value=clamp_auto(next_estimate);
    // The floor constrains descent too. Ignoring it when floor <= applied
    // would repeatedly step below a learned safe value and bounce back up.
    wire [11:0] requested_high=(floor_value>image_value)?floor_value:image_value;
    // Rate status may arrive mid-frame. Only applied_high is visible to NMS,
    // and it is latched once at frame entry, including the complete floor value.
    assign threshold_high=adaptive_enable?applied_high:manual_high;
    assign threshold_low=threshold_high>>1;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            adaptive_high<=DEFAULT_HIGH;applied_high<=DEFAULT_HIGH;sample_vsync_d<=0;nonzero_count<=0;
            scene_samples<=0;scene_high<=DEFAULT_HIGH;frame_vsync_d<=0;scan_samples<=0;
            collecting<=0;scan_busy<=0;scan_valid<=0;found<=0;scan_index<=0;
            cumulative<=0;target<=0;candidate<=DEFAULT_HIGH;estimate_high<=DEFAULT_HIGH;
            estimate_valid<=0;frame_was_adaptive<=0;update_phase<=0;
        end else begin
            sample_vsync_d<=sample_vsync;
            frame_vsync_d<=frame_vsync;
            if(apply_start) begin
                frame_was_adaptive<=adaptive_enable;
                if(!adaptive_enable) update_phase<=0;
                else if(!frame_was_adaptive) begin
                    adaptive_high<=RATE_GUARD_ENABLE?clamp_auto(manual_high):clamp_high(manual_high);
                    update_phase<=0;
                end else if(!scan_busy && estimate_valid) begin
                    if(update_phase==3) begin
                        adaptive_high<=RATE_GUARD_ENABLE?clamp_auto(next_estimate):next_estimate;
                        update_phase<=0;
                    end else update_phase<=update_phase+1'b1;
                end else update_phase<=0;
                if(adaptive_enable) begin
                    if(!frame_was_adaptive)
                        applied_high<=RATE_GUARD_ENABLE?clamp_auto(manual_high):clamp_high(manual_high);
                    // Image-only: exactly the old four-frame, deadband/quarter
                    // smoothing policy; FPS and its old floor cannot hold HIGH.
                    else if(!RATE_GUARD_ENABLE) applied_high<=next_estimate;
                    else if(requested_high>applied_high) begin
                        if(requested_high-applied_high>MAX_STEP) applied_high<=applied_high+MAX_STEP;
                        else applied_high<=requested_high;
                    end else if(decrease_allowed && requested_high<applied_high) begin
                        if(applied_high-requested_high>2) applied_high<=applied_high-12'd2;
                        else applied_high<=requested_high;
                    end
                end
            end
            if(frame_start) begin
                // Short blanking discards this frame's stats; complete clearing
                // before collecting another whole frame. Video is never stalled.
                collecting<=!scan_busy;
                nonzero_count<=0;
                if(!scan_busy && sample_valid) begin
                    nonzero_count<=1;
                end
            end else if(sample_vsync && collecting && sample_valid) begin
                if(nonzero_count!=20'hfffff) nonzero_count<=nonzero_count+1'b1;
            end else if(frame_end && collecting) begin
                collecting<=0;scan_busy<=1;scan_index<=63;cumulative<=0;found<=0;
                // ceil(count/8); sparse/empty frames preserve the learned value.
                target<={1'b0,nonzero_count[19:3]}+(nonzero_count[2:0]!=0);
                scan_valid<=nonzero_count>=MIN_SAMPLES;
                scan_samples<=nonzero_count;
                estimate_valid<=0;
            end
            if(scan_busy) begin
                cumulative<=scan_sum;
                if(!found && crossed) begin
                    candidate<=bin_high(scan_index);
                    found<=1;
                end
                if(scan_index==0) begin
                    scan_busy<=0;
                    estimate_high<=found?candidate:bin_high(scan_index);
                    estimate_valid<=scan_valid && (found || crossed);
                    scene_samples<=scan_samples;
                    if(scan_valid && (found || crossed)) scene_high<=found?candidate:bin_high(scan_index);
                end else scan_index<=scan_index-1'b1;
            end
        end
    end

    // synthesis translate_off
    initial if(MIN_HIGH==0 || MIN_HIGH>MAX_HIGH || MIN_HIGH>AUTO_LIMIT_HIGH || AUTO_LIMIT_HIGH>MAX_HIGH ||
               MAX_STEP==0 || MIN_SAMPLES==0)
        $fatal(1,"Invalid adaptive Canny threshold limits");
    // synthesis translate_on
endmodule
