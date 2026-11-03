`timescale 1ns/1ps
// Titanium internal reconfiguration interface (Efinix AN 010).
// The combined Remote Update Flash Image owns the address table. Profile 0
// selects the golden 720p image and profile 1 selects the 480p image. Selection
// is captured by the hard configuration block; no flash sector is rewritten.
module Profile_Reconfig_Backend #(
    parameter integer NOTICE_CYCLES=24_000_000
)(
    input wire clk,rst_n,
    input wire [1:0] target_profile,
    input wire store_boot_selection,quiesce_video,request_reconfiguration,
    input wire traffic_idle,traffic_error,
    input wire cfg_error,
    output reg boot_selection_stored,
    output wire video_quiesced,backend_failed,
    output wire [1:0] failure_reason,
    output reg [1:0] cfg_cbsel,
    output reg cfg_ena,
    output wire cfg_config
);
    localparam integer WAIT_CYCLES=NOTICE_CYCLES<4?4:NOTICE_CYCLES;
    localparam integer WAIT_BITS=$clog2(WAIT_CYCLES+1);
    reg [WAIT_BITS-1:0] capture_count;
    (* async_reg="true" *) reg [2:0] error_sync;
    reg sequence_error;
    reg config_armed;
    wire active=store_boot_selection || quiesce_video;
    assign backend_failed=error_sync[2] || traffic_error || sequence_error;
    assign failure_reason=error_sync[2]?2'd1:traffic_error?2'd2:sequence_error?2'd3:2'd0;
    assign cfg_config=config_armed && active && !backend_failed;
    assign video_quiesced=quiesce_video && boot_selection_stored && traffic_idle && !backend_failed;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) error_sync<=0;
        else error_sync<={error_sync[1:0],cfg_error};
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            capture_count<=0;boot_selection_stored<=0;cfg_cbsel<=0;
            cfg_ena<=0;config_armed<=0;sequence_error<=0;
        end else if(!active || backend_failed) begin
            capture_count<=0;boot_selection_stored<=0;cfg_ena<=0;config_armed<=0;
            if(!active) sequence_error<=0;
        end else begin
            if(store_boot_selection && !boot_selection_stored) begin
                if(target_profile>1) sequence_error<=1;
                else if(!cfg_ena) begin
                    cfg_cbsel<=target_profile;cfg_ena<=1;capture_count<=0;
                end else if(cfg_cbsel!=target_profile) sequence_error<=1;
                else if(capture_count==WAIT_CYCLES-1) boot_selection_stored<=1;
                else capture_count<=capture_count+1'b1;
            end
            if(request_reconfiguration) begin
                if(video_quiesced && cfg_ena) config_armed<=1;
                else sequence_error<=1;
            end
            // CONFIG is asynchronous and stays asserted until actual device
            // reconfiguration, or controller failure/timeout cancels the flow.
        end
    end
endmodule
