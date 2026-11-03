`timescale 1ns/1ps
// Resolution is a property of the running image, never a writable UI register.
// The backend accepts level requests for STORE/QUIESCE and a single RECONFIG
// pulse. Successful reconfiguration resets this FPGA; there is no fake DONE.
module Profile_Switch_Controller #(
    parameter integer TIMEOUT_CYCLES = 96_000_000*5
)(
    input wire clk,rst_n,
    input wire [1:0] current_profile,requested_profile,
    input wire profile_switch_request,
    input wire backend_supported,backend_busy,backend_failed,
    input wire [1:0] backend_failure_reason,
    input wire boot_selection_stored,video_quiesced,
    output wire store_boot_selection,quiesce_video,
    output reg request_reconfiguration,
    output reg [1:0] target_profile,
    output reg [3:0] state,
    output wire busy,failed
);
    localparam [3:0] IDLE=0,REQUEST=1,VALIDATE_PROFILE=2,
        STORE_BOOT_SELECTION=3,QUIESCE_VIDEO=4,REQUEST_RECONFIG=5,
        UNSUPPORTED=6,FAILED=7,CFG_FAILED=8,DDR_FAILED=9,SEQUENCE_FAILED=10,TIMED_OUT=11;
    localparam integer TIMER_BITS=$clog2(TIMEOUT_CYCLES+1);
    reg [TIMER_BITS-1:0] timer;
    assign busy=state>=REQUEST && state<=REQUEST_RECONFIG;
    assign failed=state>=FAILED;
    function [3:0] failure_state(input [1:0] reason);
        case(reason)
            1: failure_state=CFG_FAILED;
            2: failure_state=DDR_FAILED;
            3: failure_state=SEQUENCE_FAILED;
            default: failure_state=FAILED;
        endcase
    endfunction
    assign store_boot_selection=state==STORE_BOOT_SELECTION;
    assign quiesce_video=state==QUIESCE_VIDEO || state==REQUEST_RECONFIG;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;target_profile<=current_profile;timer<=0;request_reconfiguration<=0;
        end else begin
            request_reconfiguration<=0;
            if(busy) begin
                if(backend_failed || timer==TIMEOUT_CYCLES-1) begin
                    state<=backend_failed?failure_state(backend_failure_reason):TIMED_OUT;timer<=0;
                end else begin
                    timer<=timer+1'b1;
                    case(state)
                        REQUEST: if(!backend_busy) state<=VALIDATE_PROFILE;
                        VALIDATE_PROFILE:
                            if(target_profile>1 || current_profile>1 || target_profile==current_profile)
                                state<=FAILED;
                            else if(!backend_supported) state<=UNSUPPORTED;
                            else state<=STORE_BOOT_SELECTION;
                        STORE_BOOT_SELECTION: if(boot_selection_stored) state<=QUIESCE_VIDEO;
                        QUIESCE_VIDEO: if(video_quiesced && !backend_busy) begin
                            state<=REQUEST_RECONFIG;request_reconfiguration<=1;
                        end
                        // Remain busy until a real reboot, backend failure, or timeout.
                        default: ;
                    endcase
                end
            end else if(backend_failed) begin
                // The hard configuration block preserves ERROR after fallback.
                // Report it after boot, without automatically retrying.
                // Preserve the first reason; subsequent errors must not
                // rewrite the evidence while the user reads the menu.
                if(!failed) state<=failure_state(backend_failure_reason);
                timer<=0;
            end else if(profile_switch_request) begin
                target_profile<=requested_profile;state<=REQUEST;timer<=0;
            end
        end
    end
endmodule
