`timescale 1ns/1ps

// Six UI levels. Media requests drive UI metadata, never DDR ownership.
module debug #(
    parameter integer DEBOUNCE_COUNT = 1_920_000,
    parameter integer COUNTER_WIDTH = 21,
    parameter [2:0] DEFAULT_MODE = 3'd3,
    parameter DEFAULT_EDGE_SELECT = 1'b0,
    parameter [2:0] LAST_MODE = 3'd6,
    parameter integer K1_LONG_PRESS_COUNT = 48_000_000,
    parameter integer K1_LONG_COUNTER_WIDTH = 27,
    parameter integer NOTICE_COUNT = 144_000_000,
    parameter integer NOTICE_COUNTER_WIDTH = 28,
    parameter integer LONG_PRESS_COUNT = 57_600_000,
    parameter integer HOLD_COUNTER_WIDTH = 26,
    parameter integer DOUBLE_CLICK_COUNT = 28_800_000,
    parameter integer DOUBLE_COUNTER_WIDTH = 25,
    parameter [11:0] DEFAULT_THRESHOLD = 12'd96,
    parameter [11:0] THRESHOLD_MIN = 12'd16,
    parameter [11:0] THRESHOLD_MAX = 12'd1024,
    parameter [11:0] THRESHOLD_STEP = 12'd16,
    parameter DEFAULT_OSD_ENABLE = 1'b1,
    parameter DEFAULT_DEBUG_ENABLE = 1'b0,
    parameter REMOTE_ENABLE = 0
)(
    input wire clk, rst_n, k1_n, k2_n,
    input wire remote_write,
    input wire [3:0] remote_address,
    input wire [15:0] remote_value,
    output wire key_event,
    output reg [2:0] mode_select,
    output reg edge_select,
    output reg [11:0] threshold_value,
    output wire osd_enable,
    output reg auto_threshold_enable,
    // Explicit PC AUTO selection also applies while the board is at THRESHOLD.
    // A physical gesture restores the board's manual-adjustment convention.
    output reg remote_auto_override,
    output reg control_notice,
    output reg [2:0] control_group,
    // User preferences survive both ALL OFF and temporary quick hide.
    // mask[0]: STATUS/CONTEXT; [1]: GUIDE; [2]: DEBUG.
    output reg [2:0] osd_mask,
    output reg osd_all_enable,
    output reg osd_quick_hidden,
    output reg [1:0] control_item,
    output reg shape_enable,
    // Future backend requests: IN/OUT/PRE/SNAP, PREV/NEXT/PLAY,
    // EDGE/FILTER notifications. OUTPUT has a separate reconfiguration request.
    output reg [8:0] operation_request,
    // Last reserved operation, for visible acknowledgement only.
    output reg [3:0] reserved_action,
    output reg gaussian_select,
    input wire [1:0] current_profile,
    output reg profile_switch_request,
    output reg [1:0] requested_profile
);
    localparam [2:0] CAPTURE=0, PLAYBACK=1, DISPLAY=2,
                     THRESHOLD=3, ALGORITHM=4, OSD=5;
    localparam [COUNTER_WIDTH-1:0] DEBOUNCE_LIMIT=DEBOUNCE_COUNT-1;
    localparam [NOTICE_COUNTER_WIDTH-1:0] NOTICE_LIMIT=NOTICE_COUNT-1;
    localparam [11:0] INITIAL_THRESHOLD = DEFAULT_THRESHOLD < THRESHOLD_MIN ? THRESHOLD_MIN :
                                         DEFAULT_THRESHOLD > THRESHOLD_MAX ? THRESHOLD_MAX : DEFAULT_THRESHOLD;
    reg [1:0] k1_sync, k2_sync, key_stable;
    reg [COUNTER_WIDTH-1:0] debounce_counter[0:1];
    wire [1:0] key_sync={k2_sync[1], k1_sync[1]};
    wire k1_single, k1_double, k1_long, k2_single, k2_double, k2_long;
    reg [NOTICE_COUNTER_WIDTH-1:0] notice_counter;
    wire [12:0] threshold_up={1'b0,threshold_value}+{1'b0,THRESHOLD_STEP};
    wire [12:0] lower_limit={1'b0,THRESHOLD_MIN}+{1'b0,THRESHOLD_STEP};
    integer i;

    // Capture/Playback owns the media backend.
    assign key_event=k1_single || k1_double || k1_long || k2_single || k2_double || k2_long;
    assign osd_enable=osd_all_enable && !osd_quick_hidden;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin k1_sync<=2'b11; k2_sync<=2'b11; end
        else begin k1_sync<={k1_sync[0],k1_n}; k2_sync<={k2_sync[0],k2_n}; end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            key_stable<=2'b11;
            for(i=0;i<2;i=i+1) debounce_counter[i]<=0;
        end else for(i=0;i<2;i=i+1) begin
            if(key_sync[i]==key_stable[i]) debounce_counter[i]<=0;
            else if(debounce_counter[i]==DEBOUNCE_LIMIT) begin
                key_stable[i]<=key_sync[i]; debounce_counter[i]<=0;
            end else debounce_counter[i]<=debounce_counter[i]+1'b1;
        end
    end
    UI_Key_Gesture #(.LONG_COUNT(K1_LONG_PRESS_COUNT),.HOLD_WIDTH(K1_LONG_COUNTER_WIDTH),
        .DOUBLE_COUNT(DOUBLE_CLICK_COUNT),.DOUBLE_WIDTH(DOUBLE_COUNTER_WIDTH)) k1_gesture (
        .clk(clk),.rst_n(rst_n),.key_n(key_stable[0]),.cancel(1'b0),
        .single_click(k1_single),.double_click(k1_double),.long_press(k1_long));
    // A level change discards pending clicks and a concurrently held K2.
    UI_Key_Gesture #(.LONG_COUNT(LONG_PRESS_COUNT),.HOLD_WIDTH(HOLD_COUNTER_WIDTH),
        .DOUBLE_COUNT(DOUBLE_CLICK_COUNT),.DOUBLE_WIDTH(DOUBLE_COUNTER_WIDTH)) k2_gesture (
        .clk(clk),.rst_n(rst_n),.key_n(key_stable[1]),
        .cancel(k1_long || (REMOTE_ENABLE && remote_write && remote_address==8)),
        .single_click(k2_single),.double_click(k2_double),.long_press(k2_long));

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            mode_select<=DEFAULT_MODE; threshold_value<=INITIAL_THRESHOLD;
            edge_select<=DEFAULT_EDGE_SELECT; gaussian_select<=0;
            shape_enable<=1;
            auto_threshold_enable<=0; control_group<=CAPTURE;
            remote_auto_override<=0;
            control_item<=0; reserved_action<=0;
            osd_mask<={DEFAULT_DEBUG_ENABLE,2'b11};
            osd_all_enable<=DEFAULT_OSD_ENABLE;
            osd_quick_hidden<=0;
            operation_request<=0;profile_switch_request<=0;
            requested_profile<=current_profile;
            notice_counter<=NOTICE_LIMIT; control_notice<=1;
        end else begin
            operation_request<=0;profile_switch_request<=0;
            if(key_event) remote_auto_override<=0;
            if(key_event || (REMOTE_ENABLE && remote_write)) begin
                notice_counter<=NOTICE_LIMIT; control_notice<=1;
            end else if(notice_counter!=0) begin
                notice_counter<=notice_counter-1'b1; control_notice<=1;
            end else control_notice<=0;

            // Protocol validation/arbitration stays in UART_Control. These
            // writes update the SAME settings/events used by physical keys.
            if(REMOTE_ENABLE && remote_write) case(remote_address)
                0:threshold_value<=remote_value[11:0];
                1:begin auto_threshold_enable<=remote_value[0];remote_auto_override<=1;end
                2:edge_select<=remote_value[0];
                3:shape_enable<=remote_value[0];
                4:osd_all_enable<=remote_value[0];
                5:osd_quick_hidden<=remote_value[0];
                6:osd_mask<=remote_value[2:0];
                7:mode_select<=remote_value[2:0];
                8:begin control_group<=remote_value[2:0];control_item<=remote_value[4:3];reserved_action<=0;remote_auto_override<=0;end
                9:gaussian_select<=remote_value[0];
                10:osd_mask[2]<=remote_value[0];
                14:begin requested_profile<=remote_value[1:0];profile_switch_request<=1;end
                11:begin
                    control_group<=remote_value[2:0]>=4?PLAYBACK:CAPTURE;
                    control_item<=0;reserved_action<={1'b0,remote_value[2:0]}+1'b1;
                    case(remote_value[2:0])
                        0:operation_request[0]<=1;1:operation_request[1]<=1;
                        2:operation_request[2]<=1;3:operation_request[3]<=1;
                        4:operation_request[4]<=1;5:operation_request[5]<=1;6:operation_request[6]<=1;
                    endcase
                end
            endcase
            else if(k1_long) begin
                control_group<=control_group==OSD?CAPTURE:control_group+1'b1;
                control_item<=0; reserved_action<=0;
            end else if(k1_double) begin
                // Temporary suppression only: the second double restores the
                // exact previous STATUS/GUIDE/DEBUG/ALL preferences.
                osd_quick_hidden<=~osd_quick_hidden;
            end else begin
                if(k1_single) case(control_group)
                    CAPTURE: begin reserved_action<=1; operation_request[0]<=1; end
                    PLAYBACK: begin reserved_action<=5; operation_request[4]<=1; end
                    DISPLAY: mode_select<=mode_select==0?LAST_MODE:mode_select-1'b1;
                    THRESHOLD: threshold_value<=({1'b0,threshold_value}<=lower_limit)?
                        THRESHOLD_MIN:threshold_value-THRESHOLD_STEP;
                    ALGORITHM: control_item<=control_item-1'b1;
                    OSD: control_item<=control_item-1'b1;
                endcase
                if(k2_single) case(control_group)
                    CAPTURE: begin reserved_action<=2; operation_request[1]<=1; end
                    PLAYBACK: begin reserved_action<=6; operation_request[5]<=1; end
                    DISPLAY: mode_select<=mode_select>=LAST_MODE?3'd0:mode_select+1'b1;
                    THRESHOLD: threshold_value<=threshold_up>{1'b0,THRESHOLD_MAX}?
                        THRESHOLD_MAX:threshold_up[11:0];
                    ALGORITHM: control_item<=control_item+1'b1;
                    OSD: control_item<=control_item+1'b1;
                endcase
                if(k2_long) case(control_group)
                    CAPTURE: begin reserved_action<=3; operation_request[2]<=1; end
                    PLAYBACK: begin reserved_action<=7; operation_request[6]<=1; end
                    THRESHOLD: auto_threshold_enable<=~auto_threshold_enable;
                    ALGORITHM: case(control_item)
                        0: begin edge_select<=~edge_select; operation_request[7]<=1; end
                        1: begin gaussian_select<=~gaussian_select; operation_request[8]<=1; end
                        2: if(current_profile==2'd0 || current_profile==2'd1) begin
                            requested_profile<=current_profile ^ 2'd1;
                            profile_switch_request<=1;
                        end
                        3: shape_enable<=~shape_enable;
                    endcase
                    OSD: case(control_item)
                        0: osd_mask[0]<=~osd_mask[0];
                        1: osd_mask[1]<=~osd_mask[1];
                        2: osd_mask[2]<=~osd_mask[2];
                        3: osd_all_enable<=~osd_all_enable;
                    endcase
                endcase
                if(k2_double && control_group==CAPTURE) begin
                    reserved_action<=4; operation_request[3]<=1;
                end
            end
        end
    end
endmodule

// UI-only media model. Eight photo indicators and the latest video descriptor
// exercise every HUD interaction without issuing a DDR read/write or freezing
// live video. Descriptors are volatile UI state, not proof of captured pixels.
// PRE models an eight-second window; OUT clamps the displayed duration to it.
// K1/K2 always browse media (including during video playback); no frame stepping.
module UI_Media_State #(
    parameter integer TENTH_COUNT=9_600_000,
    parameter integer TICK_WIDTH=24
)(
    input wire clk,rst_n,
    input wire [2:0] control_group,
    input wire [8:0] operation_request,
    output wire [51:0] media_status,
    output reg [2:0] feedback_code,
    output reg feedback_toggle
);
    localparam [TICK_WIDTH-1:0] TICK_LIMIT=TENTH_COUNT-1;
    reg [TICK_WIDTH-1:0] rolling_counter,record_counter,play_counter;
    reg [11:0] rolling_time,record_time,elapsed,duration;
    reg [3:0] snap_count,media_index;
    reg recording,clip_valid,playing;
    wire [3:0] media_count=snap_count+{3'd0,clip_valid};
    wire valid=media_count!=0 && media_index!=0 && media_index<=media_count;
    wire is_video=valid && clip_valid && media_index>snap_count;
    wire [11:0] next_elapsed=bcd_next(elapsed);
    assign media_status={valid,duration,elapsed,is_video && playing,is_video,
                         media_count,media_index,record_time,recording,snap_count};
    function [11:0] bcd_next;
        input [11:0] value;
        begin
            if(value==12'h999) bcd_next=value;
            else if(value[3:0]!=9) bcd_next=value+12'h001;
            else if(value[7:4]!=9) bcd_next={value[11:4]+8'h01,4'd0};
            else bcd_next={value[11:8]+4'd1,8'd0};
        end
    endfunction
    task feedback;
        input [2:0] code;
        begin feedback_code<=code;feedback_toggle<=~feedback_toggle;end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rolling_counter<=0;record_counter<=0;play_counter<=0;
            rolling_time<=0;record_time<=0;elapsed<=0;duration<=0;
            snap_count<=0;media_index<=0;recording<=0;clip_valid<=0;playing<=0;
            feedback_code<=0;feedback_toggle<=0;
        end else begin
            if(rolling_counter==TICK_LIMIT) begin
                rolling_counter<=0;
                if(rolling_time<12'h080) rolling_time<=bcd_next(rolling_time);
            end else rolling_counter<=rolling_counter+1'b1;
            if(recording) begin
                if(record_counter==TICK_LIMIT) begin record_counter<=0;record_time<=bcd_next(record_time);end
                else record_counter<=record_counter+1'b1;
            end else record_counter<=0;
            if(control_group!=1 || !is_video) begin playing<=0;play_counter<=0;end
            else if(playing) begin
                if(play_counter==TICK_LIMIT) begin
                    play_counter<=0;
                    if(next_elapsed>=duration) begin elapsed<=duration;playing<=0;feedback(6);end
                    else elapsed<=next_elapsed;
                end else play_counter<=play_counter+1'b1;
            end

            if(control_group==0) begin
                if(operation_request[0]) begin recording<=1;record_time<=0;record_counter<=0;end
                if(operation_request[1] && recording) begin
                    recording<=0;clip_valid<=1;media_index<=snap_count+4'd1;
                    duration<=record_time>12'h080 ? 12'h080 : (record_time==0 ? 12'h001 : record_time);
                    elapsed<=0;playing<=0;feedback(3);
                end
                if(operation_request[2]) begin
                    clip_valid<=1;media_index<=snap_count+4'd1;
                    duration<=rolling_time==0?12'h001:rolling_time;
                    elapsed<=0;playing<=0;feedback(2);
                end
                if(operation_request[3]) begin
                    if(snap_count<8) begin
                        snap_count<=snap_count+1'b1;
                        if(is_video) media_index<=media_index+1'b1;
                    end
                    if(media_index==0) media_index<=1;
                    feedback(1);
                end
            end else if(control_group==1) begin
                if((operation_request[4] || operation_request[5]) && valid) begin
                    if(operation_request[4]) media_index<=media_index==1?media_count:media_index-1'b1;
                    else media_index<=media_index>=media_count?4'd1:media_index+1'b1;
                    playing<=0;elapsed<=0;play_counter<=0;
                end else if(operation_request[6] && is_video) begin
                    if(!playing && elapsed>=duration) elapsed<=0;
                    playing<=~playing;play_counter<=0;feedback(playing?3'd5:3'd4);
                end
            end
        end
    end
endmodule

// Mutually exclusive gestures from one debounced, active-low key. Singles
// wait for the double-click window. A second press reserves the gesture until
// release (double) or the hold limit (long); no speculative single is emitted.
module UI_Key_Gesture #(
    parameter integer LONG_COUNT=57_600_000,
    parameter integer HOLD_WIDTH=26,
    parameter integer DOUBLE_COUNT=28_800_000,
    parameter integer DOUBLE_WIDTH=25
)(
    input wire clk,rst_n,key_n,cancel,
    output reg single_click,double_click,long_press
);
    localparam [HOLD_WIDTH-1:0] LONG_LIMIT=LONG_COUNT-1;
    localparam [DOUBLE_WIDTH-1:0] DOUBLE_LIMIT=DOUBLE_COUNT-1;
    reg previous_n,held,long_sent,pending,second,blocked;
    reg [HOLD_WIDTH-1:0] hold_count;
    reg [DOUBLE_WIDTH-1:0] click_count;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            previous_n<=1; held<=0; long_sent<=0; pending<=0; second<=0; blocked<=0;
            hold_count<=0; click_count<=0;
            single_click<=0; double_click<=0; long_press<=0;
        end else begin
            previous_n<=key_n;
            single_click<=0; double_click<=0; long_press<=0;
            if(cancel) begin
                held<=0; pending<=0; second<=0; long_sent<=0;
                hold_count<=0; click_count<=0; blocked<=!key_n;
            end else if(blocked) begin
                if(key_n) blocked<=0;
            end else if(previous_n && !key_n) begin
                held<=1; long_sent<=0; hold_count<=0;
                second<=pending; pending<=0; click_count<=0;
            end else if(!previous_n && key_n) begin
                held<=0; hold_count<=0;
                if(!long_sent) begin
                    if(second) begin double_click<=1; second<=0; end
                    else begin pending<=1; click_count<=0; end
                end
            end else if(held && !key_n && !long_sent) begin
                if(hold_count==LONG_LIMIT) begin
                    long_press<=1; long_sent<=1; pending<=0; second<=0;
                end else hold_count<=hold_count+1'b1;
            end else if(pending) begin
                if(click_count==DOUBLE_LIMIT) begin single_click<=1; pending<=0; end
                else click_count<=click_count+1'b1;
            end
        end
    end
endmodule
