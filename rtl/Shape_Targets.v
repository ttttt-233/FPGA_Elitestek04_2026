`timescale 1ns/1ps
// Packed descriptors: {kind[2:0], left[11:0], top[11:0], right[11:0], bottom[11:0]}.
// Slot zero is least significant. Bounded streaming selection, 1..8 targets.
module Shape_Target_Bank #(parameter TARGETS=4,PREFER_CONTAINING=0,
    ALLOW_CROSS_KIND_CONTAINMENT=0,AREA_BITS=32)(
    input wire clk,rst_n,clear,accept,
    input wire [50:0] candidate,
    input wire [3:0] candidate_digit,
    input wire [1:0] quality,
    input wire [31:0] area,
    output wire [TARGETS*51-1:0] results,
    output wire [TARGETS*4-1:0] digits,
    output wire [TARGETS*2-1:0] qualities,
    output wire ready
);
    // accept is sampled only when ready. Inputs may change afterwards; clear
    // cancels any pending transaction. Published slots stay atomic while busy.
    reg [50:0] boxes[0:TARGETS-1], work_boxes[0:TARGETS-1];
    reg [3:0] digit_values[0:TARGETS-1], work_digits[0:TARGETS-1];
    reg [1:0] grades[0:TARGETS-1], work_grades[0:TARGETS-1];
    reg [AREA_BITS-1:0] areas[0:TARGETS-1], work_areas[0:TARGETS-1];
    localparam RB=$clog2(TARGETS+1);
    localparam IDLE=0,FILTER=1,DECIDE=2,PACK=3,COMMIT=4;
    reg [2:0] state;
    reg [50:0] pending_box;
    reg [3:0] pending_digit;
    reg [1:0] pending_grade;
    reg [AREA_BITS-1:0] pending_area;
    reg [TARGETS-1:0] keep;
    reg drop_candidate;
    reg [RB-1:0] cursor,candidate_rank,write_count;
    assign ready=rst_n && !clear && state==IDLE;
    wire [50:0] slot_box=cursor<TARGETS?boxes[cursor]:51'd0;
    wire [3:0] slot_digit=cursor<TARGETS?digit_values[cursor]:4'd0;
    wire [1:0] slot_grade=cursor<TARGETS?grades[cursor]:2'd0;
    wire [31:0] slot_area=cursor<TARGETS?areas[cursor]:32'd0;
    function contains;
        input [50:0] a,b;
        begin contains=a[50:48]!=0 && b[50:48]!=0 &&
            a[47:36]<=b[47:36] && a[35:24]<=b[35:24] &&
            a[23:12]>=b[23:12] && a[11:0]>=b[11:0];end
    endfunction
    function better;
        input [50:0] a,b;
        input [1:0] qa,qb;
        input [31:0] aa,ab;
        begin better=a[50:48]!=0 && (b[50:48]==0 || qa>qb || (qa==qb &&
            (aa>ab || (aa==ab && {a[35:24],a[47:36]}<{b[35:24],b[47:36]}))));end
    endfunction
    wire in_box=contains(slot_box,pending_box),out_box=contains(pending_box,slot_box);
    wire small_box={1'b0,slot_area}>({1'b0,pending_area}<<1);
    wire large_box={1'b0,pending_area}>({1'b0,slot_area}<<1);
    wire duplicate=(in_box || out_box) && !small_box && !large_box &&
        slot_box[50:48]==pending_box[50:48] &&
        (pending_box[50:48]!=3'd4 || slot_digit==pending_digit);
    wire same_kind=slot_box[50:48]==pending_box[50:48];
    wire reject_slot=PREFER_CONTAINING ?
        (same_kind && in_box && small_box) || (duplicate &&
            (pending_area<slot_area || (pending_area==slot_area && pending_grade<=slot_grade))):
        ((same_kind || !ALLOW_CROSS_KIND_CONTAINMENT) && out_box && large_box) || (duplicate &&
            (pending_area<slot_area || (pending_area==slot_area && pending_grade<=slot_grade)));
    wire keep_slot=slot_box[50:48]!=0 &&
        !(PREFER_CONTAINING ? (same_kind && out_box && large_box) || duplicate :
            ((same_kind || !ALLOW_CROSS_KIND_CONTAINMENT) && in_box && small_box) || duplicate);
    wire insert_here=write_count==candidate_rank;
    wire emit_slot=state==PACK && (insert_here || cursor==TARGETS || keep[cursor]);
    wire [50:0] emit_box=insert_here?pending_box:slot_box;
    wire [3:0] emit_digit=insert_here?pending_digit:slot_digit;
    wire [1:0] emit_grade=insert_here?pending_grade:slot_grade;
    wire [31:0] emit_area=insert_here?pending_area:slot_area;
    // N comparisons, one decision, at most 2*N packing clocks, one commit:
    // ready returns within 3*N+2 clocks after acceptance (20 for six slots).
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;pending_box<=0;pending_digit<=0;pending_grade<=0;pending_area<=0;
            keep<=0;drop_candidate<=0;cursor<=0;candidate_rank<=0;write_count<=0;
        end else if(clear) state<=IDLE;
        else case(state)
            IDLE:if(accept && candidate[50:48]!=0 && quality!=0) begin
                pending_box<=candidate;
                pending_digit<=PREFER_CONTAINING || candidate[50:48]==3'd4 ? candidate_digit : 4'd0;
                pending_grade<=quality;pending_area<=area;
                keep<=0;drop_candidate<=0;cursor<=0;candidate_rank<=0;write_count<=0;
                state<=FILTER;
            end
            FILTER:begin
                keep[cursor]<=keep_slot;
                if(reject_slot) drop_candidate<=1;
                // Strict comparison leaves an older equal-rank slot first.
                if(keep_slot && !better(pending_box,slot_box,pending_grade,slot_grade,pending_area,slot_area))
                    candidate_rank<=candidate_rank+1'b1;
                if(cursor==TARGETS-1) state<=DECIDE;
                else cursor<=cursor+1'b1;
            end
            DECIDE:begin cursor<=0;state<=drop_candidate?IDLE:PACK;end
            PACK:begin
                if(!insert_here && cursor<TARGETS) cursor<=cursor+1'b1;
                if(emit_slot) begin
                    if(write_count==TARGETS-1) state<=COMMIT;
                    else write_count<=write_count+1'b1;
                end
            end
            COMMIT:state<=IDLE;
            default:state<=IDLE;
        endcase
    end
    genvar g;
    generate for(g=0;g<TARGETS;g=g+1) begin: g_slot
        assign results[g*51 +: 51]=boxes[g];assign digits[g*4 +: 4]=digit_values[g];
        assign qualities[g*2 +: 2]=grades[g];
        always @(posedge clk or negedge rst_n)
            if(!rst_n) begin boxes[g]<=0;digit_values[g]<=0;grades[g]<=0;areas[g]<=0;end
            else if(clear) begin boxes[g]<=0;digit_values[g]<=0;grades[g]<=0;areas[g]<=0;end
            else if(state==COMMIT) begin
                boxes[g]<=work_boxes[g];digit_values[g]<=work_digits[g];
                grades[g]<=work_grades[g];areas[g]<=work_areas[g];
            end
        // Fixed-neighbour shifts replace the N-by-N descriptor scatter. All
        // N scratch slots are overwritten before COMMIT, so need no reset.
        if(g==TARGETS-1) begin: g_tail
            always @(posedge clk) if(emit_slot) begin
                work_boxes[g]<=emit_box;work_digits[g]<=emit_digit;
                work_grades[g]<=emit_grade;work_areas[g]<=emit_area;
            end
        end else begin: g_shift
            always @(posedge clk) if(emit_slot) begin
                work_boxes[g]<=work_boxes[g+1];work_digits[g]<=work_digits[g+1];
                work_grades[g]<=work_grades[g+1];work_areas[g]<=work_areas[g+1];
            end
        end
    end endgenerate
endmodule

// Disable aborts classifier/tracker work and bypasses the OUT result wait.
// Re-enable during a raster keeps bypassing that raster until the next start;
// otherwise a cancelled result could deadlock video publication indefinitely.
module Shape_Runtime_Control #(parameter TARGETS=6)(
    input wire clk,rst_n,enable,analysis_start,analysis_valid,tracked_ready,
    input wire [TARGETS*51-1:0] tracked_shapes,
    input wire [TARGETS*4-1:0] tracked_digits,
    input wire [TARGETS*2-1:0] tracked_qualities,
    output wire detector_reset_n,detector_start,detector_pixel_valid,analysis_ready,
    output wire [TARGETS*51-1:0] completed_shapes,
    output wire [TARGETS*4-1:0] completed_digits,
    output wire [TARGETS*2-1:0] completed_qualities
);
    reg frame_enabled;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) frame_enabled<=0;
        else if(!enable) frame_enabled<=0;
        else if(analysis_start) frame_enabled<=1;
    assign detector_reset_n=rst_n && enable;
    assign detector_start=analysis_start && enable;
    assign detector_pixel_valid=analysis_valid && enable && frame_enabled;
    assign analysis_ready=!enable || !frame_enabled || tracked_ready;
    assign completed_shapes=enable && frame_enabled?tracked_shapes:{(TARGETS*51){1'b0}};
    assign completed_digits=enable && frame_enabled?tracked_digits:{(TARGETS*4){1'b0}};
    assign completed_qualities=enable && frame_enabled?tracked_qualities:{(TARGETS*2){1'b0}};
endmodule

// Serial nearest-pair association bounds the combinational path as N grows.
// Each old/current slot is used at most once. This is greedy association, not
// a maximum-cardinality assignment in ambiguous scenes. Persistent identity
// follows this SAME association, independently of current Top-N slot order.
// Only publish commits history; clear cancels unfinished or failed work.
module Shape_Target_Track #(parameter TARGETS=4,PERSISTENT_ENABLE=1,PROFILE_FPS=30,
    STRONG_RETAIN_FRAMES=(PROFILE_FPS+1)/2,LOST_TIMEOUT_FRAMES=PROFILE_FPS*2)(
    input wire clk,rst_n,clear,raw_valid,publish,
    input wire [TARGETS*51-1:0] raw_results,
    input wire [TARGETS*4-1:0] raw_digits,
    input wire [TARGETS*2-1:0] qualities,
    output reg ready,
    output reg [TARGETS*51-1:0] confirmed,
    output reg [TARGETS*4-1:0] confirmed_digits,
    output reg [TARGETS*2-1:0] confirmed_qualities,
    output reg [TARGETS*8-1:0] track_ids,track_ages,
    output reg [TARGETS*8-1:0] track_lost,
    output reg [TARGETS*13-1:0] track_vx,track_vy,
    output reg [TARGETS*(TARGETS<2?1:$clog2(TARGETS))-1:0] track_banks,
    output wire [TARGETS*51-1:0] track_boxes,
    output wire [TARGETS*51-1:0] track_raw_boxes,
    output wire [TARGETS*4-1:0] track_digits,
    output wire [TARGETS-1:0] track_alive,track_observed,
    output reg [TARGETS-1:0] track_trail_reset,
    output wire track_snapshot_ready,
    output wire [TARGETS*2-1:0] track_qualities
);
    localparam IB=TARGETS<2?1:$clog2(TARGETS);
    localparam WAIT=0,SEARCH=1,CHOOSE=2,OUTPUT=3,DONE=4,RETAIN=5,ALLOCATE=6;
    reg [2:0] state;
    reg idle_snapshot_valid;
    // A cancelled SEARCH/RETAIN must not expose its speculative partial bank.
    always @(posedge clk or negedge rst_n)
        if(!rst_n) idle_snapshot_valid<=0;
        else if(clear) idle_snapshot_valid<=state==DONE || (state==WAIT && idle_snapshot_valid);
        else if(state==WAIT && raw_valid) idle_snapshot_valid<=0;
    assign track_snapshot_ready=rst_n && !clear &&
        ((state==WAIT && !raw_valid && idle_snapshot_valid) || state==DONE);
    reg [TARGETS*51-1:0] previous,current,previous_boxes,stable_boxes;
    reg [TARGETS*4-1:0] previous_digits,current_digits,stable_digits,previous_stable_digits;
    reg [TARGETS*2-1:0] digit_votes,next_digit_votes;
    reg [TARGETS*2-1:0] kind_votes,next_kind_votes;
    reg [TARGETS*3-1:0] streak,pending;
    reg [TARGETS*2-1:0] grades;
    reg [TARGETS-1:0] locked,matched_locked;
    // Triangle-only hysteresis: acquire from three good observations in four
    // publications. One weak/missing observation may retain association, but
    // an absent target NEVER draws an old box on the current image.
    reg [TARGETS*4-1:0] triangle_history,next_history;
    reg [TARGETS-1:0] degraded,next_degraded,pending_locked,blocked_previous;
    reg [TARGETS-1:0] used_current,used_previous;
    reg [TARGETS*8-1:0] previous_ids,previous_ages;
    reg [TARGETS*8-1:0] previous_lost;
    reg [TARGETS*IB-1:0] previous_banks;
    reg [TARGETS-1:0] reserved_banks;
    // Allocation is speculative until publish, just like existing evidence.
    reg [7:0] next_track_id,pending_next_id;
    reg [7:0] old_id,old_age;
    reg [7:0] old_lost;
    reg [IB-1:0] old_bank;
    assign track_boxes=stable_boxes;
    // Held through clear and the next raster; changes only on raw_valid or
    // during RETAIN, well before ready. Trail can serialize these centers.
    assign track_raw_boxes=current;
    assign track_digits=stable_digits;
    genvar ts;
    generate for(ts=0;ts<TARGETS;ts=ts+1) begin: g_lifecycle
        // The frame-publication ports clear during the next CNN analysis.
        // UART queries use the same held bank as ID/box, without another copy.
        assign track_qualities[ts*2 +: 2]=pending_locked[ts]?grades[ts*2 +: 2]:2'd0;
        assign track_alive[ts]=pending[ts*3 +: 3]!=0;
        assign track_observed[ts]=grades[ts*2 +: 2]!=0 && pending[ts*3 +: 3]!=0 &&
            track_lost[ts*8 +: 8]==0;
    end endgenerate
    reg [IB-1:0] i,j,best_i,best_j;
    reg found;
    reg [14:0] best_distance;
    // SEARCH, CHOOSE and RETAIN never run together. Share their read ports
    // instead of building separate wide variable-part-select networks for
    // i, j, best_i and best_j. Fixed slots also avoid index*51 barrel shifts.
    wire [IB-1:0] current_slot=state==CHOOSE?best_i:i;
    wire [IB-1:0] previous_slot=state==CHOOSE?best_j:(state==RETAIN?i:j);
    reg [50:0] a,b,old_stable;
    reg [3:0] a_digit,b_digit,old_stable_digit;
    reg [1:0] old_digit_vote;
    reg [1:0] old_kind_vote;
    reg [1:0] current_grade;
    reg [2:0] old_streak,history_kind;
    reg [3:0] old_history;
    reg old_locked,old_degraded;
    integer read_slot;
    always @(*) begin
        a=0;b=0;old_stable=0;a_digit=0;b_digit=0;old_stable_digit=0;
        old_digit_vote=0;old_kind_vote=0;current_grade=0;old_streak=0;
        old_history=0;old_locked=0;old_degraded=0;history_kind=0;
        old_id=0;old_age=0;old_lost=0;old_bank=0;
        for(read_slot=0;read_slot<TARGETS;read_slot=read_slot+1) begin
            a=a | (current[read_slot*51 +: 51] & {51{current_slot==read_slot}});
            a_digit=a_digit | (current_digits[read_slot*4 +: 4] & {4{current_slot==read_slot}});
            current_grade=current_grade | (grades[read_slot*2 +: 2] & {2{current_slot==read_slot}});
            b=b | (previous[read_slot*51 +: 51] & {51{previous_slot==read_slot}});
            b_digit=b_digit | (previous_digits[read_slot*4 +: 4] & {4{previous_slot==read_slot}});
            old_stable=old_stable | (previous_boxes[read_slot*51 +: 51] & {51{previous_slot==read_slot}});
            old_stable_digit=old_stable_digit | (previous_stable_digits[read_slot*4 +: 4] & {4{previous_slot==read_slot}});
            old_digit_vote=old_digit_vote | (digit_votes[read_slot*2 +: 2] & {2{previous_slot==read_slot}});
            old_kind_vote=old_kind_vote | (kind_votes[read_slot*2 +: 2] & {2{previous_slot==read_slot}});
            old_streak=old_streak | (streak[read_slot*3 +: 3] & {3{previous_slot==read_slot}});
            old_history=old_history | (triangle_history[read_slot*4 +: 4] & {4{previous_slot==read_slot}});
            old_locked=old_locked | (locked[read_slot] && previous_slot==read_slot);
            old_degraded=old_degraded | (degraded[read_slot] && previous_slot==read_slot);
            history_kind=history_kind | (current[read_slot*51+48 +: 3] & {3{j==read_slot}});
            old_id=old_id | (previous_ids[read_slot*8 +: 8] & {8{previous_slot==read_slot}});
            old_age=old_age | (previous_ages[read_slot*8 +: 8] & {8{previous_slot==read_slot}});
            old_lost=old_lost | (previous_lost[read_slot*8 +: 8] & {8{previous_slot==read_slot}});
            old_bank=old_bank | (previous_banks[read_slot*IB +: IB] & {IB{previous_slot==read_slot}});
        end
    end
    wire [12:0] w={1'b0,a[23:12]}-{1'b0,a[47:36]}+13'd1;
    wire [12:0] h={1'b0,a[11:0]}-{1'b0,a[35:24]}+13'd1;
    wire [12:0] pw={1'b0,b[23:12]}-{1'b0,b[47:36]}+13'd1;
    wire [12:0] ph={1'b0,b[11:0]}-{1'b0,b[35:24]}+13'd1;
    wire [11:0] l=a[47:36]>b[47:36]?a[47:36]:b[47:36];
    wire [11:0] r=a[23:12]<b[23:12]?a[23:12]:b[23:12];
    wire [11:0] t=a[35:24]>b[35:24]?a[35:24]:b[35:24];
    wire [11:0] bt=a[11:0]<b[11:0]?a[11:0]:b[11:0];
    wire [12:0] iw={1'b0,r}-{1'b0,l}+13'd1,ih={1'b0,bt}-{1'b0,t}+13'd1;
    wire overlap=l<=r && t<=bt &&
        {iw,1'b0}>=w && {iw,1'b0}>=pw && {ih,1'b0}>=h && {ih,1'b0}>=ph;
    wire [12:0] ax={1'b0,a[47:36]}+a[23:12],ay={1'b0,a[35:24]}+a[11:0];
    wire [12:0] bx={1'b0,b[47:36]}+b[23:12],by={1'b0,b[35:24]}+b[11:0];
    wire [14:0] distance={2'b0,(ax>bx?ax-bx:bx-ax)}+{2'b0,(ay>by?ay-by:by-ay)};
    wire pair_matches=a[50:48]!=0 && b[50:48]!=0 &&
        (a[50:48]==b[50:48] ||
         (old_locked && (a[50:48]==3'd4 || b[50:48]==3'd4))) &&
        current_grade!=0 && old_streak!=0 && overlap &&
        (!PERSISTENT_ENABLE || old_lost<STRONG_RETAIN_FRAMES ||
         (a[50:48]==old_stable[50:48] &&
          (a[50:48]!=4 || a_digit==old_stable_digit) && distance<=15'd16));
    // Deadband is bounded to one 4x4 sampling cell. Larger motion uses the
    // current coordinates immediately, so a moving target does not trail.
    function [50:0] stabilize;
        input [50:0] fresh,old;
        reg [11:0] n,p;
        integer c;
        begin
            stabilize=fresh;
            for(c=0;c<4;c=c+1) begin
                n=fresh[c*12 +: 12];p=old[c*12 +: 12];
                if((n>p?n-p:p-n)<=4) stabilize[c*12 +: 12]=p;
            end
        end
    endfunction
    function [2:0] evidence_count;
        input [3:0] bits;
        begin evidence_count={2'b0,bits[0]}+{2'b0,bits[1]}+{2'b0,bits[2]}+{2'b0,bits[3]};end
    endfunction
    function show_target;
        input [50:0] box;
        input [1:0] grade;
        input [2:0] observations;
        input was_locked;
        input [3:0] evidence;
        begin
            if(box[50:48]==3) show_target=(grade>=2 && evidence_count(evidence)>=3) ||
                (grade!=0 && was_locked && evidence_count(evidence)>=2);
            else show_target=was_locked || observations>=(grade==3?3:grade==2?4:5);
        end
    endfunction
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=WAIT;ready<=0;confirmed<=0;confirmed_digits<=0;confirmed_qualities<=0;
            previous<=0;current<=0;streak<=0;pending<=0;grades<=0;
            previous_digits<=0;current_digits<=0;stable_digits<=0;previous_stable_digits<=0;
            digit_votes<=0;next_digit_votes<=0;
            kind_votes<=0;next_kind_votes<=0;
            previous_boxes<=0;stable_boxes<=0;locked<=0;matched_locked<=0;
            triangle_history<=0;next_history<=0;degraded<=0;next_degraded<=0;
            pending_locked<=0;blocked_previous<=0;
            used_current<=0;used_previous<=0;i<=0;j<=0;best_i<=0;best_j<=0;found<=0;best_distance<=0;
            track_trail_reset<=0;track_ids<=0;track_ages<=0;track_lost<=0;track_vx<=0;track_vy<=0;track_banks<=0;
            previous_ids<=0;previous_ages<=0;previous_lost<=0;previous_banks<=0;
            reserved_banks<=0;next_track_id<=0;pending_next_id<=0;
        end else begin
            if(publish && ready) begin
                previous<=current;streak<=pending;previous_boxes<=stable_boxes;
                previous_digits<=current_digits;previous_stable_digits<=stable_digits;
                digit_votes<=next_digit_votes;
                kind_votes<=next_kind_votes;
                locked<=pending_locked;triangle_history<=next_history;degraded<=next_degraded;
                previous_ids<=track_ids;previous_ages<=track_ages;
                previous_lost<=track_lost;previous_banks<=track_banks;next_track_id<=pending_next_id;
            end
            if(clear) begin state<=WAIT;ready<=0;confirmed<=0;confirmed_digits<=0;confirmed_qualities<=0;end
            else case(state)
                WAIT:if(raw_valid) begin
                    current<=raw_results;grades<=qualities;used_current<=0;used_previous<=0;
                    current_digits<=raw_digits;
                    blocked_previous<=0;pending_locked<=0;
                    stable_boxes<=raw_results;stable_digits<=raw_digits;
                    next_digit_votes<=0;next_kind_votes<=0;matched_locked<=0;
                    track_trail_reset<=0;track_ids<=0;track_ages<=0;track_lost<=0;track_vx<=0;track_vy<=0;track_banks<=0;
                    reserved_banks<=0;pending_next_id<=next_track_id;
                    for(k=0;k<TARGETS;k=k+1) begin
                        pending[k*3 +: 3]<=raw_results[k*51+48 +: 3]!=0 && qualities[k*2 +: 2]!=0?3'd1:3'd0;
                        next_history[k*4 +: 4]<=qualities[k*2 +: 2]>=2?4'd1:4'd0;
                        next_degraded[k]<=qualities[k*2 +: 2]<2;
                    end
                    i<=0;j<=0;found<=0;state<=SEARCH;
                end
                SEARCH:begin
                    if(a[50:48]!=0 && b[50:48]==3 && overlap) blocked_previous[j]<=1;
                    if(!used_current[i] && !used_previous[j] && pair_matches && (!found || distance<best_distance)) begin
                        found<=1;best_distance<=distance;best_i<=i;best_j<=j;
                    end
                    if(j==TARGETS-1) begin
                        j<=0;if(i==TARGETS-1) state<=CHOOSE;else i<=i+1'b1;
                    end else j<=j+1'b1;
                end
                CHOOSE:if(found) begin
                    used_current[best_i]<=1;used_previous[best_j]<=1;
                    if(PERSISTENT_ENABLE) reserved_banks[old_bank]<=1;
                    for(k=0;k<TARGETS;k=k+1) if(best_i==k) begin
                        track_trail_reset[k]<=old_lost>=STRONG_RETAIN_FRAMES;
                        track_ids[k*8 +: 8]<=old_id;
                        track_ages[k*8 +: 8]<=old_age==255?8'd255:old_age+8'd1;
                        track_banks[k*IB +: IB]<=old_bank;
                        // Raw observed centers avoid stable-box deadband bias.
                        // Across a gap this is displacement since last sighting.
                        track_vx[k*13 +: 13]<=$signed({1'b0,ax[12:1]})-$signed({1'b0,bx[12:1]});
                        track_vy[k*13 +: 13]<=$signed({1'b0,ay[12:1]})-$signed({1'b0,by[12:1]});
                        pending[k*3 +: 3]<=old_streak>=5?3'd5:old_streak+3'd1;
                        matched_locked[k]<=old_locked && (a[50:48]!=3 || !old_degraded || current_grade>=2);
                        next_history[k*4 +: 4]<={old_history[2:0],current_grade>=2};
                        stable_boxes[k*51 +: 51]<=stabilize(a,old_stable);
                        if(a[50:48]==old_stable[50:48]) next_kind_votes[k*2 +: 2]<=0;
                        else if(a[50:48]==b[50:48] && old_kind_vote!=0)
                            next_kind_votes[k*2 +: 2]<=0;
                        else begin
                            stable_boxes[k*51+48 +: 3]<=old_stable[50:48];
                            next_kind_votes[k*2 +: 2]<=1;
                        end
                        if(a[50:48]!=3'd4 && old_stable[50:48]==3'd4 &&
                           !(a[50:48]==b[50:48] && old_kind_vote!=0)) begin
                            stable_digits[k*4 +: 4]<=old_stable_digit;
                            next_digit_votes[k*2 +: 2]<=0;
                        end else if(a[50:48]==3'd4) begin
                            if(a_digit==old_stable_digit) begin
                                stable_digits[k*4 +: 4]<=old_stable_digit;
                                next_digit_votes[k*2 +: 2]<=0;
                            end else if(a_digit==b_digit && old_digit_vote!=0) begin
                                stable_digits[k*4 +: 4]<=a_digit;
                                next_digit_votes[k*2 +: 2]<=0;
                            end else begin
                                stable_digits[k*4 +: 4]<=old_stable_digit;
                                next_digit_votes[k*2 +: 2]<=1;
                            end
                        end
                    end
                    i<=0;j<=0;found<=0;state<=SEARCH;
                end else state<=OUTPUT;
                OUTPUT:begin
                    for(k=0;k<TARGETS;k=k+1) begin
                        pending_locked[k]<=show_target(stable_boxes[k*51 +: 51],grades[k*2 +: 2],
                            pending[k*3 +: 3],matched_locked[k],next_history[k*4 +: 4]);
                        confirmed[k*51 +: 51]<=show_target(stable_boxes[k*51 +: 51],grades[k*2 +: 2],
                            pending[k*3 +: 3],matched_locked[k],next_history[k*4 +: 4])?stable_boxes[k*51 +: 51]:51'd0;
                        confirmed_digits[k*4 +: 4]<=show_target(stable_boxes[k*51 +: 51],grades[k*2 +: 2],
                            pending[k*3 +: 3],matched_locked[k],next_history[k*4 +: 4]) &&
                            stable_boxes[k*51+48 +: 3]==3'd4 ? stable_digits[k*4 +: 4] : 4'd0;
                        confirmed_qualities[k*2 +: 2]<=
                            show_target(stable_boxes[k*51 +: 51],grades[k*2 +: 2],
                                pending[k*3 +: 3],matched_locked[k],next_history[k*4 +: 4]) ?
                            grades[k*2 +: 2] : 2'd0;
                    end
                    i<=0;j<=0;state<=RETAIN;
                end
                RETAIN:begin
                    if(!used_previous[i] && !blocked_previous[i] && history_kind==0 &&
                       (PERSISTENT_ENABLE ? b[50:48]!=0 && old_streak!=0 && old_lost<LOST_TIMEOUT_FRAMES-1 :
                        b[50:48]==3 && !old_degraded && old_history[2:0]!=0)) begin
                        if(PERSISTENT_ENABLE) reserved_banks[old_bank]<=1;
                        for(k=0;k<TARGETS;k=k+1) if(j==k) begin
                            current[k*51 +: 51]<=b;
                            current_digits[k*4 +: 4]<=b_digit;
                            stable_boxes[k*51 +: 51]<=old_stable;
                            stable_digits[k*4 +: 4]<=old_stable_digit;
                            pending[k*3 +: 3]<=old_streak;
                            track_ids[k*8 +: 8]<=old_id;track_ages[k*8 +: 8]<=old_age;
                            track_lost[k*8 +: 8]<=old_lost+8'd1;
                            track_banks[k*IB +: IB]<=old_bank;
                            next_digit_votes[k*2 +: 2]<=old_digit_vote;
                            next_kind_votes[k*2 +: 2]<=old_kind_vote;
                            next_history[k*4 +: 4]<={old_history[2:0],1'b0};next_degraded[k]<=1;
                            pending_locked[k]<=b[50:48]==3 ?
                                old_locked && evidence_count({old_history[2:0],1'b0})>=2:old_locked;
                        end
                        j<=0;
                        if(i==TARGETS-1) begin
                            j<=0;
                            if(PERSISTENT_ENABLE) begin i<=0;state<=ALLOCATE;end else begin ready<=1;state<=DONE;end
                        end else i<=i+1'b1;
                    end else if(j==TARGETS-1) begin
                        j<=0;if(i==TARGETS-1) begin
                            if(PERSISTENT_ENABLE) begin i<=0;state<=ALLOCATE;end else begin ready<=1;state<=DONE;end
                        end else i<=i+1'b1;
                    end else j<=j+1'b1;
                end
                ALLOCATE:begin
                    // Only actual unmatched observations get new identities.
                    // Retained tracks have age!=0. One small bank scan is shared.
                    if(pending[i*3 +: 3]!=0 && track_ages[i*8 +: 8]==0 && reserved_banks[j]) j<=j+1'b1;
                    else begin
                        for(k=0;k<TARGETS;k=k+1) if(i==k && pending[k*3 +: 3]!=0 && track_ages[k*8 +: 8]==0) begin
                            track_trail_reset[k]<=1;
                            track_ids[k*8 +: 8]<=pending_next_id;track_ages[k*8 +: 8]<=1;
                            track_banks[k*IB +: IB]<=j;reserved_banks[j]<=1;
                            pending_next_id<=pending_next_id+1'b1;
                        end
                        j<=0;
                        if(i==TARGETS-1) begin ready<=1;state<=DONE;end else i<=i+1'b1;
                    end
                end
                DONE:state<=DONE;
                default:state<=WAIT;
            endcase
        end
    end
// synthesis translate_off
    initial if(PROFILE_FPS<1 || STRONG_RETAIN_FRAMES<1 ||
        LOST_TIMEOUT_FRAMES>255 || LOST_TIMEOUT_FRAMES<=STRONG_RETAIN_FRAMES)
        $fatal(1,"Invalid tracking retention profile");
    // synthesis translate_on
endmodule

// Streaming two-target selector. Geometry gates run upstream; quality is an
// ordinal fit grade (1/2/3), NOT a calibrated probability. Equal grades prefer
// larger targets, then raster order. No frame storage or extra DDR traffic.
module Shape_Target_Select (
    input wire clk,rst_n,clear,accept,
    input wire [49:0] candidate,
    input wire [1:0] quality,
    input wire [31:0] area,
    output reg [49:0] first,second,
    output reg [1:0] first_quality,second_quality
);
    reg [31:0] area0,area1;
    function contains;
        input [49:0] a,b;
        begin contains=a[49:48]!=0 && b[49:48]!=0 &&
            a[47:36]<=b[47:36] && a[35:24]<=b[35:24] &&
            a[23:12]>=b[23:12] && a[11:0]>=b[11:0];end
    endfunction
    function better;
        input [49:0] a,b;
        input [1:0] qa,qb;
        input [31:0] aa,ab;
        begin better=b[49:48]==0 || qa>qb || (qa==qb && (aa>ab ||
            (aa==ab && {a[35:24],a[47:36]}<{b[35:24],b[47:36]})));end
    endfunction
    wire in0=contains(first,candidate),out0=contains(candidate,first);
    wire in1=contains(second,candidate),out1=contains(candidate,second);
    // Preserve the established paper-border heuristic: a much larger enclosing
    // contour is suppressed, even across classes. Similar-size nested contours
    // of the SAME class merge to the outer box (hollow-stroke deduplication).
    wire small0={1'b0,area0}>({1'b0,area}<<1);
    wire small1={1'b0,area1}>({1'b0,area}<<1);
    wire large0={1'b0,area}>({1'b0,area0}<<1);
    wire large1={1'b0,area}>({1'b0,area1}<<1);
    wire dup0=(in0 || out0) && !small0 && !large0 && first[49:48]==candidate[49:48];
    wire dup1=(in1 || out1) && !small1 && !large1 && second[49:48]==candidate[49:48];
    wire drop_candidate=(out0 && large0) || (out1 && large1) ||
        (dup0 && (area<area0 || (area==area0 && quality<=first_quality))) ||
        (dup1 && (area<area1 || (area==area1 && quality<=second_quality)));
    wire remove0=(in0 && small0) || dup0;
    wire remove1=(in1 && small1) || dup1;
    reg [49:0] next0,next1,swap_box;
    reg [1:0] q0,q1,swap_quality;
    reg [31:0] a0,a1,swap_area;
    always @(*) begin
        next0=first;next1=second;q0=first_quality;q1=second_quality;a0=area0;a1=area1;
        swap_box=0;swap_quality=0;swap_area=0;
        if(!drop_candidate) begin
            if(remove0) begin next0=0;q0=0;a0=0;end
            if(remove1) begin next1=0;q1=0;a1=0;end
            if(next0[49:48]==0) begin next0=candidate;q0=quality;a0=area;end
            else if(next1[49:48]==0 || better(candidate,next1,quality,q1,area,a1)) begin
                next1=candidate;q1=quality;a1=area;
            end
            if(next1[49:48]!=0 && better(next1,next0,q1,q0,a1,a0)) begin
                swap_box=next0;swap_quality=q0;swap_area=a0;
                next0=next1;q0=q1;a0=a1;
                next1=swap_box;q1=swap_quality;a1=swap_area;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin first<=0;second<=0;first_quality<=0;second_quality<=0;area0<=0;area1<=0;end
        else if(clear) begin first<=0;second<=0;first_quality<=0;second_quality<=0;area0<=0;area1<=0;end
        else if(accept && candidate[49:48]!=0 && quality!=0) begin
            first<=next0;second<=next1;first_quality<=q0;second_quality<=q1;area0<=a0;area1<=a1;
        end
    end
endmodule

// Associate by geometry, not by rank/slot: ranking can swap between frames.
// Exhaust both 2x2 assignments; maximize matches, then minimize center distance.
// One old target cannot confirm two new ones. Medium/high grades need two
// successful observations, low grades three. No stale-box hold or prediction.
module Shape_Target_Confirm (
    input wire clk,rst_n,publish,
    input wire [49:0] raw_first,raw_second,
    input wire [1:0] quality_first,quality_second,
    output wire [49:0] confirmed_first,confirmed_second
);
    reg [49:0] previous0,previous1;
    reg [1:0] streak0,streak1;
    function box_matches;
        input [49:0] a,b;
        reg [12:0] w,h,pw,ph,iw,ih;
        reg [11:0] l,t,r,bt;
        begin
            w={1'b0,a[23:12]}-{1'b0,a[47:36]}+13'd1;
            h={1'b0,a[11:0]}-{1'b0,a[35:24]}+13'd1;
            pw={1'b0,b[23:12]}-{1'b0,b[47:36]}+13'd1;
            ph={1'b0,b[11:0]}-{1'b0,b[35:24]}+13'd1;
            l=a[47:36]>b[47:36]?a[47:36]:b[47:36];r=a[23:12]<b[23:12]?a[23:12]:b[23:12];
            t=a[35:24]>b[35:24]?a[35:24]:b[35:24];bt=a[11:0]<b[11:0]?a[11:0]:b[11:0];
            iw={1'b0,r}-{1'b0,l}+13'd1;ih={1'b0,bt}-{1'b0,t}+13'd1;
            box_matches=a[49:48]!=0 && a[49:48]==b[49:48] && l<=r && t<=bt &&
                {iw,1'b0}>=w && {iw,1'b0}>=pw && {ih,1'b0}>=h && {ih,1'b0}>=ph;
        end
    endfunction
    function [14:0] distance;
        input [49:0] a,b;
        reg [12:0] ax,ay,bx,by;
        begin
            ax={1'b0,a[47:36]}+a[23:12];ay={1'b0,a[35:24]}+a[11:0];
            bx={1'b0,b[47:36]}+b[23:12];by={1'b0,b[35:24]}+b[11:0];
            distance={2'b0,(ax>bx?ax-bx:bx-ax)}+{2'b0,(ay>by?ay-by:by-ay)};
        end
    endfunction
    wire m00=box_matches(raw_first,previous0),m01=box_matches(raw_first,previous1);
    wire m10=box_matches(raw_second,previous0),m11=box_matches(raw_second,previous1);
    wire [1:0] direct_count={1'b0,m00}+{1'b0,m11},cross_count={1'b0,m01}+{1'b0,m10};
    wire [15:0] direct_cost=(m00?{1'b0,distance(raw_first,previous0)}:16'd0)+
        (m11?{1'b0,distance(raw_second,previous1)}:16'd0);
    wire [15:0] cross_cost=(m01?{1'b0,distance(raw_first,previous1)}:16'd0)+
        (m10?{1'b0,distance(raw_second,previous0)}:16'd0);
    wire cross_match=cross_count>direct_count || (cross_count==direct_count && cross_cost<direct_cost);
    wire matched0=cross_match?m01:m00,matched1=cross_match?m10:m11;
    wire [1:0] old0=cross_match?streak1:streak0,old1=cross_match?streak0:streak1;
    wire [1:0] next0=raw_first[49:48]==0 || quality_first==0?0:!matched0?1:old0>=2?3:old0+2'd1;
    wire [1:0] next1=raw_second[49:48]==0 || quality_second==0?0:!matched1?1:old1>=2?3:old1+2'd1;
    assign confirmed_first=next0>=(quality_first==1?3:2)?raw_first:50'd0;
    assign confirmed_second=next1>=(quality_second==1?3:2)?raw_second:50'd0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin previous0<=0;previous1<=0;streak0<=0;streak1<=0;end
        else if(publish) begin previous0<=raw_first;previous1<=raw_second;streak0<=next0;streak1<=next1;end
    end
endmodule
