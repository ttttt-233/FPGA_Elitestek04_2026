`timescale 1ns/1ps

// Exact 8-connected hysteresis using frame labels and union-by-rank.
// One label row is local; optional local storage holds one resolved strong bit
// per component. Candidate frames, pixel labels and parent tables stay external.
//
// Port contract: byte addresses, aligned 32-bit words, one outstanding request.
// Hold request until mem_ready. EVERY accepted read/write gets one response;
// a response may coincide with request acceptance. Write responses mean visible
// completion. BLOCK_MEMORY adds aligned 128-bit transfers for consecutive data;
// mem_block selects that payload while parent-table accesses remain 32-bit.
// Candidate words pack 16 pixels, earliest pixel in bits [1:0]: 0/1/2, never 3.
// All output waits for completed labeling AND equivalence resolution. Backpressure
// is allowed indefinitely. A frame with error must not be published downstream.
// Canny_Frame_Engine supplies the board's capture/cache/DDR integration.
module Canny_Hysteresis_CCL #(
    parameter integer WIDTH=1280,
    parameter integer HEIGHT=720,
    parameter integer PIXELS=WIDTH*HEIGHT,
    parameter integer LABEL_BITS=$clog2(PIXELS+1),
    parameter integer MAX_LABELS=PIXELS,
    // Board capture stores one {RGB565, median8, reserved6, class2} word/pixel.
    // Keep the compact 16-class/word interface as the default for core users.
    parameter integer RECORD_INPUT=0,
    parameter integer BLOCK_MEMORY=0,
    parameter integer LOCAL_FLAGS=0,
    // Consume successful block reads at the response edge. The board enables
    // this; 0 retains the original registered-response schedule for comparison.
    parameter integer FAST_BLOCK_READ=0,
    // Skip scheduler-only bubbles BETWEEN complete blocks. Memory ordering,
    // row updates and error checks remain unchanged; legacy default is off.
    parameter integer FAST_BLOCK_NEXT=0,
    // Frame pipeline: keep this frame's resolved flags until the OUT token is
    // granted. The legacy engine leaves this disabled and needs no new input.
    parameter integer DEFER_OUTPUT=0,
    // Optional bounded on-chip component area. No new scratch transactions or
    // FSM states: reads overlap FIND and writes use existing pixel/union edges.
    // Components containing any seed beyond AREA_TRACK_LABELS fail open (keep).
    // Zero removes the feature at elaboration and preserves legacy behaviour.
    parameter integer MIN_COMPONENT_PIXELS=0,
    parameter integer AREA_TRACK_LABELS=4096,
    parameter integer EDGE_DIAGNOSTIC_ENABLE=0,
    parameter [31:0] CANDIDATE_BASE=32'h02000000,
    parameter [31:0] LABEL_BASE=32'h02400000,
    parameter [31:0] TABLE_BASE=32'h02800000
)(
    input wire clk,rst_n,start,
    input wire output_start,
    output wire output_pending,
    output wire busy,
    output reg done,error,
    output reg [2:0] error_code, // 1=bad class,2=capacity,3=memory,4=corrupt table
    output wire mem_valid,
    input wire mem_ready,
    output reg mem_write,
    output reg [31:0] mem_addr,mem_wdata,
    output reg mem_block,
    output reg [127:0] mem_block_wdata,
    input wire mem_rsp_valid,mem_rsp_error,
    input wire [31:0] mem_rdata,
    input wire [127:0] mem_block_rdata,
    output wire output_valid,
    input wire output_ready,
    output wire output_edge,
    output wire output_first,output_last,
    output reg [11:0] output_x,output_y,
    output reg [31:0] label_count,union_count,candidate_count,strong_count,cycle_count,
    // Low to high: candidates, provisional labels, unions, removed components,
    // removed pixels, bypassed strong components, excess seeds, output pixels.
    output wire [255:0] edge_counts,
    output reg [1:0] profile_phase // 1=scan,2=roots,3=output; held after done
);
    localparam integer STRONG_BIT=LABEL_BITS,RANK_LOW=LABEL_BITS+1;
    localparam [5:0] IDLE=0,PIXEL=1,CLASSIFY=2,READ_NE=3,NEXT_NEIGHBOR=4,
        NEIGHBOR_FOUND=5,UNION_CHILD_DONE=6,UNION_ROOT_DONE=7,WRITE_LABEL=8,
        LABEL_DONE=9,FLATTEN=10,FLAT_FOUND=11,FLAT_DONE=12,OUTPUT_START=13,
        OUTPUT_LABEL=14,OUTPUT_FLAG=15,EMIT=16,FIND=17,FIND_RESPONSE=18,
        MEMORY_REQUEST=19,MEMORY_RESPONSE=20,CANDIDATE_WORD=21,
        BLOCK_WRITTEN=22,OUTPUT_LANE=23,WAIT_OUTPUT=24;
    reg [5:0] state,return_state,find_return_state;
    reg [31:0] response_word,candidate_word,pixel_index,output_index,flat_index;
    reg [127:0] response_block,candidate_block,label_block,output_block;
    reg edge_reg;
    reg [11:0] x,y;
    reg [LABEL_BITS-1:0] west,northwest,north_saved,pixel_label;
    reg [LABEL_BITS-1:0] neighbor[0:3];
    reg [2:0] neighbor_index;
    reg pixel_strong;
    reg [LABEL_BITS-1:0] component_root,find_label,found_root,union_root;
    reg [31:0] component_entry,found_entry,union_entry;
    reg [31:0] pixel_entry,west_entry;
    reg [5:0] find_depth;
    wire [1:0] candidate_class;
    wire [4:0] component_rank=component_entry[RANK_LOW +: 5];
    wire [4:0] found_rank=found_entry[RANK_LOW +: 5];
    wire [LABEL_BITS-1:0] next_label=label_count[LABEL_BITS-1:0]+1'b1;
    reg neighbor_available;
    localparam integer AREA_BITS=MIN_COMPONENT_PIXELS>1?$clog2(MIN_COMPONENT_PIXELS+1):1;
    localparam integer AREA_CAPACITY=AREA_TRACK_LABELS<MAX_LABELS?AREA_TRACK_LABELS:MAX_LABELS;
    localparam integer AREA_ADDR_BITS=AREA_CAPACITY>1?$clog2(AREA_CAPACITY):1;
    localparam [AREA_BITS-1:0] AREA_SAT=MIN_COMPONENT_PIXELS;
    reg [AREA_BITS-1:0] component_area,found_area,pixel_area,west_area;
    wire [AREA_BITS-1:0] area_q;
    reg component_bypass,found_bypass,pixel_bypass,west_bypass;
    wire area_bypass_q;
    reg [31:0] removed_components,removed_pixels,bypassed_components,kept_pixels;
    wire [31:0] excess_seeds=(MIN_COMPONENT_PIXELS>0 && label_count>AREA_CAPACITY)?
        label_count-AREA_CAPACITY:32'd0;
    assign edge_counts=EDGE_DIAGNOSTIC_ENABLE?
        {kept_pixels,excess_seeds,bypassed_components,removed_pixels,removed_components,
         union_count,label_count,candidate_count}:256'd0;
    function [AREA_BITS-1:0] add_area;
        input [AREA_BITS-1:0] a,b;
        reg [AREA_BITS:0] total;
        begin
            total={1'b0,a}+{1'b0,b};
            add_area=total>=MIN_COMPONENT_PIXELS?AREA_SAT:total[AREA_BITS-1:0];
        end
    endfunction
    wire choose_component_root=component_rank>found_rank ||
        (component_rank==found_rank && component_root<found_root);
    wire [AREA_BITS-1:0] merged_area=add_area(component_area,found_area);
    wire [AREA_BITS-1:0] incremented_area=add_area(component_area,{{(AREA_BITS-1){1'b0}},1'b1});
    wire final_keep=found_entry[STRONG_BIT] &&
        (MIN_COMPONENT_PIXELS==0 || found_area>=MIN_COMPONENT_PIXELS);

    generate if(MIN_COMPONENT_PIXELS>0) begin: g_area
        // Label k occupies address k-1. Every allocated tracked seed is
        // initialized before use, so reset/frame start needs no sweep.
        // Allocated components have positive area: store min(area,MIN)-1.
        // For MIN=8, codes 0..7 represent areas 1..8 and save one RAM bit/label.
        // Keep arithmetic/diagnostics in actual-area units; the bypass bit
        // remains independent. Unallocated entries are never consumed.
        localparam STORED_AREA_BITS=MIN_COMPONENT_PIXELS>1?$clog2(MIN_COMPONENT_PIXELS):1;
        localparam COUNT_BITS=STORED_AREA_BITS+(EDGE_DIAGNOSTIC_ENABLE?1:0);
        reg [COUNT_BITS-1:0] counts[0:AREA_CAPACITY-1];
        reg [COUNT_BITS-1:0] count_q;
        wire pixel_commit=state==NEXT_NEIGHBOR &&
            (BLOCK_MEMORY?!neighbor_available:neighbor_index>=4);
        wire merging=state==NEIGHBOR_FOUND && component_root!=0 && component_root!=found_root;
        wire [LABEL_BITS-1:0] merge_id=choose_component_root?component_root:found_root;
        wire [LABEL_BITS-1:0] write_id=merging?merge_id:
            component_root==0?next_label:component_root;
        wire [AREA_BITS-1:0] write_area=merging?merged_area:
            component_root==0?{{(AREA_BITS-1){1'b0}},1'b1}:incremented_area;
        wire write_bypass=merging?(component_bypass | found_bypass):
            component_root==0?1'b0:component_bypass;
        wire write_enable=(merging || (pixel_commit &&
            (component_root!=0 || label_count<MAX_LABELS))) && write_id<=AREA_CAPACITY;
        // Clamp absent/out-of-budget reads to a valid address; the consumer
        // explicitly substitutes AREA_SAT for an untracked root. Never wrap.
        wire [AREA_ADDR_BITS-1:0] read_id=find_label!=0 && find_label<=AREA_CAPACITY?find_label-1'b1:0;
        wire [AREA_ADDR_BITS-1:0] write_address=write_id-1'b1;
        wire [STORED_AREA_BITS-1:0] encoded_area=write_area-1'b1;
        always @(posedge clk) begin
            count_q<=counts[read_id];
            if(rst_n && write_enable) counts[write_address]<=EDGE_DIAGNOSTIC_ENABLE?
                {write_bypass,encoded_area}:encoded_area;
        end
        // Extend before adding: the saturated code must decode to MIN, not 0.
        assign area_q={{(AREA_BITS-STORED_AREA_BITS){1'b0}},count_q[STORED_AREA_BITS-1:0]}+1'b1;
        assign area_bypass_q=EDGE_DIAGNOSTIC_ENABLE?count_q[COUNT_BITS-1]:1'b0;
    end else begin: g_no_area
        assign area_q=0;
        assign area_bypass_q=0;
    end endgenerate

    // Count each final root once, before FLAT_FOUND rewrites its strong flag.
    // A removed root is below saturation, so its pixel count is exact.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            removed_components<=0;removed_pixels<=0;bypassed_components<=0;kept_pixels<=0;
        end else if(start && state==IDLE) begin
            removed_components<=0;removed_pixels<=0;bypassed_components<=0;kept_pixels<=0;
        end else if(EDGE_DIAGNOSTIC_ENABLE) begin
            if(state==FLAT_FOUND && flat_index==found_root && found_entry[STRONG_BIT]) begin
                if(!final_keep) begin
                    removed_components<=removed_components+1'b1;
                    removed_pixels<=removed_pixels+found_area;
                end
                if(MIN_COMPONENT_PIXELS>0 && found_bypass)
                    bypassed_components<=bypassed_components+1'b1;
            end
            if(output_valid && output_ready && output_edge) kept_pixels<=kept_pixels+1'b1;
        end
    end

    // Read the previous-row N at CLASSIFY and NE one cycle later. The previous
    // N is saved as NW before overwriting the same row-memory position.
    reg [LABEL_BITS-1:0] row_memory[0:WIDTH-1];
    reg [LABEL_BITS-1:0] row_q;
    wire row_commit=state==LABEL_DONE ||
        (BLOCK_MEMORY && state==CLASSIFY && candidate_class==0);
    wire [11:0] row_read_addr=(state==CLASSIFY || (BLOCK_MEMORY && state==LABEL_DONE))?
                               (x<WIDTH-1?x+12'd1:12'd0):x;
    always @(posedge clk) begin
        row_q<=row_memory[row_read_addr];
        if(rst_n && row_commit) row_memory[x]<=state==CLASSIFY?{LABEL_BITS{1'b0}}:pixel_label;
    end

    // Final strong flags need only one bit per provisional component. All
    // allocated entries are rewritten by flattening before any output. The
    // board bounds MAX_LABELS by ceil(W/2)*ceil(H/2), the maximum number of
    // mutually non-adjacent 8-connected component seeds, never by scene content.
    reg resolved_strong[0:(LOCAL_FLAGS?MAX_LABELS:0)];
    reg resolved_q;
    wire [31:0] output_label=output_block[output_index[1:0]*32 +: 32];
    wire [31:0] next_output_label=output_block[(output_index[1:0]+2'd1)*32 +: 32];
    wire block_output_response=FAST_BLOCK_READ && BLOCK_MEMORY && LOCAL_FLAGS &&
        mem_block && !mem_write && return_state==OUTPUT_LABEL &&
        (state==MEMORY_RESPONSE || (state==MEMORY_REQUEST && mem_ready)) &&
        mem_rsp_valid && !mem_rsp_error;
    // The response beat supplies lane zero to the synchronous flag RAM on
    // the SAME edge that latches output_block. No asynchronous RAM is needed.
    // Invalid labels are rejected before EMIT; never index flags with garbage.
    wire [LABEL_BITS-1:0] flag_read_label=block_output_response?
        (mem_block_rdata[31:0]<=label_count?mem_block_rdata[LABEL_BITS-1:0]:{LABEL_BITS{1'b0}}):
        (state==EMIT && output_ready && output_index[1:0]!=3)?
        next_output_label[LABEL_BITS-1:0]:output_label[LABEL_BITS-1:0];
    always @(posedge clk) if(LOCAL_FLAGS) begin
        resolved_q<=resolved_strong[flag_read_label];
        if(rst_n && state==FLAT_FOUND)
            resolved_strong[flat_index[LABEL_BITS-1:0]]<=final_keep;
    end
    assign output_edge=LOCAL_FLAGS?(output_label!=0 && resolved_q):edge_reg;

    assign busy=state!=IDLE;
    assign output_pending=state==WAIT_OUTPUT;
    assign mem_valid=state==MEMORY_REQUEST;
    assign output_valid=state==EMIT;
    assign output_first=output_valid && output_index==0;
    assign output_last=output_valid && output_index==PIXELS-1;
    assign candidate_class=BLOCK_MEMORY?
        (RECORD_INPUT?candidate_block[pixel_index[1:0]*32 +: 2]:
                      candidate_block[pixel_index[5:0]*2 +: 2]):
        (RECORD_INPUT?candidate_word[1:0]:candidate_word[pixel_index[3:0]*2 +: 2]);
    wire [LABEL_BITS-1:0] response_parent=response_word[LABEL_BITS-1:0];
    wire merged_strong=component_entry[STRONG_BIT] | found_entry[STRONG_BIT];
    reg [2:0] selected_neighbor;
    integer ni;
    always @(*) begin
        neighbor_available=0;selected_neighbor=0;
        for(ni=3;ni>=0;ni=ni-1) begin
            if(ni>=neighbor_index && neighbor[ni]!=0 &&
               (ni<1 || neighbor[ni]!=neighbor[0]) &&
               (ni<2 || neighbor[ni]!=neighbor[1]) &&
               (ni<3 || neighbor[ni]!=neighbor[2])) begin
                neighbor_available=1;selected_neighbor=ni;
            end
        end
    end

    function [31:0] entry;
        input [LABEL_BITS-1:0] parent_id;
        input has_strong;
        input [4:0] rank_value;
        begin
            entry=0;
            entry[LABEL_BITS-1:0]=parent_id;
            entry[STRONG_BIT]=has_strong;
            entry[RANK_LOW +: 5]=rank_value;
        end
    endfunction
    task transfer;
        input write_enable;
        input [31:0] address,data;
        input [5:0] resume_state;
        begin
            mem_write<=write_enable;mem_addr<=address;mem_wdata<=data;
            mem_block<=0;mem_block_wdata<=0;
            return_state<=resume_state;state<=MEMORY_REQUEST;
        end
    endtask
    task block_transfer;
        input write_enable;
        input [31:0] address;
        input [127:0] data;
        input [5:0] resume_state;
        begin
            mem_write<=write_enable;mem_addr<=address;mem_wdata<=0;
            mem_block<=1;mem_block_wdata<=data;
            return_state<=resume_state;state<=MEMORY_REQUEST;
        end
    endtask
    // Commit a label locally; only every fourth pixel needs an external write.
    // Zero candidates advance directly from CLASSIFY, one cycle per pixel
    // between block boundaries. Synchronous row reads prefetch the next N.
    task finish_block_pixel;
        input [LABEL_BITS-1:0] label_id;
        reg [127:0] packed_labels;
        begin
            packed_labels=label_block;
            packed_labels[pixel_index[1:0]*32 +: 32]={{(32-LABEL_BITS){1'b0}},label_id};
            label_block<=packed_labels;
            west<=label_id;
            west_entry<=state==CLASSIFY?32'd0:pixel_entry;
            west_area<=state==CLASSIFY?{AREA_BITS{1'b0}}:pixel_area;
            west_bypass<=state==CLASSIFY?1'b0:pixel_bypass;
            northwest<=state==CLASSIFY?((y==0)?{LABEL_BITS{1'b0}}:row_q):north_saved;
            if(x==WIDTH-1) begin x<=0;y<=y+1'b1;west<=0;northwest<=0;end
            else x<=x+1'b1;
            pixel_index<=pixel_index+1'b1;
            if(pixel_index[1:0]==3 || pixel_index==PIXELS-1) begin
                block_transfer(1,LABEL_BASE+((pixel_index>>2)<<4),packed_labels,BLOCK_WRITTEN);
                label_block<=0;
            end else if(WIDTH==1) state<=PIXEL;
            else state<=CLASSIFY;
        end
    endtask
    task find_root;
        input [LABEL_BITS-1:0] label_id;
        input [5:0] resume_state;
        begin
            find_label<=label_id;find_depth<=0;find_return_state<=resume_state;state<=FIND;
        end
    endtask
    task fail;
        input [2:0] code;
        begin error<=1;error_code<=code;done<=1;state<=IDLE;end
    endtask
    // Both immediate and delayed memory responses use the same error-first
    // path. Only read-result staging is bypassed; request ordering and all
    // label writes/union operations retain their original semantics.
    task accept_response;
        begin
            if(mem_rsp_error) fail(3);
            else if(FAST_BLOCK_NEXT && BLOCK_MEMORY && mem_block && mem_write &&
                    return_state==BLOCK_WRITTEN) begin
                if(pixel_index==PIXELS) begin flat_index<=1;state<=FLATTEN;profile_phase<=2;end
                else if(RECORD_INPUT)
                    block_transfer(0,CANDIDATE_BASE+(pixel_index<<2),0,CANDIDATE_WORD);
                else state<=PIXEL;
            end
            else if(FAST_BLOCK_READ && BLOCK_MEMORY && mem_block && !mem_write &&
                    return_state==CANDIDATE_WORD) begin
                candidate_block<=mem_block_rdata;state<=CLASSIFY;
            end else if(FAST_BLOCK_READ && BLOCK_MEMORY && mem_block && !mem_write &&
                        return_state==OUTPUT_LABEL) begin
                output_block<=mem_block_rdata;
                if(LOCAL_FLAGS) begin
                    if(mem_block_rdata[31:0]>label_count) fail(4);
                    else state<=EMIT;
                end else state<=OUTPUT_LANE;
            end else begin
                response_word<=mem_rdata;response_block<=mem_block_rdata;state<=return_state;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;return_state<=IDLE;find_return_state<=IDLE;
            done<=0;error<=0;error_code<=0;profile_phase<=0;
            mem_write<=0;mem_addr<=0;mem_wdata<=0;
            mem_block<=0;mem_block_wdata<=0;
            response_block<=0;candidate_block<=0;label_block<=0;output_block<=0;
            response_word<=0;candidate_word<=0;pixel_index<=0;output_index<=0;flat_index<=0;
            x<=0;y<=0;west<=0;northwest<=0;north_saved<=0;pixel_label<=0;
            neighbor[0]<=0;neighbor[1]<=0;neighbor[2]<=0;neighbor[3]<=0;
            neighbor_index<=0;pixel_strong<=0;component_root<=0;component_entry<=0;
            pixel_entry<=0;west_entry<=0;
            component_area<=0;found_area<=0;pixel_area<=0;west_area<=0;
            component_bypass<=0;found_bypass<=0;pixel_bypass<=0;west_bypass<=0;
            find_label<=0;found_root<=0;found_entry<=0;find_depth<=0;
            union_root<=0;union_entry<=0;
            edge_reg<=0;output_x<=0;output_y<=0;
            label_count<=0;union_count<=0;candidate_count<=0;strong_count<=0;cycle_count<=0;
        end else begin
            done<=0;
            if(busy) cycle_count<=cycle_count+1'b1;
            case(state)
                IDLE: if(start) begin
                    profile_phase<=1;
                    error<=0;error_code<=0;pixel_index<=0;output_index<=0;x<=0;y<=0;
                    west<=0;northwest<=0;west_entry<=0;output_x<=0;output_y<=0;
                    west_area<=0;
                    west_bypass<=0;
                    label_count<=0;union_count<=0;candidate_count<=0;strong_count<=0;cycle_count<=0;
                    label_block<=0;
                    state<=PIXEL;
                end
                PIXEL: begin
                    if(BLOCK_MEMORY) begin
                        if(RECORD_INPUT && pixel_index[1:0]==0)
                            block_transfer(0,CANDIDATE_BASE+(pixel_index<<2),0,CANDIDATE_WORD);
                        else if(!RECORD_INPUT && pixel_index[5:0]==0)
                            block_transfer(0,CANDIDATE_BASE+((pixel_index>>6)<<4),0,CANDIDATE_WORD);
                        else state<=CLASSIFY;
                    end else if(RECORD_INPUT)
                        transfer(0,CANDIDATE_BASE+(pixel_index<<2),0,CANDIDATE_WORD);
                    else if(pixel_index[3:0]==0)
                        transfer(0,CANDIDATE_BASE+((pixel_index>>4)<<2),0,CANDIDATE_WORD);
                    else state<=CLASSIFY;
                end
                CANDIDATE_WORD: begin
                    candidate_word<=response_word;candidate_block<=response_block;state<=CLASSIFY;
                end
                CLASSIFY: begin
                    north_saved<=(y==0)?0:row_q;
                    if(candidate_class==3) fail(1);
                    else if(candidate_class==0) begin
                        pixel_label<=0;
                        if(BLOCK_MEMORY) finish_block_pixel(0);
                        else state<=WRITE_LABEL;
                    end
                    else begin
                        candidate_count<=candidate_count+1'b1;
                        if(candidate_class==2) strong_count<=strong_count+1'b1;
                        pixel_strong<=candidate_class==2;
                        neighbor[0]<=west;neighbor[1]<=northwest;
                        neighbor[2]<=(y==0)?0:row_q;
                        state<=READ_NE;
                    end
                end
                READ_NE: begin
                    neighbor[3]<=(y!=0 && x<WIDTH-1)?row_q:0;
                    component_root<=0;component_entry<=0;component_area<=0;neighbor_index<=0;state<=NEXT_NEIGHBOR;
                    component_bypass<=0;
                end
                NEXT_NEIGHBOR: begin
                    if(BLOCK_MEMORY && neighbor_available) begin
                        neighbor_index<=selected_neighbor;
                        // W is the immediately preceding pixel's final root.
                        // No union can intervene between that commit and here;
                        // its parent/strong/rank entry is therefore current.
                        if(selected_neighbor==0) begin
                            found_root<=west;found_entry<=west_entry;found_area<=west_area;state<=NEIGHBOR_FOUND;
                            found_bypass<=west_bypass;
                        end else find_root(neighbor[selected_neighbor],NEIGHBOR_FOUND);
                    end else if(!BLOCK_MEMORY && neighbor_index<4) begin
                        if(neighbor[neighbor_index]==0 ||
                           (neighbor_index>0 && neighbor[neighbor_index]==neighbor[0]) ||
                           (neighbor_index>1 && neighbor[neighbor_index]==neighbor[1]) ||
                           (neighbor_index>2 && neighbor[neighbor_index]==neighbor[2]))
                            neighbor_index<=neighbor_index+1'b1;
                        else find_root(neighbor[neighbor_index],NEIGHBOR_FOUND);
                    end else if(component_root==0) begin
                        if(label_count>=MAX_LABELS) fail(2);
                        else begin
                            label_count<=label_count+1'b1;
                            pixel_label<=next_label;
                            pixel_area<=next_label<=AREA_CAPACITY?{{(AREA_BITS-1){1'b0}},1'b1}:AREA_SAT;
                            pixel_bypass<=EDGE_DIAGNOSTIC_ENABLE && next_label>AREA_CAPACITY;
                            pixel_entry<=entry(next_label,pixel_strong,0);
                            transfer(1,TABLE_BASE+((label_count+1'b1)<<2),
                                     entry(next_label,pixel_strong,0),WRITE_LABEL);
                        end
                    end else begin
                        pixel_label<=component_root;
                        pixel_area<=incremented_area;
                        pixel_bypass<=component_bypass;
                        pixel_entry<=entry(component_root,pixel_strong | component_entry[STRONG_BIT],component_rank);
                        if(pixel_strong && !component_entry[STRONG_BIT])
                            transfer(1,TABLE_BASE+({{(32-LABEL_BITS){1'b0}},component_root}<<2),
                                     entry(component_root,1'b1,component_rank),WRITE_LABEL);
                        else state<=WRITE_LABEL;
                    end
                end
                NEIGHBOR_FOUND: begin
                    if(component_root==0 || component_root==found_root) begin
                        component_root<=found_root;component_entry<=found_entry;
                        component_area<=found_area;
                        component_bypass<=found_bypass;
                        neighbor_index<=neighbor_index+1'b1;state<=NEXT_NEIGHBOR;
                    end else begin
                        union_count<=union_count+1'b1;
                        component_area<=merged_area;
                        component_bypass<=component_bypass | found_bypass;
                        if(component_rank>found_rank ||
                           (component_rank==found_rank && component_root<found_root)) begin
                            union_root<=component_root;
                            union_entry<=entry(component_root,merged_strong,
                                component_rank+((component_rank==found_rank)?5'd1:5'd0));
                            component_entry<=entry(component_root,merged_strong,
                                component_rank+((component_rank==found_rank)?5'd1:5'd0));
                            transfer(1,TABLE_BASE+({{(32-LABEL_BITS){1'b0}},found_root}<<2),
                                     entry(component_root,found_entry[STRONG_BIT],found_rank),UNION_CHILD_DONE);
                        end else begin
                            union_root<=found_root;
                            union_entry<=entry(found_root,merged_strong,
                                found_rank+((component_rank==found_rank)?5'd1:5'd0));
                            component_root<=found_root;
                            component_entry<=entry(found_root,merged_strong,
                                found_rank+((component_rank==found_rank)?5'd1:5'd0));
                            transfer(1,TABLE_BASE+({{(32-LABEL_BITS){1'b0}},component_root}<<2),
                                     entry(found_root,component_entry[STRONG_BIT],component_rank),UNION_CHILD_DONE);
                        end
                    end
                end
                UNION_CHILD_DONE:
                    transfer(1,TABLE_BASE+({{(32-LABEL_BITS){1'b0}},union_root}<<2),union_entry,UNION_ROOT_DONE);
                UNION_ROOT_DONE: begin neighbor_index<=neighbor_index+1'b1;state<=NEXT_NEIGHBOR;end
                WRITE_LABEL:
                    if(BLOCK_MEMORY) state<=LABEL_DONE;
                    else transfer(1,LABEL_BASE+(pixel_index<<2),{{(32-LABEL_BITS){1'b0}},pixel_label},LABEL_DONE);
                LABEL_DONE: begin
                    if(BLOCK_MEMORY) finish_block_pixel(pixel_label);
                    else begin
                    west<=pixel_label;northwest<=north_saved;
                    if(x==WIDTH-1) begin x<=0;y<=y+1'b1;west<=0;northwest<=0;end
                    else x<=x+1'b1;
                    pixel_index<=pixel_index+1'b1;
                    if(pixel_index==PIXELS-1) begin flat_index<=1;state<=FLATTEN;profile_phase<=2;end
                    else state<=PIXEL;
                    end
                end
                BLOCK_WRITTEN: begin
                    if(pixel_index==PIXELS) begin flat_index<=1;state<=FLATTEN;profile_phase<=2;end
                    else state<=PIXEL;
                end
                FLATTEN: begin
                    if(flat_index>label_count) begin
                        profile_phase<=DEFER_OUTPUT?2'd0:2'd3;
                        output_index<=0;output_x<=0;output_y<=0;
                        state<=DEFER_OUTPUT?WAIT_OUTPUT:OUTPUT_START;
                    end else find_root(flat_index[LABEL_BITS-1:0],FLAT_FOUND);
                end
                FLAT_FOUND:
                    transfer(1,TABLE_BASE+(flat_index<<2),entry(found_root,final_keep,0),FLAT_DONE);
                FLAT_DONE: begin flat_index<=flat_index+1'b1;state<=FLATTEN;end
                WAIT_OUTPUT: if(output_start) begin profile_phase<=3;state<=OUTPUT_START;end
                OUTPUT_START:
                    if(BLOCK_MEMORY) block_transfer(0,LABEL_BASE+((output_index>>2)<<4),0,OUTPUT_LABEL);
                    else transfer(0,LABEL_BASE+(output_index<<2),0,OUTPUT_LABEL);
                OUTPUT_LABEL: begin
                    if(BLOCK_MEMORY) begin
                        output_block<=response_block;state<=OUTPUT_LANE;
                    end else if(response_word==0) begin edge_reg<=0;state<=EMIT;end
                    else if(response_word>label_count) fail(4);
                    else transfer(0,TABLE_BASE+(response_word<<2),0,OUTPUT_FLAG);
                end
                OUTPUT_LANE: begin
                    if(output_label>label_count) fail(4);
                    else if(LOCAL_FLAGS) state<=EMIT;
                    else if(output_label==0) begin edge_reg<=0;state<=EMIT;end
                    else transfer(0,TABLE_BASE+(output_label<<2),0,OUTPUT_FLAG);
                end
                OUTPUT_FLAG: begin edge_reg<=response_word[STRONG_BIT];state<=EMIT;end
                EMIT: if(output_ready) begin
                    if(LOCAL_FLAGS && output_label>label_count) fail(4);
                    else if(output_index==PIXELS-1) begin done<=1;state<=IDLE;end
                    else begin
                        output_index<=output_index+1'b1;
                        if(output_x==WIDTH-1) begin output_x<=0;output_y<=output_y+1'b1;end
                        else output_x<=output_x+1'b1;
                        if(BLOCK_MEMORY && output_index[1:0]!=3)
                            state<=LOCAL_FLAGS?EMIT:OUTPUT_LANE;
                        else if(FAST_BLOCK_NEXT && BLOCK_MEMORY)
                            block_transfer(0,LABEL_BASE+(((output_index+1'b1)>>2)<<4),0,OUTPUT_LABEL);
                        else state<=OUTPUT_START;
                    end
                end
                FIND: begin
                    if(find_label==0 || find_label>label_count || find_depth>LABEL_BITS) fail(4);
                    else transfer(0,TABLE_BASE+({{(32-LABEL_BITS){1'b0}},find_label}<<2),0,FIND_RESPONSE);
                end
                FIND_RESPONSE: begin
                    if(response_parent==0 || response_parent>label_count) fail(4);
                    else if(response_parent==find_label) begin
                        found_root<=find_label;found_entry<=response_word;
                        found_area<=find_label<=AREA_CAPACITY?area_q:AREA_SAT;
                        found_bypass<=EDGE_DIAGNOSTIC_ENABLE &&
                            (find_label>AREA_CAPACITY || area_bypass_q);
                        state<=find_return_state;
                    end else begin
                        find_label<=response_parent;find_depth<=find_depth+1'b1;state<=FIND;
                    end
                end
                MEMORY_REQUEST: if(mem_ready) begin
                    if(mem_rsp_valid) accept_response();
                    else state<=MEMORY_RESPONSE;
                end
                MEMORY_RESPONSE: if(mem_rsp_valid) accept_response();
                default: fail(4);
            endcase
        end
    end

    // synthesis translate_off
    initial begin
        if(WIDTH<1 || WIDTH>4096 || HEIGHT<1 || HEIGHT>4096 || PIXELS!=WIDTH*HEIGHT ||
           LABEL_BITS<1 || LABEL_BITS>26 || MAX_LABELS<1 || MAX_LABELS>PIXELS ||
           MIN_COMPONENT_PIXELS<0 || MIN_COMPONENT_PIXELS>255 || AREA_TRACK_LABELS<1 ||
           (64'd1<<LABEL_BITS)<=PIXELS || CANDIDATE_BASE[1:0]!=0 ||
           LABEL_BASE[1:0]!=0 || TABLE_BASE[1:0]!=0 ||
           (LOCAL_FLAGS && !BLOCK_MEMORY) ||
           (BLOCK_MEMORY && (CANDIDATE_BASE[3:0]!=0 || LABEL_BASE[3:0]!=0)))
            $fatal(1,"Invalid Canny CCL parameters");
    end
    // synthesis translate_on
endmodule
