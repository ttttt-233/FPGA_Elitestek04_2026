`timescale 1ns/1ps
// One immutable frame context. SCAN+ROOT runs after start; OUT waits for the
// scheduler token. Each worker owns its CCL flags, cache, scratch and burst RAM.
// done means every outstanding AXI transaction has drained, including errors.
module Canny_Frame_Worker #(
    parameter integer WIDTH=1280,HEIGHT=720,SLOT_BITS=22,PROFILE_ENABLE=0,
    parameter integer STREAM_LABELS=1,FAST_CACHE=1,PREFETCH_RECORD=1,FAST_BLOCK_READ=1,
    parameter integer MEMORY_BURST=128,FAST_BLOCK_NEXT=1,LOOKAHEAD_RECORD=1,
    parameter integer RESULT_WAIT_ENABLE=0,
    parameter integer MIN_COMPONENT_PIXELS=0,AREA_TRACK_LABELS=4096,
    parameter integer EDGE_DIAGNOSTIC_ENABLE=0,
    parameter [31:0] LABEL_BASE=32'h02400000,TABLE_BASE=32'h02800000
)(
    input wire clk,rst_n,start,output_start,
    input wire [31:0] input_base,
    output wire analysis_start,analysis_valid,analysis_edge,
    output wire [11:0] analysis_x,analysis_y,
    input wire analysis_ready,
    input wire [2:0] mode_request,write_slot,
    output wire [2:0] completed_mode,
    output wire busy,output_pending,processing_error,
    output reg done,frame_complete,
    output reg [31:0] last_cycles,
    // SCAN, ROOT, OUT clocks, then each stage's DDR-wait subset.
    output wire [191:0] stage_counts,
    // Passive one-hot OUT clocks: READ, MIX, WRITE, SHAPE tail, REST.
    // The scheduler shares a counter bank across the exclusive OUT owner.
    output wire [4:0] out_profile_tick,
    output wire out_profile_ddr,
    output wire [255:0] edge_counts,
    output wire [31:0] awaddr,araddr,
    output wire [7:0] awlen,arlen,
    output wire awvalid,wvalid,wlast,arvalid,rready,
    output wire [127:0] wdata,
    input wire awready,wready,bvalid,arready,rvalid,rlast,
    input wire [1:0] bresp,rresp,
    input wire [127:0] rdata
);
    reg [31:0] process_base;
    wire [1:0] core_phase;
    localparam IDLE=0,CCL_START=3,RUN=4,RETRY=5;
    // Cache indexes contain address[23:22]. Keep virtual record/label/table
    // regions fixed and distinct; translate only external AXI addresses. A
    // physical input bank must never evict a label/parent line just because its
    // bank number happens to share those index bits. Mapping stays immutable
    // until the worker has drained, and the cache is invalidated for each job.
    localparam [31:0] V_RECORD=32'h02000000,V_LABEL=32'h02400000,V_TABLE=32'h02800000;
    function [31:0] physical_address;
        input [31:0] address;
        // Continuous assignments must also reevaluate when only the frame bank
        // changes (e.g. the same first record address after a failed read).
        input [31:0] frame_base;
        begin case(address[31:22])
            V_RECORD[31:22]:physical_address={frame_base[31:22],address[21:0]};
            V_LABEL[31:22]:physical_address={LABEL_BASE[31:22],address[21:0]};
            V_TABLE[31:22]:physical_address={TABLE_BASE[31:22],address[21:0]};
            default:physical_address=address;
        endcase end
    endfunction
    localparam integer BURST_BITS=$clog2(MEMORY_BURST);
    localparam O_IDLE=0,O_READ=1,O_RESPONSE=2,O_PIXEL=3,O_AW=4,O_W=5,O_B=6;
    reg [2:0] state,out_state;
    assign busy=state!=IDLE;
    reg [2:0] frame_mode,slot;
    assign completed_mode=frame_mode;
    reg [31:0] output_count,elapsed;
    reg [15:0] result_pixel;
    reg [127:0] pixels;
    reg [127:0] output_burst[0:MEMORY_BURST-1];
    reg [127:0] output_word;
    reg [BURST_BITS-1:0] output_beats,send_beat,burst_last;
    reg [31:0] burst_address;
    reg core_finished;
    reg label_flush;
    wire edge_valid,edge_ready,edge_bit;
    wire proc_awready,proc_wready,proc_bvalid;
    wire [BURST_BITS-1:0] output_read_index=(out_state==O_W && proc_wready)?send_beat+1'b1:send_beat;
    reg [127:0] record_block;
    reg [31:0] record_group;
    reg record_valid;
    reg record_pending;
    reg [29:0] requested_record_group;
    wire [31:0] record_q=record_block[output_count[1:0]*32 +: 32];
    // Registered reset stays released across CCL_START -> RUN. A combinational
    // decode of those multi-bit states would be unsafe on an async reset pin.
    reg core_reset_n;
    wire core_done,core_error,core_mem_valid,core_mem_write,core_mem_ready;
    wire [31:0] core_addr,core_wdata;
    wire core_mem_block;
    wire [127:0] core_block_wdata,cache_rsp_block;
    wire [11:0] edge_x,edge_y;
    // Final, hysteresis-resolved edges exist in OUT, not SCAN. The shared
    // detector is cleared only when this worker receives the output token.
    assign analysis_start=output_start;
    assign analysis_valid=state==RUN && out_state==O_PIXEL && edge_valid;
    assign analysis_edge=edge_bit;
    assign analysis_x=edge_x;
    assign analysis_y=edge_y;
    wire cache_req_ready,cache_rsp_valid,cache_rsp_error;
    wire [31:0] cache_rsp_data;
    reg cache_response_compose;
    // A synchronous cache response cannot arrive before the next clock. Issue
    // the next group while accepting lane 2, so the earliest response lands
    // on lane 3: its old record is consumed before record_block is replaced.
    // No second data buffer is needed. Misses drain normally while OUT waits.
    // Keep the legacy schedule for widths that do not align four-pixel groups.
    wire lookahead_record=LOOKAHEAD_RECORD && WIDTH%4==0 &&
        out_state==O_PIXEL && edge_valid && output_count[1:0]==2 &&
        output_count<WIDTH*HEIGHT-2 && !record_pending && frame_mode!=2 &&
        ((frame_mode!=3 && frame_mode!=5) || edge_x==WIDTH-2 || edge_x+12'd2<WIDTH/2);
    wire compose_request=out_state==O_READ || lookahead_record;
    wire [29:0] compose_group=lookahead_record?(output_count[31:2]+1'b1):output_count[31:2];
    wire compose_response=cache_rsp_valid && cache_response_compose && record_pending;
    wire compose_write=out_state==O_AW || out_state==O_W || out_state==O_B;
    // Labels are a sequential write-only stream during SCAN. Reuse the result
    // burst RAM (unused until OUT) instead of fetching obsolete label lines.
    // All but the last block write complete into this private buffer; the last waits
    // for physical B. Thus all labels are visible before ROOT/OUT can start.
    wire label_request=STREAM_LABELS && core_mem_valid && core_mem_write &&
        core_mem_block && core_addr[31:22]==V_LABEL[31:22];
    wire label_accept=label_request && core_mem_ready;
    // CCL explicitly supports a response on the request-acceptance edge.
    wire label_response=label_accept && core_addr[BURST_BITS+3:4]!=MEMORY_BURST-1;
    wire label_bresponse=label_flush && out_state==O_B && proc_bvalid;
    wire burst_store=label_accept || (out_state==O_PIXEL && edge_valid && output_count[2:0]==7);
    wire [BURST_BITS-1:0] burst_store_index=label_accept?core_addr[BURST_BITS+3:4]:output_beats;
    wire [127:0] burst_store_data=label_accept?core_block_wdata:{pixels[111:0],result_pixel};
    always @(posedge clk) begin
        output_word<=output_burst[output_read_index];
        if(rst_n && burst_store) output_burst[burst_store_index]<=burst_store_data;
    end
    wire cache_request=state==RUN && !compose_write &&
        (compose_request || (core_mem_valid && !label_request));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) cache_response_compose<=0;
        else if(cache_request && cache_req_ready) cache_response_compose<=compose_request;
    end
    Canny_Hysteresis_CCL #(
        .WIDTH(WIDTH),.HEIGHT(HEIGHT),.RECORD_INPUT(1),.BLOCK_MEMORY(1),.LOCAL_FLAGS(1),
        .FAST_BLOCK_READ(FAST_BLOCK_READ),.FAST_BLOCK_NEXT(FAST_BLOCK_NEXT),.DEFER_OUTPUT(1),
        .MIN_COMPONENT_PIXELS(MIN_COMPONENT_PIXELS),.AREA_TRACK_LABELS(AREA_TRACK_LABELS),
        .EDGE_DIAGNOSTIC_ENABLE(EDGE_DIAGNOSTIC_ENABLE),
        .MAX_LABELS(((WIDTH+1)/2)*((HEIGHT+1)/2)),
        .CANDIDATE_BASE(V_RECORD),.LABEL_BASE(V_LABEL),.TABLE_BASE(V_TABLE)
    ) ccl (
        .clk(clk),.rst_n(core_reset_n),.start(state==CCL_START),
        .output_start(output_start),.output_pending(output_pending),
        .busy(),.done(core_done),.error(core_error),.error_code(),
        .mem_valid(core_mem_valid),.mem_write(core_mem_write),.mem_addr(core_addr),.mem_wdata(core_wdata),
        .mem_block(core_mem_block),.mem_block_wdata(core_block_wdata),.mem_block_rdata(cache_rsp_block),
        .mem_ready(core_mem_ready),
        .mem_rsp_valid((cache_rsp_valid && !cache_response_compose) || label_response || label_bresponse),
        .mem_rsp_error(cache_rsp_error || (label_bresponse && bresp!=0)),.mem_rdata(cache_rsp_data),
        .output_valid(edge_valid),.output_ready(edge_ready),.output_edge(edge_bit),
        .output_first(),.output_last(),.output_x(edge_x),.output_y(edge_y),
        .label_count(),.union_count(),.candidate_count(),.strong_count(),.cycle_count(),
        .edge_counts(edge_counts),
        .profile_phase(core_phase));
    wire [31:0] cache_awaddr,cache_araddr;
    wire [7:0] cache_awlen,cache_arlen;
    wire cache_awvalid,cache_wvalid,cache_wlast,cache_arvalid,cache_rready;
    wire [127:0] cache_wdata;
    assign core_mem_ready=(label_request || cache_req_ready) && !compose_request && !compose_write && state==RUN;
    Canny_Word_Cache #(.BLOCK_PORT(1),.FAST_HIT(FAST_CACHE),.LINE_BEATS(MEMORY_BURST)) cache (
        .clk(clk),.rst_n(rst_n),.invalidate(state==CCL_START),
        .req_valid(cache_request),
        .req_write(!compose_request && core_mem_write),
        .req_addr(compose_request?V_RECORD+{compose_group[27:0],4'd0}:core_addr),
        .req_block(compose_request || core_mem_block),.req_block_data(core_block_wdata),
        .req_wdata(core_wdata),.req_ready(cache_req_ready),
        .rsp_valid(cache_rsp_valid),.rsp_error(cache_rsp_error),.rsp_data(cache_rsp_data),
        .rsp_block_data(cache_rsp_block),
        .awaddr(cache_awaddr),.awlen(cache_awlen),.awvalid(cache_awvalid),.wvalid(cache_wvalid),.wlast(cache_wlast),.wdata(cache_wdata),
        .awready(proc_awready && !compose_write),
        .wready(proc_wready && !compose_write),
        .bvalid(proc_bvalid && !compose_write),.bresp(bresp),
        .araddr(cache_araddr),.arlen(cache_arlen),.arvalid(cache_arvalid),.arready(arready),
        .rvalid(rvalid),.rlast(rlast),.rresp(rresp),.rdata(rdata),.rready(cache_rready));

    function [15:0] gray565;
        input [7:0] v;
        begin gray565={v[7:3],v[7:2],v[7:3]};end
    endfunction
    wire [15:0] red={8'd0,record_q[31:27],record_q[31:29]};
    wire [15:0] green={8'd0,record_q[26:21],record_q[26:25]};
    wire [15:0] blue={8'd0,record_q[20:16],record_q[20:18]};
    wire [15:0] gray_sum=(red<<6)+(red<<3)+(red<<2)+red+
        (green<<7)+(green<<4)+(green<<2)+(green<<1)+(blue<<4)+(blue<<3)+(blue<<2)+blue;
    // Same RGB565 palette as the GUIDE axes: 0/45/90/135 degrees.
    wire [63:0] direction_palette=64'hffe0_001f_07e0_f800;
    wire [15:0] direction_color=direction_palette[record_q[3:2]*16 +: 16];
    always @(*) begin
        case(frame_mode)
            0:result_pixel=record_q[31:16];
            1:result_pixel=gray565(record_q[15:8]);
            2:result_pixel={16{edge_bit}};
            3:result_pixel=edge_x<WIDTH/2?record_q[31:16]:{16{edge_bit}};
            4:result_pixel=edge_bit?16'hf800:record_q[31:16];
            5:result_pixel=edge_x<WIDTH/2?gray565(gray_sum[15:8]):{16{edge_bit}};
            6:result_pixel=edge_bit?direction_color:record_q[31:16];
            default:result_pixel={16{edge_bit}};
        endcase
    end
    assign edge_ready=out_state==O_PIXEL;
    wire [31:0] proc_awaddr=compose_write?burst_address:physical_address(cache_awaddr,process_base);
    wire [7:0] proc_awlen=compose_write?burst_last:cache_awlen;
    wire proc_awvalid=compose_write?(out_state==O_AW):cache_awvalid;
    wire proc_wvalid=compose_write?(out_state==O_W):cache_wvalid;
    wire proc_wlast=compose_write?(send_beat==burst_last):cache_wlast;
    wire [127:0] proc_wdata=compose_write?output_word:cache_wdata;
    assign awaddr=proc_awaddr;
    assign awlen=proc_awlen;
    assign awvalid=proc_awvalid;
    assign wvalid=proc_wvalid;
    assign wlast=proc_wlast;
    assign wdata=proc_wdata;
    assign proc_awready=awready;
    assign proc_wready=wready;
    assign proc_bvalid=bvalid;
    assign araddr=physical_address(cache_araddr,process_base);
    assign arlen=cache_arlen;
    assign arvalid=cache_arvalid;
    assign rready=cache_rready;
    assign processing_error=state==RUN &&
        ((compose_response && cache_rsp_error) ||
         (out_state==O_B && proc_bvalid && bresp!=0) ||
         (core_done && (core_error || output_count!=WIDTH*HEIGHT)));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;out_state<=O_IDLE;frame_mode<=0;slot<=0;output_count<=0;
            elapsed<=0;pixels<=0;record_block<=0;record_valid<=0;record_group<=0;frame_complete<=0;
            record_pending<=0;requested_record_group<=0;
            done<=0;last_cycles<=0;process_base<=0;
            core_reset_n<=0;core_finished<=0;output_beats<=0;send_beat<=0;burst_last<=0;burst_address<=0;
            label_flush<=0;
        end else begin
            frame_complete<=0;
            done<=0;
            // Reserve the display slot only when output can actually begin.
            if(output_start && output_pending) slot<=write_slot;
            if(state!=IDLE) elapsed<=elapsed+1'b1;
            case(state)
                IDLE: if(start) begin
                    elapsed<=0;core_reset_n<=1;frame_mode<=mode_request;
                    process_base<=input_base;
                    output_count<=0;out_state<=O_IDLE;state<=CCL_START;
                    core_finished<=0;output_beats<=0;send_beat<=0;record_valid<=0;record_pending<=0;
                end
                CCL_START: state<=RUN;
                RUN: begin
                    case(out_state)
                        O_IDLE: if(edge_valid || (PREFETCH_RECORD && core_phase==3 && output_count<WIDTH*HEIGHT)) begin
                            if(frame_mode==2 || ((frame_mode==3 || frame_mode==5) && edge_x>=WIDTH/2))
                                out_state<=O_PIXEL;
                            else if(record_valid && record_group==(output_count>>2)) out_state<=O_PIXEL;
                            else if(record_pending) out_state<=O_RESPONSE;
                            else out_state<=O_READ;
                        end
                        O_READ: if(cache_req_ready) out_state<=O_RESPONSE;
                        O_RESPONSE: if(compose_response && !cache_rsp_error) out_state<=O_PIXEL;
                        O_PIXEL: if(edge_valid) begin
                            pixels<={pixels[111:0],result_pixel};
                            output_count<=output_count+1'b1;
                            out_state<=output_count[1:0]==3?O_IDLE:O_PIXEL;
                            if(output_count[2:0]==7) begin
                                if(output_beats==MEMORY_BURST-1 || output_count==WIDTH*HEIGHT-1) begin
                                    burst_last<=output_beats;send_beat<=0;
                                    burst_address<=({29'd0,slot}<<SLOT_BITS)+
                                        (((output_count>>3)-output_beats)<<4);
                                    out_state<=O_AW;
                                end else output_beats<=output_beats+1'b1;
                            end
                        end
                        O_AW: if(proc_awready) out_state<=O_W;
                        O_W: if(proc_wready) begin
                            if(send_beat==burst_last) out_state<=O_B;
                            else send_beat<=send_beat+1'b1;
                        end
                        O_B: if(proc_bvalid) begin
                            label_flush<=0;
                            out_state<=O_IDLE;
                            if(bresp!=0) begin state<=RETRY;end
                            else begin output_beats<=0;send_beat<=0;out_state<=O_IDLE;end
                        end
                        default: out_state<=O_IDLE;
                    endcase
                    if(cache_request && cache_req_ready && compose_request) begin
                        record_pending<=1;requested_record_group<=compose_group;
                    end
                    if(compose_response) begin
                        record_pending<=0;
                        if(cache_rsp_error) state<=RETRY;
                        else begin
                            record_block<=cache_rsp_block;record_group<={2'd0,requested_record_group};
                            record_valid<=1;
                            // A miss can finish on the IDLE edge immediately
                            // after lane 3. Do not enter RESPONSE after its
                            // only response has already been consumed.
                            if(out_state==O_IDLE && requested_record_group==output_count[31:2])
                                out_state<=O_PIXEL;
                        end
                    end
                    if(label_accept) begin
                        if(core_addr[BURST_BITS+3:4]==MEMORY_BURST-1) begin
                            label_flush<=1;burst_address<=physical_address({core_addr[31:BURST_BITS+4],{(BURST_BITS+4){1'b0}}},process_base);
                            burst_last<=MEMORY_BURST-1;send_beat<=0;out_state<=O_AW;
                        end
                    end
                    if(core_done) begin
                        if(core_error || output_count!=WIDTH*HEIGHT) begin
                            state<=RETRY;
                        end else core_finished<=1;
                    end
                    // Last edge can be consumed before the final burst completes.
                    // Publish only after the last successful physical B response.
                    if(core_finished && out_state==O_IDLE && output_count==WIDTH*HEIGHT &&
                       (!RESULT_WAIT_ENABLE || analysis_ready)) begin
                        frame_complete<=1;
                        last_cycles<=elapsed;state<=RETRY;
                    end
                end
                // Prefetched records can still be in flight when a CCL read
                // fails. Drain them (and any already-offered output burst)
                // before invalidating cache/switching banks. Never abandon AW.
                // The independent capture port continues throughout recovery.
                RETRY: begin
                    core_reset_n<=0;
                    case(out_state)
                        O_AW: if(proc_awready) out_state<=O_W;
                        O_W: if(proc_wready) begin
                            if(send_beat==burst_last) out_state<=O_B;
                            else send_beat<=send_beat+1'b1;
                        end
                        O_B: if(proc_bvalid) begin out_state<=O_IDLE;label_flush<=0;end
                        default: begin
                            out_state<=O_IDLE;label_flush<=0;
                            if(cache_req_ready) begin state<=IDLE;done<=1;end
                        end
                    endcase
                end
                default:state<=IDLE;
            endcase
        end
    end
    generate if(PROFILE_ENABLE) begin: g_stage_profile
        wire [31:0] counts[0:5];
        reg read_pending,write_pending;
        wire ddr_wait=(arvalid && !arready) || (read_pending && rready && !rvalid) ||
            (awvalid && !awready) || (wvalid && !wready) || (write_pending && !bvalid);
        wire out_active=state==RUN && core_phase==3;
        wire record_read=out_state==O_READ || out_state==O_RESPONSE;
        wire pixel_mix=out_state==O_PIXEL && edge_valid;
        wire shape_tail=RESULT_WAIT_ENABLE && core_finished && out_state==O_IDLE &&
            output_count==WIDTH*HEIGHT && !analysis_ready;
        assign out_profile_tick=out_active ?
            {!(record_read || pixel_mix || compose_write || shape_tail),
             shape_tail,compose_write,pixel_mix,record_read} : 5'd0;
        // Same client-side wait definition as the original OUT counter. This
        // is a subset of each state bucket, not an additional elapsed time.
        assign out_profile_ddr=ddr_wait;
        genvar k;
        for(k=0;k<6;k=k+1) begin: g_counts
            assign stage_counts[k*32 +: 32]=counts[k];
            wire count_enable=(state==RUN && core_phase==(k%3)+1) &&
                ((k<3) || ddr_wait);
            Canny_Profile_Counter_DSP counter(
                .clk(clk),.rst_n(rst_n),.clear(start),.enable(count_enable),
                .value(counts[k]));
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                read_pending<=0;write_pending<=0;
            end else begin
                if(arvalid && arready) read_pending<=1;
                if(rvalid && rready && rlast) read_pending<=0;
                if(wvalid && wready && wlast) write_pending<=1;
                if(bvalid) write_pending<=0;
            end
        end
    end else begin: g_no_stage_profile
        assign stage_counts=0;
        assign out_profile_tick=0;
        assign out_profile_ddr=0;
    end endgenerate
endmodule

// Private profiling counter: unchanged async reset, synchronous clear priority,
// enable and saturation. Two DSP accumulators replace the fabric FF/add chain.
// WIDTH is 32 in production; smaller widths exercise saturation exhaustively.
// Supported WIDTH: 2..36 (each half must fit the DSP's unsigned 18-bit C input).
module Canny_Profile_Counter_DSP #(parameter WIDTH=32)(
    input clk,rst_n,clear,enable,output [WIDTH-1:0] value);
    localparam LO=WIDTH/2, HI=WIDTH-LO;
    wire [47:0] low_result,high_result;
    wire [LO-1:0] low_count=low_result[LO-1:0];
    wire [HI-1:0] high_count=high_result[HI-1:0];
    wire tick=enable && !(&value);
    wire [17:0] low_operand=clear ? {{(18-LO){1'b0}},low_count}:18'd1;
    wire [17:0] high_operand=clear ? {{(18-HI){1'b0}},high_count}:18'd1;
    // OP=10 computes W-C. Subtracting the exposed half clears it on this edge;
    // unobserved upper DSP bits may retain carries but cannot affect lower bits.
    // Clear uses the datapath, never a combinational asynchronous reset gate.
    EFX_DSP48 #(.M_SEL("C"),.N_SEL("W"),.W_SEL("X"),.W_REG(1),
        .SIGNED(0),.RST_SYNC(0),.RST_POLARITY(0)) low_accumulator(
        .A(19'd0),.B(18'd0),.C(low_operand),.CASCIN(48'd0),
        .OP(clear?2'b10:2'b00),.SHIFT_ENA(1'b0),.CLK(clk),
        .CE(clear || tick),.RST(rst_n),.O(low_result),.CASCOUT(),.OVFL());
    EFX_DSP48 #(.M_SEL("C"),.N_SEL("W"),.W_SEL("X"),.W_REG(1),
        .SIGNED(0),.RST_SYNC(0),.RST_POLARITY(0)) high_accumulator(
        .A(19'd0),.B(18'd0),.C(high_operand),.CASCIN(48'd0),
        .OP(clear?2'b10:2'b00),.SHIFT_ENA(1'b0),.CLK(clk),
        .CE(clear || (tick && (&low_count))),.RST(rst_n),.O(high_result),.CASCOUT(),.OVFL());
    assign value={high_count,low_count};
endmodule
