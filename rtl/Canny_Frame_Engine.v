`timescale 1ns/1ps
// Exact complete-frame Canny. Three banks separate immutable processing,
// newest-complete waiting input, and capture. Display/media is separate.
module Canny_Frame_Engine #(
    parameter integer WIDTH=1280,HEIGHT=720,
    parameter integer PROFILE_ENABLE=0,CLOCK_HZ=96000000,
    parameter integer DIAGNOSTIC_ENABLE=0,
    parameter integer DOUBLE_BUFFER=1,
    parameter integer TRIPLE_BUFFER=1,
    parameter integer METADATA_ENABLE=0,METADATA_WIDTH=13,
    parameter integer RESULT_WAIT_ENABLE=0,
    parameter [METADATA_WIDTH-1:0] DEFAULT_METADATA={1'b1,12'd96},
    parameter integer STREAM_LABELS=1,FAST_CACHE=1,PREFETCH_RECORD=1,
    parameter integer FAST_BLOCK_READ=1,
    parameter integer MIN_COMPONENT_PIXELS=0,AREA_TRACK_LABELS=4096,
    parameter [31:0] CAPTURE_BASE=32'h02000000,
    parameter [31:0] CAPTURE_BASE_1=32'h02c00000,
    parameter [31:0] CAPTURE_BASE_2=32'h03000000,
    parameter [31:0] LABEL_BASE=32'h02400000,
    parameter [31:0] TABLE_BASE=32'h02800000,
    parameter integer SLOT_BITS=22
)(
    input wire clk,rst_n,enable,pclk,vs,de,
    input wire [15:0] rgb565,
    input wire [7:0] median_gray,
    input wire [1:0] candidate_class,gradient_direction,
    input wire [METADATA_WIDTH-1:0] frame_metadata,
    output reg [METADATA_WIDTH-1:0] completed_metadata,
    output wire analysis_start,analysis_valid,analysis_edge,
    output wire [11:0] analysis_x,analysis_y,
    input wire analysis_ready,
    input wire [2:0] mode_request,write_slot,
    output reg frame_complete,
    output reg [31:0] completed_frames,failed_frames,last_cycles,
    output wire [143:0] profile_ms_bcd,
    output wire profile_valid,
    output wire [143:0] schedule_bcd,
    output wire schedule_valid,
    output wire [31:0] awaddr,araddr,
    output wire [7:0] awlen,arlen,
    output wire awvalid,wvalid,wlast,arvalid,rready,
    output wire [127:0] wdata,
    input wire awready,wready,bvalid,arready,rvalid,rlast,
    input wire [1:0] bresp,rresp,
    input wire [127:0] rdata
);
    localparam IDLE=0,CAP_START=1,CAP_WAIT=2,CCL_START=3,RUN=4,RETRY=5;
    localparam O_IDLE=0,O_READ=1,O_RESPONSE=2,O_PIXEL=3,O_AW=4,O_W=5,O_B=6;
    reg [2:0] state,out_state;
    reg [2:0] frame_mode,slot;
    reg [31:0] output_count,elapsed;
    reg [15:0] result_pixel;
    reg [127:0] pixels;
    reg [127:0] output_burst[0:15];
    reg [127:0] output_word;
    reg [3:0] output_beats,send_beat,burst_last;
    reg [31:0] burst_address;
    reg core_finished;
    reg label_flush;
    wire edge_valid,edge_ready,edge_bit;
    wire proc_awready,proc_wready,proc_bvalid;
    wire [3:0] output_read_index=(out_state==O_W && proc_wready)?send_beat+4'd1:send_beat;
    reg [127:0] record_block;
    reg [31:0] record_group;
    reg record_valid;
    wire [31:0] record_q=record_block[output_count[1:0]*32 +: 32];
    // Registered reset stays released across CCL_START -> RUN. A combinational
    // decode of those multi-bit states would be unsafe on an async reset pin.
    reg core_reset_n;
    wire cap_done,cap_error,cap_started,cap_busy;
    reg [2:0] bank_ready;
    reg [1:0] capture_bank,process_bank;
    wire [METADATA_WIDTH-1:0] captured_metadata;
    reg [METADATA_WIDTH-1:0] bank_metadata[0:2],process_metadata;
    wire process_owned=state==CCL_START || state==RUN || state==RETRY;
    wire free0=!bank_ready[0] && !(process_owned && process_bank==0);
    wire free1=!bank_ready[1] && !(process_owned && process_bank==1);
    wire free2=TRIPLE_BUFFER && !bank_ready[2] && !(process_owned && process_bank==2);
    wire [1:0] next_capture_bank=free0?2'd0:free1?2'd1:2'd2;
    wire capture_good=cap_done && !cap_error;
    // Three-bank mailbox: keep only the newest COMPLETE waiting frame. If
    // completion and processing start coincide, consume that new frame directly.
    // The two-bank parameter retains the accepted FIFO-like schedule for A/B.
    wire [1:0] next_process_bank=TRIPLE_BUFFER?
        (capture_good?capture_bank:bank_ready[0]?2'd0:bank_ready[1]?2'd1:2'd2):
        ((&bank_ready[1:0])?(capture_bank==0?2'd1:2'd0):(bank_ready[0]?2'd0:2'd1));
    wire cap_start=DOUBLE_BUFFER?
        (enable && !cap_busy && !cap_done && (free0 || free1 || free2)):(state==CAP_START);
    wire take_bank=DOUBLE_BUFFER && state==IDLE && enable &&
        ((|bank_ready) || (TRIPLE_BUFFER && capture_good));
    function [31:0] record_base;
        input [1:0] bank;
        begin case(bank)
            1:record_base=CAPTURE_BASE_1;
            2:record_base=CAPTURE_BASE_2;
            default:record_base=CAPTURE_BASE;
        endcase end
    endfunction
    wire [31:0] process_base=DOUBLE_BUFFER?record_base(process_bank):CAPTURE_BASE;
    reg [2:0] ready_next;
    always @(*) begin
        ready_next=bank_ready;
        if(capture_good) begin
            if(TRIPLE_BUFFER) ready_next=3'b001<<capture_bank;
            else ready_next[capture_bank]=1'b1;
        end
        if(take_bank) ready_next[next_process_bank]=1'b0;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            bank_ready<=0;capture_bank<=0;process_bank<=0;
            bank_metadata[0]<=DEFAULT_METADATA;bank_metadata[1]<=DEFAULT_METADATA;bank_metadata[2]<=DEFAULT_METADATA;
        end
        else if(DOUBLE_BUFFER) begin
            if(cap_start) capture_bank<=next_capture_bank;
            bank_ready<=ready_next;
            if(capture_good) bank_metadata[capture_bank]<=captured_metadata;
            if(take_bank) process_bank<=next_process_bank;
        end
    end
    wire [1:0] core_phase;
    wire [31:0] cap_awaddr;
    wire [7:0] cap_awlen;
    wire cap_awvalid,cap_wvalid,cap_wlast;
    wire [127:0] cap_wdata;
    wire capture_phase=state==CAP_START || state==CAP_WAIT;
    wire cap_awready,cap_wready,cap_bvalid;
    Canny_Frame_Capture #(.PIXELS(WIDTH*HEIGHT),.BASE(CAPTURE_BASE),.DYNAMIC_BASE(DOUBLE_BUFFER),.METADATA_WIDTH(METADATA_WIDTH)) capture (
        .clk(clk),.rst_n(rst_n),.start(cap_start),.pclk(pclk),.vs(vs),.de(de),
        .base_address(record_base(next_capture_bank)),
        .record_data({rgb565,median_gray,4'd0,gradient_direction,candidate_class}),
        .frame_metadata(METADATA_ENABLE?frame_metadata:DEFAULT_METADATA),.captured_metadata(captured_metadata),
        .busy(cap_busy),.done(cap_done),.error(cap_error),.frame_started(cap_started),
        .awaddr(cap_awaddr),.awlen(cap_awlen),.awvalid(cap_awvalid),
        .wvalid(cap_wvalid),.wlast(cap_wlast),.wdata(cap_wdata),
        .awready(cap_awready),.wready(cap_wready),
        .bvalid(cap_bvalid),.bresp(bresp));

    wire core_done,core_error,core_mem_valid,core_mem_write,core_mem_ready;
    wire [31:0] core_addr,core_wdata;
    wire core_mem_block;
    wire [127:0] core_block_wdata,cache_rsp_block;
    wire [11:0] edge_x,edge_y;
    assign analysis_start=state==CCL_START;
    assign analysis_valid=state==RUN && out_state==O_PIXEL && edge_valid;
    assign analysis_edge=edge_bit;
    assign analysis_x=edge_x;
    assign analysis_y=edge_y;
    wire cache_req_ready,cache_rsp_valid,cache_rsp_error;
    wire [31:0] cache_rsp_data;
    wire compose_request=out_state==O_READ;
    wire compose_write=out_state==O_AW || out_state==O_W || out_state==O_B;
    // Labels are a sequential write-only stream during SCAN. Reuse the result
    // burst RAM (unused until OUT) instead of fetching obsolete label lines.
    // First 15 block writes complete into this private buffer; the 16th waits
    // for physical B. Thus all labels are visible before ROOT/OUT can start.
    wire label_request=STREAM_LABELS && core_mem_valid && core_mem_write &&
        core_mem_block && core_addr[31:22]==LABEL_BASE[31:22];
    wire label_accept=label_request && core_mem_ready;
    // CCL explicitly supports a response on the request-acceptance edge.
    wire label_response=label_accept && core_addr[7:4]!=15;
    wire label_bresponse=label_flush && out_state==O_B && proc_bvalid;
    wire burst_store=label_accept || (out_state==O_PIXEL && edge_valid && output_count[2:0]==7);
    wire [3:0] burst_store_index=label_accept?core_addr[7:4]:output_beats;
    wire [127:0] burst_store_data=label_accept?core_block_wdata:{pixels[111:0],result_pixel};
    always @(posedge clk) begin
        output_word<=output_burst[output_read_index];
        if(rst_n && burst_store) output_burst[burst_store_index]<=burst_store_data;
    end
    wire cache_request=state==RUN && !compose_write &&
        (compose_request || (core_mem_valid && !label_request));
    reg cache_response_compose;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) cache_response_compose<=0;
        else if(cache_request && cache_req_ready) cache_response_compose<=compose_request;
    end
    Canny_Hysteresis_CCL #(
        .WIDTH(WIDTH),.HEIGHT(HEIGHT),.RECORD_INPUT(1),.BLOCK_MEMORY(1),.LOCAL_FLAGS(1),
        .FAST_BLOCK_READ(FAST_BLOCK_READ),
        .MIN_COMPONENT_PIXELS(MIN_COMPONENT_PIXELS),.AREA_TRACK_LABELS(AREA_TRACK_LABELS),
        .MAX_LABELS(((WIDTH+1)/2)*((HEIGHT+1)/2)),
        .CANDIDATE_BASE(CAPTURE_BASE),.LABEL_BASE(LABEL_BASE),.TABLE_BASE(TABLE_BASE)
    ) ccl (
        .clk(clk),.rst_n(core_reset_n),.start(state==CCL_START),
        .output_start(1'b0),.output_pending(),
        .busy(),.done(core_done),.error(core_error),.error_code(),
        .mem_valid(core_mem_valid),.mem_write(core_mem_write),.mem_addr(core_addr),.mem_wdata(core_wdata),
        .mem_block(core_mem_block),.mem_block_wdata(core_block_wdata),.mem_block_rdata(cache_rsp_block),
        .mem_ready(core_mem_ready),
        .mem_rsp_valid((cache_rsp_valid && !cache_response_compose) || label_response || label_bresponse),
        .mem_rsp_error(cache_rsp_error || (label_bresponse && bresp!=0)),.mem_rdata(cache_rsp_data),
        .output_valid(edge_valid),.output_ready(edge_ready),.output_edge(edge_bit),
        .output_first(),.output_last(),.output_x(edge_x),.output_y(edge_y),
        .label_count(),.union_count(),.candidate_count(),.strong_count(),.cycle_count(),
        .profile_phase(core_phase));
    wire [31:0] cache_awaddr,cache_araddr;
    wire [7:0] cache_awlen,cache_arlen;
    wire cache_awvalid,cache_wvalid,cache_wlast,cache_arvalid,cache_rready;
    wire [127:0] cache_wdata;
    // Translate only the CCL's virtual candidate region; labels/table retain
    // their addresses. Cache is invalidated before switching record banks.
    wire [31:0] physical_core_addr=(DOUBLE_BUFFER &&
        core_addr[31:22]==CAPTURE_BASE[31:22])?
        {process_base[31:22],core_addr[21:0]}:core_addr;
    assign core_mem_ready=(label_request || cache_req_ready) && !compose_request && !compose_write && state==RUN;
    Canny_Word_Cache #(.BLOCK_PORT(1),.FAST_HIT(FAST_CACHE)) cache (
        .clk(clk),.rst_n(rst_n),.invalidate(state==CCL_START),
        .req_valid(cache_request),
        .req_write(!compose_request && core_mem_write),
        .req_addr(compose_request?process_base+((output_count>>2)<<4):physical_core_addr),
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
    wire [31:0] proc_awaddr=compose_write?burst_address:cache_awaddr;
    wire [7:0] proc_awlen=compose_write?{4'd0,burst_last}:cache_awlen;
    wire proc_awvalid=compose_write?(out_state==O_AW):cache_awvalid;
    wire proc_wvalid=compose_write?(out_state==O_W):cache_wvalid;
    wire proc_wlast=compose_write?(send_beat==burst_last):cache_wlast;
    wire [127:0] proc_wdata=compose_write?output_word:cache_wdata;
    generate if(DOUBLE_BUFFER) begin: g_write_arb
        AXI_Write_Arbiter_2 arb (
            .clk(clk),.rst_n(rst_n),
            .p0_addr(cap_awaddr),.p0_len(cap_awlen),.p0_valid(cap_awvalid),
            .p0_wvalid(cap_wvalid),.p0_wlast(cap_wlast),.p0_data(cap_wdata),
            .p0_ready(cap_awready),.p0_wready(cap_wready),.p0_bvalid(cap_bvalid),
            .p1_addr(proc_awaddr),.p1_len(proc_awlen),.p1_valid(proc_awvalid),
            .p1_wvalid(proc_wvalid),.p1_wlast(proc_wlast),.p1_data(proc_wdata),
            .p1_ready(proc_awready),.p1_wready(proc_wready),.p1_bvalid(proc_bvalid),
            .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),
            .wvalid(wvalid),.wlast(wlast),.wdata(wdata),.wready(wready),.bvalid(bvalid));
    end else begin: g_serial_write
        assign awaddr=capture_phase?cap_awaddr:proc_awaddr;
        assign awlen=capture_phase?cap_awlen:proc_awlen;
        assign awvalid=capture_phase?cap_awvalid:proc_awvalid;
        assign wvalid=capture_phase?cap_wvalid:proc_wvalid;
        assign wlast=capture_phase?cap_wlast:proc_wlast;
        assign wdata=capture_phase?cap_wdata:proc_wdata;
        assign cap_awready=awready && capture_phase;
        assign cap_wready=wready && capture_phase;
        assign cap_bvalid=bvalid && capture_phase;
        assign proc_awready=awready && !capture_phase;
        assign proc_wready=wready && !capture_phase;
        assign proc_bvalid=bvalid && !capture_phase;
    end endgenerate
    assign araddr=cache_araddr;
    assign arlen=cache_arlen;
    assign arvalid=cache_arvalid;
    assign rready=cache_rready;
    // Instrumentation observes the existing schedule, never controls it.
    reg capture_started;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) capture_started<=0;
        else if(DOUBLE_BUFFER?cap_start:(state==IDLE)) capture_started<=0;
        else if(cap_started) capture_started<=1;
    end
    reg profile_started;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) profile_started<=0;else if(enable) profile_started<=1;
    // Pipeline report measures NON-overlapping publication-interval time:
    // WAIT/CAP are only the exposed camera wait while processing is idle.
    wire profile_camera=DOUBLE_BUFFER?!process_owned:capture_phase;
    wire [2:0] profile_phase=profile_camera?
        ((capture_started || cap_started)?3'd1:3'd0):
        (state==CCL_START || core_phase==1)?3'd2:
        core_phase==2?3'd3:3'd4;
    generate if(PROFILE_ENABLE) begin: g_profile
        Canny_Profiler #(.CLOCK_HZ(CLOCK_HZ),.INTERVAL_MODE(DOUBLE_BUFFER)) profiler (
            .external_counts(384'd0),
            .clk(clk),.rst_n(rst_n),.start(DOUBLE_BUFFER?(!profile_started && enable):(state==IDLE && enable)),
            .active(DOUBLE_BUFFER?profile_started:(capture_phase || state==CCL_START || state==RUN)),
            .publish(frame_complete),.phase(profile_phase),
            .awvalid(awvalid),.awready(awready),.wvalid(wvalid),.wready(wready),
            .wlast(wlast),.bvalid(bvalid),.arvalid(arvalid),.arready(arready),
            .rvalid(rvalid),.rready(rready),.rlast(rlast),
            .milliseconds_bcd(profile_ms_bcd),.report_valid(profile_valid));
    end else begin: g_no_profile
        assign profile_ms_bcd=0;assign profile_valid=0;
    end endgenerate
    wire processing_error=state==RUN &&
        ((out_state==O_RESPONSE && cache_rsp_valid && cache_response_compose && cache_rsp_error) ||
         (out_state==O_B && proc_bvalid && bresp!=0) ||
         (core_done && (core_error || output_count!=WIDTH*HEIGHT)));
    generate if(DIAGNOSTIC_ENABLE) begin: g_schedule
        Canny_Schedule_Monitor #(.CLOCK_HZ(CLOCK_HZ)) monitor (
            .clk(clk),.rst_n(rst_n),.vs(vs),.cap_start(cap_start),
            .cap_started(cap_started),.cap_done(cap_done),.cap_error(cap_error),
            .processing_error(processing_error),.publish(frame_complete),.phase(profile_phase),
            .data_bcd(schedule_bcd),.valid(schedule_valid));
    end else begin: g_no_schedule
        assign schedule_bcd=0;assign schedule_valid=0;
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;out_state<=O_IDLE;frame_mode<=0;slot<=0;output_count<=0;
            elapsed<=0;pixels<=0;record_block<=0;record_valid<=0;record_group<=0;frame_complete<=0;
            completed_frames<=0;failed_frames<=0;last_cycles<=0;
            core_reset_n<=0;core_finished<=0;output_beats<=0;send_beat<=0;burst_last<=0;burst_address<=0;
            label_flush<=0;process_metadata<=DEFAULT_METADATA;completed_metadata<=DEFAULT_METADATA;
        end else begin
            frame_complete<=0;
            if((cap_done && cap_error) || processing_error)
                failed_frames<=failed_frames+{31'd0,(cap_done && cap_error)}+{31'd0,processing_error};
            if(state!=IDLE) elapsed<=elapsed+1'b1;
            case(state)
                IDLE: if(DOUBLE_BUFFER) begin
                    if(take_bank) begin
                        process_metadata<=capture_good?captured_metadata:bank_metadata[next_process_bank];
                        elapsed<=0;core_reset_n<=1;frame_mode<=mode_request;slot<=write_slot;
                        output_count<=0;out_state<=O_IDLE;state<=CCL_START;
                        core_finished<=0;output_beats<=0;send_beat<=0;record_valid<=0;
                    end
                end else if(enable) begin elapsed<=0;state<=CAP_START;end
                CAP_START: state<=CAP_WAIT;
                CAP_WAIT: if(cap_done) begin
                    if(cap_error) begin state<=RETRY;end
                    else begin
                        core_reset_n<=1;
                        process_metadata<=captured_metadata;
                        frame_mode<=mode_request;slot<=write_slot;
                        output_count<=0;out_state<=O_IDLE;state<=CCL_START;
                        core_finished<=0;output_beats<=0;send_beat<=0;
                        record_valid<=0;
                    end
                end
                CCL_START: state<=RUN;
                RUN: begin
                    case(out_state)
                        O_IDLE: if(edge_valid || (PREFETCH_RECORD && core_phase==3 && output_count<WIDTH*HEIGHT)) begin
                            if(frame_mode==2 || ((frame_mode==3 || frame_mode==5) && edge_x>=WIDTH/2))
                                out_state<=O_PIXEL;
                            else if(record_valid && record_group==(output_count>>2)) out_state<=O_PIXEL;
                            else out_state<=O_READ;
                        end
                        O_READ: if(cache_req_ready) out_state<=O_RESPONSE;
                        O_RESPONSE: if(cache_rsp_valid && cache_response_compose) begin
                            if(cache_rsp_error) begin state<=RETRY;end
                            else begin
                                record_block<=cache_rsp_block;record_group<=output_count>>2;
                                record_valid<=1;out_state<=O_PIXEL;
                            end
                        end
                        O_PIXEL: if(edge_valid) begin
                            pixels<={pixels[111:0],result_pixel};
                            output_count<=output_count+1'b1;
                            out_state<=output_count[1:0]==3?O_IDLE:O_PIXEL;
                            if(output_count[2:0]==7) begin
                                if(output_beats==15 || output_count==WIDTH*HEIGHT-1) begin
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
                    if(label_accept) begin
                        if(core_addr[7:4]==15) begin
                            label_flush<=1;burst_address<={core_addr[31:8],8'd0};
                            burst_last<=15;send_beat<=0;out_state<=O_AW;
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
                        frame_complete<=1;completed_frames<=completed_frames+1'b1;
                        completed_metadata<=process_metadata;
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
                            if(cache_req_ready) state<=IDLE;
                        end
                    endcase
                end
                default:state<=IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if(WIDTH*HEIGHT*4>32'h00400000 || CAPTURE_BASE[21:0]!=0 ||
           CAPTURE_BASE_1[21:0]!=0 || LABEL_BASE[21:0]!=0 || TABLE_BASE[21:0]!=0)
            $fatal(1,"Record banks/scratch must fit distinct aligned 4 MiB regions");
        if(CAPTURE_BASE==LABEL_BASE || CAPTURE_BASE==TABLE_BASE || LABEL_BASE==TABLE_BASE ||
           (DOUBLE_BUFFER && (CAPTURE_BASE_1==CAPTURE_BASE || CAPTURE_BASE_1==LABEL_BASE || CAPTURE_BASE_1==TABLE_BASE)))
            $fatal(1,"Canny DDR regions overlap");
        if(DOUBLE_BUFFER && TRIPLE_BUFFER && (CAPTURE_BASE_2[21:0]!=0 ||
           CAPTURE_BASE_2==CAPTURE_BASE || CAPTURE_BASE_2==CAPTURE_BASE_1 ||
           CAPTURE_BASE_2==LABEL_BASE || CAPTURE_BASE_2==TABLE_BASE))
            $fatal(1,"Third input bank must be distinct and 4 MiB aligned");
    end
    // synthesis translate_on
endmodule
