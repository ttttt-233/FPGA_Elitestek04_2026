`timescale 1ns/1ps
// Capture -> SCAN+ROOT -> OUT, on different immutable frames. Two workers
// alternate, with one scan token and one output token. Four input banks cover
// both workers, one newest-complete mailbox and the camera being captured.
module Canny_Frame_Pipeline_Engine #(
    parameter integer WIDTH=1280,HEIGHT=720,SLOT_BITS=22,
    parameter integer PROFILE_ENABLE=0,CLOCK_HZ=96000000,DIAGNOSTIC_ENABLE=0,
    parameter integer OUT_PROFILE_ENABLE=0,RAW_EXPORT_ENABLE=0,
    parameter [31:0] RAW_BASE=32'h1fa00000,
    parameter integer OVERLAP=1,MEMORY_BURST=128,FAST_BLOCK_NEXT=1,LOOKAHEAD_RECORD=1,
    parameter integer MIN_COMPONENT_PIXELS=0,AREA_TRACK_LABELS=4096,
    parameter integer EDGE_DIAGNOSTIC_ENABLE=0,
    parameter integer METADATA_ENABLE=0,METADATA_WIDTH=13,RESULT_WAIT_ENABLE=0,
    parameter [METADATA_WIDTH-1:0] DEFAULT_METADATA={1'b1,12'd96},
    parameter [31:0] CAPTURE_BASE=32'h02000000,CAPTURE_BASE_1=32'h02c00000,
    parameter [31:0] CAPTURE_BASE_2=32'h03000000,CAPTURE_BASE_3=32'h03400000,
    parameter [31:0] LABEL_BASE=32'h02400000,TABLE_BASE=32'h02800000,
    parameter [31:0] LABEL_BASE_1=32'h03800000,TABLE_BASE_1=32'h03c00000
)(
    input wire clk,rst_n,enable,pclk,vs,de,
    input wire [15:0] rgb565,
    input wire raw_vs,raw_de,
    input wire [7:0] raw_data,
    input wire [1023:0] export_header,
    input wire snapshot_request,
    output reg snapshot_ready,snapshot_error,
    output reg [2:0] snapshot_slot,
    output wire [15:0] raw_diagnostics,
    output wire [2:0] completed_mode,
    input wire [7:0] median_gray,
    input wire [1:0] candidate_class,gradient_direction,
    input wire [METADATA_WIDTH-1:0] frame_metadata,
    output wire [METADATA_WIDTH-1:0] completed_metadata,
    output wire analysis_start,analysis_valid,analysis_edge,
    output wire [11:0] analysis_x,analysis_y,
    input wire analysis_ready,
    input wire [2:0] mode_request,write_slot,
    output wire frame_complete,
    output reg [31:0] completed_frames,failed_frames,last_cycles,
    output wire [143:0] profile_ms_bcd,schedule_bcd,
    output wire profile_valid,schedule_valid,
    output wire [143:0] out_profile_bcd,
    output wire out_profile_valid,
    output wire [191:0] edge_stats_bcd,
    output wire edge_stats_valid,
    // Sticky startup milestones remain observable before the first publication.
    output reg [7:0] startup_events,
    output wire [31:0] awaddr,araddr,
    output wire [7:0] awlen,arlen,
    output wire awvalid,wvalid,wlast,arvalid,rready,
    output wire [127:0] wdata,
    input wire awready,wready,bvalid,arready,rvalid,rlast,
    input wire [1:0] bresp,rresp,
    input wire [127:0] rdata
);
    reg [3:0] bank_ready;
    reg [1:0] capture_bank,worker_bank[0:1];
    reg [1:0] occupied;
    reg scan_busy,scan_owner,out_busy,out_owner;
    wire [1:0] worker_done,worker_pending,worker_publish,worker_error;
    wire [2:0] worker_modes[0:1];
    wire attach_done,attach_error,attach_busy;
    reg attaching;
    reg snapshot_inflight,capture_raw;
    reg [1:0] snapshot_bank;
    reg [7:0] last_raw_error;
    reg [1023:0] header_hold;
    wire snapshot_frame=RAW_EXPORT_ENABLE && snapshot_inflight && worker_bank[out_owner]==snapshot_bank;
    wire attach_start=RAW_EXPORT_ENABLE && snapshot_frame && |worker_publish;
    wire [1:0] retired=RAW_EXPORT_ENABLE?
        ((worker_done & ~(attaching?(2'b01<<out_owner):2'b00)) |
         (attach_done?(2'b01<<out_owner):2'b00)):worker_done;
    assign completed_mode=worker_modes[out_owner];
    wire [1:0] worker_start,worker_output_start;
    wire [31:0] worker_cycles[0:1];
    wire [191:0] worker_counts[0:1];
    wire [4:0] worker_out_tick[0:1];
    wire [1:0] worker_out_ddr;
    wire [255:0] worker_edge_counts[0:1];
    wire cap_done,cap_error,cap_started,cap_busy;
    wire [METADATA_WIDTH-1:0] captured_metadata;
    reg [METADATA_WIDTH-1:0] bank_metadata[0:3],job_metadata[0:1];
    wire [1:0] worker_analysis_start,worker_analysis_valid,worker_analysis_edge;
    wire [11:0] worker_analysis_x[0:1],worker_analysis_y[0:1];
    // OUT remains exclusive through shape analysis, publication and AXI drain.
    // SCAN may already belong to the next frame: never use scan_owner here.
    assign analysis_start=|worker_analysis_start;
    assign analysis_valid=out_busy && worker_analysis_valid[out_owner];
    assign analysis_edge=worker_analysis_edge[out_owner];
    assign analysis_x=worker_analysis_x[out_owner];
    assign analysis_y=worker_analysis_y[out_owner];
    assign completed_metadata=job_metadata[out_owner];
    integer metadata_bank;
    wire capture_good=cap_done && !cap_error;
    wire [3:0] free_bank;
    genvar b,w;
    generate for(b=0;b<4;b=b+1) begin: g_free
        assign free_bank[b]=!bank_ready[b] &&
            !(occupied[0] && worker_bank[0]==b) && !(occupied[1] && worker_bank[1]==b);
    end endgenerate
    wire [1:0] next_capture_bank=free_bank[0]?2'd0:free_bank[1]?2'd1:free_bank[2]?2'd2:2'd3;
    wire [1:0] next_process_bank=capture_good?capture_bank:
        bank_ready[0]?2'd0:bank_ready[1]?2'd1:bank_ready[2]?2'd2:2'd3;
    wire next_worker=occupied[0];
    // Normal live frames never capture/copy RAW or wait for the attachment.
    // One requested SNAP owns its input bank until its immutable attachment
    // and the media controller's slot lease are established.
    wire cap_start=enable && !cap_busy && !cap_done && |free_bank &&
        (!RAW_EXPORT_ENABLE || !snapshot_inflight);
    wire take_bank=enable && !scan_busy && !(&occupied) && (OVERLAP || occupied==0) &&
        ((|bank_ready) || capture_good);
    assign worker_start={take_bank && next_worker,take_bank && !next_worker};
    wire next_out=worker_pending[0]?1'b0:1'b1;
    // Keep the OUT token through drain. In particular, the next slot is latched
    // only AFTER the previous publication has advanced Frame_Buffer_Control.
    wire grant_output=!out_busy && (|worker_pending) && !frame_complete;
    assign worker_output_start={grant_output && next_out,grant_output && !next_out};
    assign frame_complete=(|worker_publish && !snapshot_frame) || (attach_done && !attach_error);
    wire processing_error=|worker_error;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) startup_events<=0;
        else startup_events<=startup_events | {frame_complete,
            (analysis_ready && out_busy),grant_output,processing_error,
            take_bank,(cap_done && cap_error),cap_done,cap_start};
    end
    function [31:0] record_base;
        input [1:0] bank;
        begin case(bank)
            1:record_base=CAPTURE_BASE_1;
            2:record_base=CAPTURE_BASE_2;
            3:record_base=CAPTURE_BASE_3;
            default:record_base=CAPTURE_BASE;
        endcase end
    endfunction
    reg [3:0] ready_next;
    always @(*) begin
        ready_next=bank_ready;
        if(capture_good) ready_next=4'b0001<<capture_bank;
        if(take_bank) ready_next[next_process_bank]=0;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            bank_ready<=0;capture_bank<=0;worker_bank[0]<=0;worker_bank[1]<=0;occupied<=0;
            scan_busy<=0;scan_owner<=0;out_busy<=0;out_owner<=0;
            completed_frames<=0;failed_frames<=0;last_cycles<=0;
            for(metadata_bank=0;metadata_bank<4;metadata_bank=metadata_bank+1)
                bank_metadata[metadata_bank]<=DEFAULT_METADATA;
            job_metadata[0]<=DEFAULT_METADATA;job_metadata[1]<=DEFAULT_METADATA;
        end else begin
            bank_ready<=ready_next;
            if(capture_good) bank_metadata[capture_bank]<=captured_metadata;
            if(cap_start) capture_bank<=next_capture_bank;
            occupied<=occupied & ~retired;
            if(scan_busy && (worker_pending[scan_owner] || worker_done[scan_owner])) scan_busy<=0;
            if(out_busy && retired[out_owner]) out_busy<=0;
            if(take_bank) begin
                worker_bank[next_worker]<=next_process_bank;
                job_metadata[next_worker]<=capture_good?captured_metadata:bank_metadata[next_process_bank];
                occupied[next_worker]<=1;scan_busy<=1;scan_owner<=next_worker;
            end
            if(grant_output) begin out_busy<=1;out_owner<=next_out;end
            if(frame_complete) begin
                completed_frames<=completed_frames+1'b1;
                last_cycles<=RAW_EXPORT_ENABLE?worker_cycles[out_owner]:(worker_publish[0]?worker_cycles[0]:worker_cycles[1]);
            end
            failed_frames<=failed_frames+{31'd0,(cap_done && cap_error)}+
                {31'd0,worker_error[0]}+{31'd0,worker_error[1]}+{31'd0,(attach_done && attach_error)};
        end
    end
    localparam RAW_BYTES=((WIDTH*HEIGHT+2047)/2048)*2048;
    wire [31:0] cap_awaddr,proc_awaddr;
    wire [31:0] fc_awaddr,rc_awaddr,at_awaddr,joined_awaddr,wr_araddr,at_araddr;
    wire [7:0] fc_awlen,rc_awlen,at_awlen,joined_awlen,wr_arlen,at_arlen;
    wire [127:0] fc_wdata,rc_wdata,at_wdata,joined_wdata;
    wire fc_awvalid,fc_wvalid,fc_wlast,fc_awready,fc_wready,fc_bvalid;
    wire rc_awvalid,rc_wvalid,rc_wlast,rc_awready,rc_wready,rc_bvalid;
    wire at_awvalid,at_wvalid,at_wlast,at_awready,at_wready,at_bvalid;
    wire joined_awvalid,joined_wvalid,joined_wlast,joined_awready,joined_wready,joined_bvalid;
    wire wr_arvalid,wr_arready,wr_rvalid,wr_rready,at_arvalid,at_arready,at_rvalid,at_rready;
    wire fc_busy,fc_done,fc_error,rc_busy,rc_done,rc_error,raw_active;
    assign raw_diagnostics={last_raw_error,snapshot_ready,attaching,rc_busy,fc_busy,
        capture_raw,snapshot_inflight,snapshot_request,snapshot_error};
    wire [1:0] fc_pixel_diagnostic,rc_pixel_diagnostic;
    reg capture_pending,fc_seen,rc_seen;
    assign cap_busy=fc_busy || (RAW_EXPORT_ENABLE && capture_raw && (rc_busy || capture_pending));
    assign cap_done=RAW_EXPORT_ENABLE && capture_raw?
        (capture_pending && (fc_seen || fc_done) && (rc_seen || rc_done)):fc_done;
    assign cap_error=fc_error || (RAW_EXPORT_ENABLE && capture_raw && rc_error);
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            attaching<=0;header_hold<=0;snapshot_inflight<=0;capture_raw<=0;
            snapshot_bank<=0;snapshot_ready<=0;snapshot_error<=0;snapshot_slot<=0;last_raw_error<=0;
        end
        else begin
            snapshot_error<=0;
            if(cap_start) begin
                capture_raw<=RAW_EXPORT_ENABLE && snapshot_request;
                if(RAW_EXPORT_ENABLE && snapshot_request) begin
                    snapshot_inflight<=1;snapshot_bank<=next_capture_bank;last_raw_error<=0;
                end
            end
            if(attach_start) begin attaching<=1;header_hold<=export_header;end
            if(attach_done) attaching<=0;
            if(attach_done && !attach_error) begin snapshot_ready<=1;snapshot_slot<=write_slot;end
            if(snapshot_ready && !snapshot_request) begin snapshot_ready<=0;snapshot_inflight<=0;end
            if((cap_done && cap_error && capture_raw) ||
               (worker_error[0] && snapshot_inflight && worker_bank[0]==snapshot_bank) ||
               (worker_error[1] && snapshot_inflight && worker_bank[1]==snapshot_bank) ||
               (attach_done && attach_error)) begin
                snapshot_error<=1;snapshot_inflight<=0;snapshot_ready<=0;
                last_raw_error<={rc_pixel_diagnostic,fc_pixel_diagnostic,attach_error,rc_error,fc_error,|worker_error};
            end
        end
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin capture_pending<=0;fc_seen<=0;rc_seen<=0;end
        else begin
            if(cap_start) begin capture_pending<=1;fc_seen<=0;rc_seen<=0;end
            if(fc_done) fc_seen<=1;
            if(rc_done) rc_seen<=1;
            if(cap_done) capture_pending<=0;
        end
    wire [7:0] cap_awlen,proc_awlen;
    wire cap_awvalid,cap_wvalid,cap_wlast,cap_awready,cap_wready,cap_bvalid;
    wire proc_awvalid,proc_wvalid,proc_wlast,proc_awready,proc_wready,proc_bvalid;
    wire [127:0] cap_wdata,proc_wdata;
    // Capture/cache/composer use a separate B state. Register the physical
    // response so even a slave returning B on final W cannot outrun that state.
    reg bvalid_q;
    reg [1:0] bresp_q;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin bvalid_q<=0;bresp_q<=0;end
        else begin bvalid_q<=bvalid;bresp_q<=bresp;end
    Canny_Frame_Capture #(.PIXELS(WIDTH*HEIGHT),.BASE(CAPTURE_BASE),.DYNAMIC_BASE(1),.METADATA_WIDTH(METADATA_WIDTH),.FRAME_GATE(RAW_EXPORT_ENABLE)) capture (
        .clk(clk),.rst_n(rst_n),.start(cap_start),.pclk(pclk),.vs(vs),.de(de),
        .frame_gate(!capture_raw || raw_active),.capture_active(),
        .pixel_diagnostic(fc_pixel_diagnostic),
        .base_address(record_base(next_capture_bank)),
        .record_data({rgb565,median_gray,4'd0,gradient_direction,candidate_class}),
        .frame_metadata(METADATA_ENABLE?frame_metadata:DEFAULT_METADATA),.captured_metadata(captured_metadata),
        .busy(fc_busy),.done(fc_done),.error(fc_error),.frame_started(cap_started),
        .awaddr(fc_awaddr),.awlen(fc_awlen),.awvalid(fc_awvalid),
        .wvalid(fc_wvalid),.wlast(fc_wlast),.wdata(fc_wdata),
        .awready(fc_awready),.wready(fc_wready),.bvalid(fc_bvalid),.bresp(bresp_q));
    generate if(RAW_EXPORT_ENABLE) begin: g_raw
        Canny_Frame_Capture #(.PIXELS(WIDTH*HEIGHT),.DYNAMIC_BASE(1),.METADATA_WIDTH(1),.RAW8(1)) raw_capture (
            .clk(clk),.rst_n(rst_n),.start(cap_start && snapshot_request),.pclk(pclk),.vs(raw_vs),.de(raw_de),
            .frame_gate(1'b1),.capture_active(raw_active),.record_data({24'd0,raw_data}),
            .pixel_diagnostic(rc_pixel_diagnostic),
            .frame_metadata(1'b0),.captured_metadata(),.base_address(RAW_BASE),
            .busy(rc_busy),.done(rc_done),.error(rc_error),.frame_started(),
            .awaddr(rc_awaddr),.awlen(rc_awlen),.awvalid(rc_awvalid),.awready(rc_awready),
            .wvalid(rc_wvalid),.wlast(rc_wlast),.wdata(rc_wdata),.wready(rc_wready),.bvalid(rc_bvalid),.bresp(bresp_q));
        Raw_Frame_Attach #(.PIXELS(WIDTH*HEIGHT),.SLOT_BITS(SLOT_BITS)) attachment (
            .clk(clk),.rst_n(rst_n),.start(attach_start),.source_base(RAW_BASE),
            .slot(write_slot),.header(header_hold),.busy(attach_busy),.done(attach_done),.error(attach_error),
            .araddr(at_araddr),.arlen(at_arlen),.arvalid(at_arvalid),.arready(at_arready),
            .rvalid(at_rvalid),.rready(at_rready),.rlast(rlast),.rdata(rdata),.rresp(rresp),
            .awaddr(at_awaddr),.awlen(at_awlen),.awvalid(at_awvalid),.awready(at_awready),
            .wvalid(at_wvalid),.wlast(at_wlast),.wdata(at_wdata),.wready(at_wready),.bvalid(at_bvalid),.bresp(bresp_q));
        AXI_Write_Arbiter_2 capture_raw_arb (
            .clk(clk),.rst_n(rst_n),
            .p0_addr(fc_awaddr),.p0_len(fc_awlen),.p0_valid(fc_awvalid),.p0_wvalid(fc_wvalid),.p0_wlast(fc_wlast),.p0_data(fc_wdata),
            .p0_ready(fc_awready),.p0_wready(fc_wready),.p0_bvalid(fc_bvalid),
            .p1_addr(rc_awaddr),.p1_len(rc_awlen),.p1_valid(rc_awvalid),.p1_wvalid(rc_wvalid),.p1_wlast(rc_wlast),.p1_data(rc_wdata),
            .p1_ready(rc_awready),.p1_wready(rc_wready),.p1_bvalid(rc_bvalid),
            .awaddr(cap_awaddr),.awlen(cap_awlen),.awvalid(cap_awvalid),.awready(cap_awready),
            .wvalid(cap_wvalid),.wlast(cap_wlast),.wdata(cap_wdata),.wready(cap_wready),.bvalid(cap_bvalid));
        AXI_Write_Arbiter_2 attachment_write_arb (
            .clk(clk),.rst_n(rst_n),
            .p0_addr(joined_awaddr),.p0_len(joined_awlen),.p0_valid(joined_awvalid),.p0_wvalid(joined_wvalid),.p0_wlast(joined_wlast),.p0_data(joined_wdata),
            .p0_ready(joined_awready),.p0_wready(joined_wready),.p0_bvalid(joined_bvalid),
            .p1_addr(at_awaddr),.p1_len(at_awlen),.p1_valid(at_awvalid),.p1_wvalid(at_wvalid),.p1_wlast(at_wlast),.p1_data(at_wdata),
            .p1_ready(at_awready),.p1_wready(at_wready),.p1_bvalid(at_bvalid),
            .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),
            .wvalid(wvalid),.wlast(wlast),.wdata(wdata),.wready(wready),.bvalid(bvalid_q));
        AXI_Read_Arbiter_2 #(.ROUND_ROBIN(1)) attachment_read_arb (
            .clk(clk),.rst_n(rst_n),
            .p0_addr(wr_araddr),.p0_len(wr_arlen),.p0_valid(wr_arvalid),.p0_ready(wr_arready),.p0_rvalid(wr_rvalid),.p0_rready(wr_rready),
            .p1_addr(at_araddr),.p1_len(at_arlen),.p1_valid(at_arvalid),.p1_ready(at_arready),.p1_rvalid(at_rvalid),.p1_rready(at_rready),
            .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rvalid(rvalid),.rlast(rlast),.rready(rready));
    end else begin: g_no_raw
        assign rc_busy=0;assign rc_done=0;assign rc_error=0;assign raw_active=1;
        assign rc_pixel_diagnostic=0;
        assign attach_done=0;assign attach_error=0;assign attach_busy=0;
        assign cap_awaddr=fc_awaddr;assign cap_awlen=fc_awlen;assign cap_awvalid=fc_awvalid;
        assign cap_wvalid=fc_wvalid;assign cap_wlast=fc_wlast;assign cap_wdata=fc_wdata;
        assign fc_awready=cap_awready;assign fc_wready=cap_wready;assign fc_bvalid=cap_bvalid;
        assign awaddr=joined_awaddr;assign awlen=joined_awlen;assign awvalid=joined_awvalid;
        assign wvalid=joined_wvalid;assign wlast=joined_wlast;assign wdata=joined_wdata;
        assign joined_awready=awready;assign joined_wready=wready;assign joined_bvalid=bvalid_q;
        assign araddr=wr_araddr;assign arlen=wr_arlen;assign arvalid=wr_arvalid;assign rready=wr_rready;
        assign wr_arready=arready;assign wr_rvalid=rvalid;
    end endgenerate
    wire [31:0] wa[0:1],ra[0:1];
    wire [7:0] wl[0:1],rl[0:1];
    wire [127:0] wd[0:1];
    wire [1:0] wav,wv,wlasts,war,wr,bv,rav,rar,rv,rr;
    generate for(w=0;w<2;w=w+1) begin: g_worker
        Canny_Frame_Worker #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SLOT_BITS(SLOT_BITS),
            .MIN_COMPONENT_PIXELS(MIN_COMPONENT_PIXELS),.AREA_TRACK_LABELS(AREA_TRACK_LABELS),
            .EDGE_DIAGNOSTIC_ENABLE(EDGE_DIAGNOSTIC_ENABLE),
            .PROFILE_ENABLE(PROFILE_ENABLE),.MEMORY_BURST(MEMORY_BURST),.FAST_BLOCK_NEXT(FAST_BLOCK_NEXT),
            .LOOKAHEAD_RECORD(LOOKAHEAD_RECORD),
            .RESULT_WAIT_ENABLE(RESULT_WAIT_ENABLE),
            .LABEL_BASE(w==0?LABEL_BASE:LABEL_BASE_1),.TABLE_BASE(w==0?TABLE_BASE:TABLE_BASE_1)) worker (
            .clk(clk),.rst_n(rst_n),.start(worker_start[w]),.output_start(worker_output_start[w]),
            .input_base(record_base(next_process_bank)),.mode_request(mode_request),.write_slot(write_slot),
            .completed_mode(worker_modes[w]),.busy(),.output_pending(worker_pending[w]),.processing_error(worker_error[w]),
            .analysis_start(worker_analysis_start[w]),.analysis_valid(worker_analysis_valid[w]),
            .analysis_edge(worker_analysis_edge[w]),.analysis_x(worker_analysis_x[w]),.analysis_y(worker_analysis_y[w]),
            .analysis_ready(analysis_ready && out_busy && out_owner==w),
            .done(worker_done[w]),.frame_complete(worker_publish[w]),.last_cycles(worker_cycles[w]),
            .stage_counts(worker_counts[w]),
            .out_profile_tick(worker_out_tick[w]),.out_profile_ddr(worker_out_ddr[w]),
            .edge_counts(worker_edge_counts[w]),
            .awaddr(wa[w]),.awlen(wl[w]),.awvalid(wav[w]),.awready(war[w]),
            .wvalid(wv[w]),.wlast(wlasts[w]),.wdata(wd[w]),.wready(wr[w]),.bvalid(bv[w]),.bresp(bresp_q),
            .araddr(ra[w]),.arlen(rl[w]),.arvalid(rav[w]),.arready(rar[w]),
            .rvalid(rv[w]),.rready(rr[w]),.rlast(rlast),.rresp(rresp),.rdata(rdata));
    end endgenerate
    AXI_Write_Arbiter_2 worker_write_arb (
        .clk(clk),.rst_n(rst_n),
        .p0_addr(wa[0]),.p0_len(wl[0]),.p0_valid(wav[0]),.p0_wvalid(wv[0]),.p0_wlast(wlasts[0]),.p0_data(wd[0]),
        .p0_ready(war[0]),.p0_wready(wr[0]),.p0_bvalid(bv[0]),
        .p1_addr(wa[1]),.p1_len(wl[1]),.p1_valid(wav[1]),.p1_wvalid(wv[1]),.p1_wlast(wlasts[1]),.p1_data(wd[1]),
        .p1_ready(war[1]),.p1_wready(wr[1]),.p1_bvalid(bv[1]),
        .awaddr(proc_awaddr),.awlen(proc_awlen),.awvalid(proc_awvalid),.awready(proc_awready),
        .wvalid(proc_wvalid),.wlast(proc_wlast),.wdata(proc_wdata),.wready(proc_wready),.bvalid(proc_bvalid));
    AXI_Write_Arbiter_2 capture_write_arb (
        .clk(clk),.rst_n(rst_n),
        .p0_addr(cap_awaddr),.p0_len(cap_awlen),.p0_valid(cap_awvalid),.p0_wvalid(cap_wvalid),.p0_wlast(cap_wlast),.p0_data(cap_wdata),
        .p0_ready(cap_awready),.p0_wready(cap_wready),.p0_bvalid(cap_bvalid),
        .p1_addr(proc_awaddr),.p1_len(proc_awlen),.p1_valid(proc_awvalid),.p1_wvalid(proc_wvalid),.p1_wlast(proc_wlast),.p1_data(proc_wdata),
        .p1_ready(proc_awready),.p1_wready(proc_wready),.p1_bvalid(proc_bvalid),
        .awaddr(joined_awaddr),.awlen(joined_awlen),.awvalid(joined_awvalid),.awready(joined_awready),
        .wvalid(joined_wvalid),.wlast(joined_wlast),.wdata(joined_wdata),.wready(joined_wready),.bvalid(joined_bvalid));
    // Fair between workers; the existing OUTER display-priority arbiter stays.
    AXI_Read_Arbiter_2 #(.ROUND_ROBIN(1)) worker_read_arb (
        .clk(clk),.rst_n(rst_n),
        .p0_addr(ra[0]),.p0_len(rl[0]),.p0_valid(rav[0]),.p0_ready(rar[0]),.p0_rvalid(rv[0]),.p0_rready(rr[0]),
        .p1_addr(ra[1]),.p1_len(rl[1]),.p1_valid(rav[1]),.p1_ready(rar[1]),.p1_rvalid(rv[1]),.p1_rready(rr[1]),
        .araddr(wr_araddr),.arlen(wr_arlen),.arvalid(wr_arvalid),.arready(wr_arready),.rvalid(wr_rvalid),.rlast(rlast),.rready(wr_rready));

    reg capture_started;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) capture_started<=0;
        else if(cap_start) capture_started<=0;
        else if(cap_started) capture_started<=1;
    // IDLE now means neither processing stage holds a token. Waiting for an
    // output token is backpressure, not camera idle.
    wire [2:0] profile_phase=(scan_busy || out_busy)?3'd2:
        (capture_started || cap_started)?3'd1:3'd0;
    generate if(DIAGNOSTIC_ENABLE) begin: g_schedule
        Canny_Schedule_Monitor #(.CLOCK_HZ(CLOCK_HZ)) monitor (
            .clk(clk),.rst_n(rst_n),.vs(vs),.cap_start(cap_start),.cap_started(cap_started),
            .cap_done(cap_done),.cap_error(cap_error),.processing_error(processing_error),
            .publish(frame_complete),.phase(profile_phase),.data_bcd(schedule_bcd),.valid(schedule_valid));
    end else begin: g_no_schedule
        assign schedule_bcd=0;assign schedule_valid=0;
    end endgenerate
    generate if(PROFILE_ENABLE) begin: g_profile
        // WAIT/CAP and SCAN/ROOT/OUT belong to the published input frame.
        // GAP belongs to successive publications. Stages overlap: DO NOT SUM.
        reg [31:0] cap_counts[0:3];
        reg [127:0] bank_counts[0:3],job_counts[0:1];
        reg cap_write_pending,read_pending,write_pending,started;
        reg [31:0] gap,gap_ddr;
        wire cap_ddr=(cap_awvalid && !cap_awready) || (cap_wvalid && !cap_wready) ||
            (cap_write_pending && !cap_bvalid);
        wire bus_ddr=(arvalid && !arready) || (read_pending && rready && !rvalid) ||
            (awvalid && !awready) || (wvalid && !wready) || (write_pending && !bvalid);
        wire cap_phase=capture_started || cap_started;
        wire [127:0] cap_snapshot={cap_counts[3],cap_counts[2],cap_counts[1],cap_counts[0]};
        wire published_worker=RAW_EXPORT_ENABLE?out_owner:worker_publish[1];
        wire [127:0] cc=job_counts[published_worker];
        wire [191:0] sc=worker_counts[published_worker];
        wire [383:0] report_counts={gap_ddr+{31'd0,(started && bus_ddr)},sc[191:96],cc[127:64],
            gap+{31'd0,started},sc[95:0],cc[63:0]};
        integer n;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                for(n=0;n<4;n=n+1) begin cap_counts[n]<=0;bank_counts[n]<=0;end
                job_counts[0]<=0;job_counts[1]<=0;
                cap_write_pending<=0;read_pending<=0;write_pending<=0;started<=0;gap<=0;gap_ddr<=0;
            end else begin
                if(enable) started<=1;
                if(cap_wvalid && cap_wready && cap_wlast) cap_write_pending<=1;
                if(cap_bvalid) cap_write_pending<=0;
                if(arvalid && arready) read_pending<=1;
                if(rvalid && rready && rlast) read_pending<=0;
                if(wvalid && wready && wlast) write_pending<=1;
                if(bvalid) write_pending<=0;
                if(cap_start) for(n=0;n<4;n=n+1) cap_counts[n]<=0;
                else if(cap_busy) begin
                    if(cap_counts[cap_phase]!=32'hffffffff) cap_counts[cap_phase]<=cap_counts[cap_phase]+1'b1;
                    if(cap_ddr && cap_counts[2+cap_phase]!=32'hffffffff)
                        cap_counts[2+cap_phase]<=cap_counts[2+cap_phase]+1'b1;
                end
                if(capture_good) bank_counts[capture_bank]<=cap_snapshot;
                if(take_bank) job_counts[next_worker]<=capture_good?cap_snapshot:bank_counts[next_process_bank];
                if(started) begin
                    if(gap!=32'hffffffff) gap<=gap+1'b1;
                    if(bus_ddr && gap_ddr!=32'hffffffff) gap_ddr<=gap_ddr+1'b1;
                end
                if(frame_complete) begin gap<=0;gap_ddr<=0;end
            end
        end
        Canny_Profiler #(.CLOCK_HZ(CLOCK_HZ),.EXTERNAL_COUNTS(1)) profiler (
            .clk(clk),.rst_n(rst_n),.start(1'b0),.active(1'b0),.publish(frame_complete),.phase(3'd0),
            .external_counts(report_counts),
            .awvalid(1'b0),.awready(1'b0),.wvalid(1'b0),.wready(1'b0),.wlast(1'b0),.bvalid(1'b0),
            .arvalid(1'b0),.arready(1'b0),.rvalid(1'b0),.rready(1'b0),.rlast(1'b0),
            .milliseconds_bcd(profile_ms_bcd),.report_valid(profile_valid));
    end else begin: g_no_profile
        assign profile_ms_bcd=0;assign profile_valid=0;
    end endgenerate
    generate if(PROFILE_ENABLE && OUT_PROFILE_ENABLE) begin: g_out_profile
        // OUT is exclusive, even while the other worker performs SCAN. Count
        // only that owner's phase-3 clocks, reset at every grant (also retries).
        // Registered publication arrives after the final RUN clock is counted.
        wire [31:0] counts[0:9];
        wire [191:0] published_counts=worker_counts[RAW_EXPORT_ENABLE?out_owner:worker_publish[1]];
        wire [383:0] report_counts;
        genvar k;
        for(k=0;k<10;k=k+1) begin: g_count
            Canny_Profile_Counter_DSP counter (
                .clk(clk),.rst_n(rst_n),.clear(grant_output),
                .enable(out_busy && worker_out_tick[out_owner][k%5] &&
                    ((k<5) || worker_out_ddr[out_owner])),.value(counts[k]));
            assign report_counts[(k<5?k:k+1)*32 +: 32]=counts[k];
        end
        // Reuse the existing OUT totals; conservation is checked in simulation.
        assign report_counts[5*32 +: 32]=published_counts[2*32 +: 32];
        assign report_counts[11*32 +: 32]=published_counts[5*32 +: 32];
        Canny_Profiler #(.CLOCK_HZ(CLOCK_HZ),.EXTERNAL_COUNTS(1)) profiler (
            .clk(clk),.rst_n(rst_n),.start(1'b0),.active(1'b0),.publish(frame_complete),.phase(3'd0),
            .external_counts(report_counts),
            .awvalid(1'b0),.awready(1'b0),.wvalid(1'b0),.wready(1'b0),.wlast(1'b0),.bvalid(1'b0),
            .arvalid(1'b0),.arready(1'b0),.rvalid(1'b0),.rready(1'b0),.rlast(1'b0),
            .milliseconds_bcd(out_profile_bcd),.report_valid(out_profile_valid));
    end else begin: g_no_out_profile
        assign out_profile_bcd=0;assign out_profile_valid=0;
    end endgenerate
    generate if(EDGE_DIAGNOSTIC_ENABLE) begin: g_edge_stats
        // Use the publishing worker, never the concurrently scanning owner.
        // Failed/cancelled jobs cannot replace a successful report.
        Canny_Edge_Statistics statistics (
            .clk(clk),.rst_n(rst_n),.publish(frame_complete),
            .counts(worker_edge_counts[RAW_EXPORT_ENABLE?out_owner:worker_publish[1]]),
            .data_bcd(edge_stats_bcd),.valid(edge_stats_valid),.updated());
    end else begin: g_no_edge_stats
        assign edge_stats_bcd=0;assign edge_stats_valid=0;
    end endgenerate
    // synthesis translate_off
    reg [31:0] regions[0:7];
    integer check_a,check_b;
    initial begin
        regions[0]=CAPTURE_BASE;regions[1]=CAPTURE_BASE_1;regions[2]=CAPTURE_BASE_2;regions[3]=CAPTURE_BASE_3;
        regions[4]=LABEL_BASE;regions[5]=TABLE_BASE;regions[6]=LABEL_BASE_1;regions[7]=TABLE_BASE_1;
        if(WIDTH*HEIGHT*4>32'h00400000 || SLOT_BITS>22) $fatal(1,"Pipeline frame exceeds region");
        for(check_a=0;check_a<8;check_a=check_a+1) begin
            if(regions[check_a][21:0]!=0 || regions[check_a]<32'h02000000 || regions[check_a]>32'h03c00000)
                $fatal(1,"Pipeline scratch must use aligned regions above display slots, below 64 MiB");
            for(check_b=0;check_b<check_a;check_b=check_b+1)
                if(regions[check_a]==regions[check_b]) $fatal(1,"Pipeline regions overlap");
        end
    end
    always @(posedge clk) if(rst_n) begin
        if(worker_publish==3) $fatal(1,"Concurrent publications");
        if(occupied==3 && worker_bank[0]==worker_bank[1]) $fatal(1,"Workers share input bank");
    end
    // synthesis translate_on
endmodule
