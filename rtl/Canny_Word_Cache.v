`timescale 1ns/1ps
// Ordered 32-bit scratch port. Default: 64 x 256-byte write-back lines.
// Long-burst mode: 8 x 2048-byte lines, SAME 16 KiB data RAM capacity.
// Region bits [23:22] partition capture/labels/parents. Every burst is aligned
// to their line size (at most 2 KiB), never cross 4 KiB. Private scratch state is discarded only
// between frames. A failed read is drained through RLAST before replying.
module Canny_Word_Cache #(
    parameter integer BLOCK_PORT=0,FAST_HIT=0,LINE_BEATS=16,LINES=1024/LINE_BEATS
)(
    input wire clk,rst_n,invalidate,
    input wire req_valid,req_write,
    input wire [31:0] req_addr,req_wdata,
    // Optional aligned four-word access shares exactly the same ordering and
    // error contract. Parent-table requests continue to use the word port.
    input wire req_block,
    input wire [127:0] req_block_data,
    output wire req_ready,
    output wire rsp_valid,rsp_error,
    output wire [31:0] rsp_data,
    output wire [127:0] rsp_block_data,
    output wire [31:0] awaddr,araddr,
    output wire [7:0] awlen,arlen,
    output wire awvalid,wvalid,wlast,arvalid,rready,
    input wire awready,wready,bvalid,arready,rvalid,rlast,
    input wire [1:0] bresp,rresp,
    output wire [127:0] wdata,
    input wire [127:0] rdata
);
    localparam integer BEAT_BITS=$clog2(LINE_BEATS),INDEX_BITS=$clog2(LINES),OFFSET_BITS=BEAT_BITS+4;
    localparam integer RAM_BITS=BEAT_BITS+INDEX_BITS;
    localparam IDLE=0,LOOKUP=1,CHECK=2,AW=3,WD=4,B=5,AR=6,R=7;
    reg [3:0] state;
    reg [LINES-1:0] valid_bits,dirty_bits;
    reg [127:0] data_ram[0:LINES*LINE_BEATS-1];
    reg [31-OFFSET_BITS:0] tag_ram[0:LINES-1];
    reg [127:0] line_q;
    reg [31-OFFSET_BITS:0] tag_q;
    reg [31:0] address,write_data;
    reg block_access;
    reg [127:0] block_write_data;
    reg writing,read_error,read_overrun;
    reg response_valid,response_error;
    reg [31:0] response_data;
    reg [127:0] response_block_data;
    reg [BEAT_BITS-1:0] beat;
    wire [INDEX_BITS-1:0] index={address[23:22],address[OFFSET_BITS+INDEX_BITS-3:OFFSET_BITS]};
    wire [INDEX_BITS-1:0] incoming_index={req_addr[23:22],req_addr[OFFSET_BITS+INDEX_BITS-3:OFFSET_BITS]};
    wire hit=valid_bits[index] && tag_q==address[31:OFFSET_BITS];
    // BRAM data/tag are synchronous. A hit can be consumed at the CHECK edge
    // itself, removing the extra response-register cycle without async RAM.
    wire fast_response=FAST_HIT && state==CHECK && hit;
    assign rsp_valid=fast_response || response_valid;
    assign rsp_error=response_error;
    assign rsp_data=fast_response?line_q[address[3:2]*32 +: 32]:response_data;
    assign rsp_block_data=fast_response?line_q:response_block_data;
    wire [BEAT_BITS-1:0] burst_next=(state==WD && wready)?beat+1'b1:beat;
    wire [RAM_BITS-1:0] read_index=state==IDLE?{incoming_index,req_addr[OFFSET_BITS-1:4]}:
                          (state==AW || state==WD)?{index,burst_next}:{index,address[OFFSET_BITS-1:4]};
    reg [127:0] updated_line;
    always @(*) begin
        updated_line=line_q;
        updated_line[address[3:2]*32 +: 32]=write_data;
        if(BLOCK_PORT && block_access) updated_line=block_write_data;
    end
    wire hit_write=state==CHECK && hit && writing;
    wire fill_write=state==R && rvalid && !read_overrun;
    wire [RAM_BITS-1:0] ram_write_index=fill_write?{index,beat}:{index,address[OFFSET_BITS-1:4]};
    wire [127:0] ram_write_data=fill_write?rdata:updated_line;
    // Read request address directly on acceptance: cache hits need no separate
    // address-register bubble. Write-back prefetch advances only on W handshake.
    always @(posedge clk) begin
        line_q<=data_ram[read_index];
        tag_q<=tag_ram[state==IDLE?incoming_index:index];
        // One explicit write-address/data mux gives Efinity a single BRAM
        // write port, rather than two independently inferred write ports.
        if(rst_n && (hit_write || fill_write))
            data_ram[ram_write_index]<=ram_write_data;
        if(rst_n && state==R && rvalid && rlast && beat==LINE_BEATS-1 &&
           !read_error && !read_overrun && rresp==0)
            tag_ram[index]<=address[31:OFFSET_BITS];
    end
    assign req_ready=state==IDLE && !invalidate;
    assign awaddr={tag_q,{OFFSET_BITS{1'b0}}};
    assign araddr={address[31:OFFSET_BITS],{OFFSET_BITS{1'b0}}};
    assign awlen=LINE_BEATS-1;
    assign arlen=LINE_BEATS-1;
    assign awvalid=state==AW;
    assign wvalid=state==WD;
    assign wlast=beat==LINE_BEATS-1;
    assign wdata=line_q;
    assign arvalid=state==AR;
    assign rready=state==R;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;valid_bits<=0;dirty_bits<=0;address<=0;write_data<=0;
            writing<=0;response_valid<=0;response_error<=0;response_data<=0;
            block_access<=0;block_write_data<=0;response_block_data<=0;
            beat<=0;read_error<=0;read_overrun<=0;
        end else begin
            response_valid<=0;response_error<=0;
            case(state)
                IDLE: if(invalidate) begin valid_bits<=0;dirty_bits<=0;end
                      else if(req_valid) begin
                          address<=req_addr;write_data<=req_wdata;writing<=req_write;state<=CHECK;
                          block_access<=BLOCK_PORT && req_block;block_write_data<=req_block_data;
                      end
                LOOKUP: state<=CHECK;
                CHECK: if(hit) begin
                    if(writing) dirty_bits[index]<=1;
                    response_data<=line_q[address[3:2]*32 +: 32];response_valid<=!FAST_HIT;state<=IDLE;
                    response_block_data<=line_q;
                end else begin
                    beat<=0;read_error<=0;read_overrun<=0;
                    valid_bits[index]<=0;dirty_bits[index]<=0;
                    if(valid_bits[index] && dirty_bits[index]) state<=AW;
                    else state<=AR;
                end
                AW: if(awready) state<=WD;
                WD: if(wready) begin
                    if(beat==LINE_BEATS-1) begin beat<=0;state<=B;end
                    else beat<=beat+1'b1;
                end
                B: if(bvalid) begin
                    if(bresp!=0) begin response_valid<=1;response_error<=1;state<=IDLE;end
                    else state<=AR;
                end
                AR: if(arready) begin beat<=0;state<=R;end
                R: if(rvalid) begin
                    if(rresp!=0) read_error<=1;
                    if(rlast) begin
                        if(read_error || read_overrun || rresp!=0 || beat!=LINE_BEATS-1) begin
                            response_valid<=1;response_error<=1;state<=IDLE;
                        end else begin valid_bits[index]<=1;state<=LOOKUP;end
                    end else if(beat==LINE_BEATS-1) begin read_overrun<=1;read_error<=1;end
                    else beat<=beat+1'b1;
                end
                default: state<=IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial if(LINE_BEATS<16 || LINE_BEATS>128 || (LINE_BEATS & (LINE_BEATS-1))!=0 ||
               LINES<8 || (LINES & (LINES-1))!=0 || OFFSET_BITS+INDEX_BITS-3>=22)
        $fatal(1,"Cache geometry must be power-of-two and retain four independent regions");
    // synthesis translate_on
endmodule
