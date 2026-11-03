//  by CrazyBird
module Video_Frame_Buffer #(
	parameter EXTERNAL_WRITE = 0,
    parameter MEDIA_ENABLE = 0,
    parameter RAW_PHOTOS = 0, PHOTO_FRAMES = 8, SHAPE_TARGETS = 6,
	parameter 	C_ID_LEN      = 8, 
			C_DATA_LEN    = 128, 
			C_DATA_SIZE   = 4, 	//	C_DATA_LEN=8:0, 16:1, 32:2, 64:3, 128:4, 256:5, 512:6, 1024:7
			
			C_BURST_LEN   = 128,
			
			C_STRB_LEN    = C_DATA_LEN / 8, 
			C_ADDR_INC    = C_BURST_LEN * C_STRB_LEN,
			
			C_BASE_ADDR   = 32'h00000000, 	//	Start at 0x00000000 by default
			C_BUF_SIZE    = 22, 			//	Allocate 2^22 bytes (4MB) by default. The module takes 8*4MB by default.
			C_RD_END_ADDR = 1280 * 720,
			
			C_W_WIDTH     = 64, 
			C_R_WIDTH     = 8
)(
    input  wire                 axi_clk         ,
    input  wire                 axi_reset       ,
    // Optional complete-frame producer. It owns the selected write slot until
    // all DDR writes finish; errors must never pulse external_write_complete.
    input  wire                 external_write_complete,
    output wire [2:0]            external_write_index,
    output wire [2:0]            display_read_index,
    output wire                  display_read_valid,
    input wire [7:0] media_pin_mask,
    input wire media_read_enable,
    input wire [31:0] media_read_base,
    input wire rframe_blanking,
    output wire media_read_active,
    output wire [31:0] media_active_base,
    output wire photo_metadata_valid,
    output wire [19:0] photo_word,
    output wire [4:0] photo_word_index,
    output wire photo_word_valid,
    input wire photo_word_ready,
    
    output       [C_ID_LEN-1:0]      axi_awid        ,
    output       [31:0]      axi_awaddr      ,
    output       [ 7:0]      axi_awlen       ,
    output       [ 2:0]      axi_awsize      ,
    output       [ 1:0]      axi_awburst     ,
    output                   axi_awlock      ,
    output       [ 3:0]      axi_awcache     ,
    output       [ 2:0]      axi_awprot      ,
    output       [ 3:0]      axi_awqos       ,
    output reg                  axi_awvalid     ,
    input  wire                 axi_awready     ,
    
    output       [C_DATA_LEN-1:0]      axi_wdata       ,
    output       [C_STRB_LEN-1:0]      axi_wstrb       ,
    output                   axi_wlast       ,
    output reg                  axi_wvalid      ,
    input  wire                 axi_wready      ,
    
    input  wire     [C_ID_LEN-1:0]      axi_bid         ,
    input  wire     [ 1:0]      axi_bresp       ,
    input  wire                 axi_bvalid      ,
    output                   axi_bready      ,
    
    output       [C_ID_LEN-1:0]      axi_arid        ,
    output       [31:0]      axi_araddr      ,
    output       [ 7:0]      axi_arlen       ,
    output       [ 2:0]      axi_arsize      ,
    output       [ 1:0]      axi_arburst     ,
    output                   axi_arlock      ,
    output       [ 3:0]      axi_arcache     ,
    output       [ 2:0]      axi_arprot      ,
    output       [ 3:0]      axi_arqos       ,
    output reg                  axi_arvalid     ,
    input  wire                 axi_arready     ,
    
    input  wire     [C_ID_LEN-1:0]      axi_rid         ,
    input  wire     [C_DATA_LEN-1:0]      axi_rdata       ,
    input  wire     [ 1:0]      axi_rresp       ,
    input  wire                 axi_rlast       ,
    input  wire                 axi_rvalid      ,
    output                   axi_rready      ,
    
    input  wire                 wframe_pclk     ,
    input  wire                 wframe_vsync    ,		//	Writter VSync. Flush on falling edge. Connect to EOF. 
    input  wire                 wframe_data_en  ,
    input  wire     [C_W_WIDTH-1:0]      wframe_data     ,
    
    input  wire                 rframe_pclk     ,
    input  wire                 rframe_vsync    ,		//	Reader VSync. Flush on falling edge. Connect to ~EOF. 
    input  wire                 rframe_data_en  ,
    output wire     [C_R_WIDTH-1:0]      rframe_data     ,
    
    output 		[31:0] 	tp_o
);
	
	initial begin
		axi_awvalid <= 0; 
		axi_wvalid <= 0; 
	end

	assign axi_awid    = {C_ID_LEN{1'b0}};
	assign axi_awlen   = C_BURST_LEN - 1'b1;
	assign axi_awsize  = C_DATA_SIZE;
	assign axi_awburst = 2'b01;	//	INCR
	assign axi_awlock  = 1'b0;
	assign axi_awcache = 0;
	assign axi_awprot  = 3'b0;
	assign axi_awqos   = 4'b0;
	assign axi_wstrb   = 32'hFFFFFFFF;
	assign axi_bready  = 1; 
	    
	assign axi_arid    = {C_ID_LEN{1'b0}};
    wire [7:0] media_reader_arlen;
	assign axi_arlen   = MEDIA_ENABLE ? media_reader_arlen : C_BURST_LEN - 1'b1;
	assign axi_arsize  = C_DATA_SIZE;
	assign axi_arburst = 2'b01;	//	INCR
	assign axi_arlock  = 1'b0;
	assign axi_arcache = 0;
	assign axi_arprot  = 3'b0;
	assign axi_arqos   = 4'b0;

	
	
	
	
	////////////////////////////////////////////////////////////////
	//	R/W Schedule
    wire [2:0] rc_wframe_index, rc_rframe_index;
    wire read_frame_valid;
    reg r_wframe_inc = 0, r_rframe_inc = 0;
    Frame_Buffer_Control #(.MEDIA_LEASE_ENABLE(MEDIA_ENABLE)) frame_control (
        .clk(axi_clk), .reset(axi_reset),
        .write_complete(EXTERNAL_WRITE ? external_write_complete : r_wframe_inc), .read_boundary(r_rframe_inc),
        .write_index(rc_wframe_index),
        .read_index(rc_rframe_index), .read_valid(read_frame_valid),
        .media_pin_mask(media_pin_mask)
    );
    assign external_write_index = rc_wframe_index;
    assign display_read_index = rc_rframe_index;
    assign display_read_valid = read_frame_valid;

	////////////////////////////////////////////////////////////////
	//	AXI Writter

	wire 				w_wfifo_pempty, w_wfifo_empty; 
	wire 				w_wfifo_ren; 
	wire 	[C_DATA_LEN-1:0] 	w_wfifo_rdata; 

	//	On EOF when ~empty, flush the last data then reset FIFO. 
	reg 	[3:0] 	rs_w = 0; 
	wire 	[3:0] 	ws_w_idle = 0; 
	wire 	[3:0] 	ws_w_wdata = 1; 
	wire 	[3:0] 	ws_w_winc = 2; 
	wire 	[3:0] 	ws_w_eof = 3; 

	//	Write Pointer. Burst bytes is always C_ADDR_INC. 
	reg 	[C_BUF_SIZE-1:0] 	rc_w_ptr = 0; 
	reg 	[0:0] 		rc_w_eof = 0; 
	reg 				r_w_rst = 1; 
	
	//	Burst Control
	reg 	[7:0] 	rc_burst = 0; 	//	Compare to C_BURST_LEN - 1. 
	
	//	EOF Monitor. 
	reg 	[1:0] 	r_wframe_sync = 0; 	
	reg 			r_weof_pending = 0;
    reg [4:0] eof_settle=0;
    reg write_response_done = 0;
    reg frame_write_error = 0;

    // One burst outstanding: keep its address until DDR acknowledges the write.
    // Error/underflow discards this frame and reuses its slot, preserving the
    // last complete frame. The configured frame size is a whole burst multiple.
    always @(posedge axi_clk or posedge axi_reset) begin
        if (axi_reset) begin
            write_response_done <= 0;
            frame_write_error <= 0;
        end else begin
            if (rs_w == ws_w_idle) write_response_done <= 0;
            if (axi_bvalid && axi_bready) begin
                write_response_done <= 1;
                if (axi_bresp != 2'b00) frame_write_error <= 1;
            end
            if (w_wfifo_ren && w_wfifo_empty) frame_write_error <= 1;
            if (rs_w == ws_w_eof) frame_write_error <= 0;
        end
    end

	always @(posedge axi_clk or posedge axi_reset) begin
		if(axi_reset) begin
			rs_w <= 0; 
			rc_w_ptr <= 0; 
			rc_w_eof <= 0; 
			r_w_rst <= 1; 
			axi_awvalid <= 0; 
			axi_wvalid <= 0; 
			r_wframe_sync <= 0; 
			r_weof_pending <= 0;
            eof_settle<=0;
			r_wframe_inc <= 0; 
			
		end else begin
			rc_w_eof <= 0; 
			r_wframe_inc <= 0; 
			
			//	Raise EOF on falling edge of VSYNC. 
			r_wframe_sync <= {r_wframe_sync, wframe_vsync}; 
            if (r_wframe_sync==2'b10) eof_settle<=5'd16;
            else if (eof_settle!=0) begin
                eof_settle<=eof_settle-1'b1;
                if (eof_settle==1) r_weof_pending<=1;
            end
			
			if(axi_awready) begin
				axi_awvalid <= 0; 
			end else begin
			end
			
			case (rs_w)
				ws_w_idle: begin
						rc_burst <= 0; 
						r_w_rst <= 0; 
						
						if(~w_wfifo_pempty) begin
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
						
						end else if(r_weof_pending && (~w_wfifo_empty)) begin
							//	There's some data in the FIFO. write last. 
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
							
						end else if(r_weof_pending) begin
							//	EOF. Reset FIFO. Increment write pointer. 
							r_w_rst <= 1; 
							r_wframe_inc <= (rc_w_ptr == C_RD_END_ADDR) && !frame_write_error;
							rs_w <= ws_w_eof; 
							
						end else begin
						end
					end
					
				ws_w_eof: begin
						//	Wait for some cycles to release reset. 
						r_weof_pending <= 0; 
						rc_w_ptr <= 0; 
						
						rc_w_eof <= rc_w_eof + 1; 
						if(&rc_w_eof)
							rs_w <= ws_w_idle; 
						else begin
						end
					end
					
				ws_w_wdata: begin
						rc_burst <= rc_burst + axi_wready; 
						
						//	On last transaction terminate write. 
						if(axi_wlast && axi_wready) begin
							axi_wvalid <= 0; 
							rs_w <= ws_w_winc; 
						end else begin
						end
					end
					
				ws_w_winc: begin
						if (write_response_done && !axi_awvalid) begin
                            rc_w_ptr <= rc_w_ptr + C_ADDR_INC;
                            rs_w <= ws_w_idle;
                        end
					end
			endcase
		end
	end
	assign axi_awaddr = C_BASE_ADDR + {rc_wframe_index, rc_w_ptr}; 
	assign axi_wdata = w_wfifo_rdata; 
	assign axi_wlast = (rc_burst >= C_BURST_LEN - 1); 
	
	assign w_wfifo_ren = axi_wvalid && axi_wready; 
	
	
	assign tp_o[1:0] = {rs_w[1:0]}; 
	assign tp_o[2] = axi_wvalid; 
	assign tp_o[3] = wframe_data_en; 
	
	
	generate
		if(C_W_WIDTH == 64) begin: Gen_FIFO_W64
			W0_FIFO_64 u_W0_FIFO_64 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 32) begin: Gen_FIFO_W32
			W0_FIFO_32 u_W0_FIFO_32
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 16) begin: Gen_FIFO_W16
            // Existing IP is 32 -> 128, big-endian. First pixel goes high
            // so R0_FIFO_16 (128 -> 16, big-endian) returns pixels in order.
            reg first_valid;
            reg [15:0] first_pixel;
            reg [31:0] pair_data;
            reg pair_en;
            always @(posedge wframe_pclk or posedge r_w_rst) begin
                if (r_w_rst) begin first_valid<=0; first_pixel<=0; pair_data<=0; pair_en<=0; end
                else begin
                    pair_en<=0;
                    if (wframe_data_en) begin
                        if (!first_valid) begin first_pixel<=wframe_data; first_valid<=1; end
                        else begin pair_data<={first_pixel,wframe_data}; pair_en<=1; first_valid<=0; end
                    end
                end
            end
            W0_FIFO_32 u_W0_FIFO_RGB565 (
                .wr_clk_i(wframe_pclk),.wr_en_i(pair_en),.wdata(pair_data),
                .a_rst_i(r_w_rst),.rd_clk_i(axi_clk),.rd_en_i(w_wfifo_ren),
                .rdata(w_wfifo_rdata),.prog_empty_o(w_wfifo_pempty),.empty_o(w_wfifo_empty)
            );
		end else if(C_W_WIDTH == 8) begin: Gen_FIFO_W8
			W0_FIFO_8 u_W0_FIFO_8 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end
	endgenerate




















reg                             rframe_vsync_dly;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rframe_vsync_dly <= 1'b0;
    else
        rframe_vsync_dly <= rframe_vsync;
end

wire                            rframe_vsync_neg;
assign rframe_vsync_neg =  rframe_vsync_dly & ~rframe_vsync;

reg             [4:0]           rfifo_cnt;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rfifo_cnt <= 5'h1f;
    else
    begin
        if(rframe_vsync_neg == 1'b1)
            rfifo_cnt <= 5'h1f;
        else if(rfifo_cnt > 5'd0)
            rfifo_cnt <= rfifo_cnt - 1'b1;
        else
            rfifo_cnt <= 5'b0;
    end
end

reg                             rfifo_rst;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rfifo_rst <= 1'b1;
    else
    begin
        if(rfifo_cnt == 5'b0)
            rfifo_rst <= 1'b0;
        else
            rfifo_rst <= 1'b1;
    end
end


wire 			w_rfifo_rst = axi_reset || rfifo_rst; 

reg                             rfifo_wenb;
reg             [C_DATA_LEN-1:0]          rfifo_wdata;
wire            [ 9:0]          rfifo_wcnt;
wire 					  rfifo_wfull; 
wire                            rfifo_wr_rst_busy;
wire 					w_rfifo_aempty; 
wire 					w_rfifo_empty; 


// Both live and media pixels are RGB565. Share the original FIFO and
// consume one 16-bit word per pixel request; DDR bursts remain continuous.
wire [C_R_WIDTH-1:0] fifo_pixel_data;
wire fifo_pixel_pop=rframe_data_en;
assign rframe_data=fifo_pixel_data;

generate
	if(C_R_WIDTH == 8) begin: Gen_RFIFO_8
		R0_FIFO_8 u_R0_FIFO_8
		(
			.wr_clk_i		(axi_clk),
			.wr_en_i		(rfifo_wenb),
			.wdata			(rfifo_wdata),
			.a_rst_i		(w_rfifo_rst), 
			.prog_full_o		(rfifo_wfull),
			
			.rd_clk_i		(rframe_pclk),
			.rd_en_i		(fifo_pixel_pop),
			.rdata			(fifo_pixel_data),
			.empty_o		(w_rfifo_empty),
			.almost_empty_o	(w_rfifo_aempty)
		);
	end else if(C_R_WIDTH == 16) begin: Gen_RFIFO_16	
		R0_FIFO_16 u_R0_FIFO_16
		(
			.wr_clk_i		(axi_clk),
			.wr_en_i		(rfifo_wenb),
			.wdata			(rfifo_wdata),
			.a_rst_i		(w_rfifo_rst), 
			.prog_full_o		(rfifo_wfull),
			
			.rd_clk_i		(rframe_pclk),
			.rd_en_i		(fifo_pixel_pop),
			.rdata			(fifo_pixel_data),
			.empty_o		(w_rfifo_empty),
			.almost_empty_o	(w_rfifo_aempty)
		);
	end
endgenerate
		
reg r_rfifo_rst = 0; 
always @(posedge axi_clk)
begin
	r_rfifo_rst <= w_rfifo_rst; 
end

//assign tp_o[7:0] = {w_rfifo_empty, rfifo_wfull, rframe_data_en, rfifo_wenb, rfifo_rst, axi_reset, rframe_pclk, axi_clk}; 

reg             [15:0]               rfifo_wr_rst_busy_dly = 2'b0;

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        rfifo_wr_rst_busy_dly <= 0;
    else
        rfifo_wr_rst_busy_dly <= {rfifo_wr_rst_busy_dly,1'b1};
end

wire                                rfifo_wr_rst_busy_neg;
assign rfifo_wr_rst_busy_neg = (rfifo_wr_rst_busy_dly[1:0] == 2'b01) ? 1'b1 : 1'b0;

reg             [4:0]               read_ddr_delay_cnt;

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        read_ddr_delay_cnt <= 5'b0;
    else
    begin
        if(rfifo_wr_rst_busy_neg == 1'b1)
            read_ddr_delay_cnt <= 5'd31;
        else if(read_ddr_delay_cnt > 5'b0)
            read_ddr_delay_cnt <= read_ddr_delay_cnt - 1'b1;
        else
            read_ddr_delay_cnt <= 5'b0;
    end
end

always @(posedge axi_clk or posedge axi_reset) begin
	if(axi_reset)
		r_rframe_inc <= 0; 
	else
		r_rframe_inc <= rfifo_wr_rst_busy_neg; 
end


wire                                read_ddr_init_flag;
assign read_ddr_init_flag = (read_ddr_delay_cnt == 5'd1) ? 1'b1 : 1'b0;

//----------------------------------------------------------------------




//----------------------------------------------------------------------
generate if(MEDIA_ENABLE) begin: g_media_reader
    (* async_reg = "true" *) reg [1:0] blank_sync;
    always @(posedge axi_clk or posedge axi_reset)
        if(axi_reset) blank_sync<=0;else blank_sync<={blank_sync[0],rframe_blanking};
    wire push,request_valid;
    wire [127:0] data;
    Media_Display_Read #(.PIXELS(C_RD_END_ADDR/2),.BEATS(C_BURST_LEN),
        .RAW_PHOTOS(RAW_PHOTOS),.PHOTO_FRAMES(PHOTO_FRAMES),.TARGETS(SHAPE_TARGETS)) reader (
        .clk(axi_clk),.rst_n(!axi_reset),.fifo_reset(r_rfifo_rst),.boundary(r_rframe_inc),
        .blanking(blank_sync[1]),.fifo_full(rfifo_wfull),.live_valid(read_frame_valid),
        .live_base(C_BASE_ADDR+{rc_rframe_index,{C_BUF_SIZE{1'b0}}}),
        .media_enable(media_read_enable),.media_base(media_read_base),
        .media_active(media_read_active),.active_base(media_active_base),.fifo_push(push),.fifo_data(data),
        .photo_metadata_valid(photo_metadata_valid),.photo_word(photo_word),
        .photo_word_index(photo_word_index),.photo_word_valid(photo_word_valid),.photo_word_ready(photo_word_ready),
        .araddr(axi_araddr),.arlen(media_reader_arlen),.arvalid(request_valid),.rready(axi_rready),
        .arready(axi_arready),.rvalid(axi_rvalid),.rlast(axi_rlast),.rdata(axi_rdata),.rresp(axi_rresp));
    always @(*) begin axi_arvalid=request_valid;rfifo_wenb=push;rfifo_wdata=data;end
end else begin: g_legacy_reader
    assign media_read_active=1'b0;
    assign media_active_base=0;
    assign photo_metadata_valid=0;
    assign photo_word=0;
    assign photo_word_index=0;
    assign photo_word_valid=0;
    assign media_reader_arlen=0;
localparam S_READ_IDLE = 2'd0;
localparam S_READ_ADDR = 2'd1;
localparam S_READ_DATA = 2'd2;

reg             [ 1:0]          rd_state;
reg             [ 8:0]          rdata_cnt;
reg             [C_BUF_SIZE-1:0]          araddr;
reg 					r_rd_pend = 0; 


always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst) begin
        rd_state <= S_READ_IDLE;
	  axi_arvalid <= 0; 
	  r_rd_pend <= 0; 
    end else
    begin
	
	
	
	if(axi_arready) begin
		axi_arvalid <= 0; 
	end else begin
	end
	if(axi_rvalid && axi_rlast) begin
		 r_rd_pend <= 0; 
	end else begin
	end
	
        case(rd_state)
            S_READ_IDLE : 
            begin
			//	Continue if RFIFO ~Full. 
                if(read_frame_valid && rfifo_wr_rst_busy_dly[15] && (araddr < C_RD_END_ADDR)&&(~rfifo_wfull)) begin	//	rfifo_wcnt < 10'd256))
                    rd_state <= S_READ_ADDR;
                end else
                    rd_state <= S_READ_IDLE;
            end
		
            S_READ_ADDR : 
            begin
				axi_arvalid <= 1; 
				r_rd_pend <= 1; 
                rd_state <= S_READ_DATA;
            end
            S_READ_DATA : 
            begin
			//	Return when ~arvalid && ~rd_pend. 
			if((~axi_arvalid) && (~r_rd_pend))
                //if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1)&&(rdata_cnt == C_BURST_LEN))
                    rd_state <= S_READ_IDLE;
                else
                    rd_state <= S_READ_DATA;
            end
            default : 
            begin
                rd_state <= S_READ_IDLE;
            end
        endcase
    end
end

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        araddr <= 0;
    else
    begin
        if((axi_arvalid == 1'b1)&&(axi_arready == 1'b1))
            araddr <= araddr + C_ADDR_INC;
        else
            araddr <= araddr;
    end
end

assign axi_araddr = C_BASE_ADDR + {rc_rframe_index, araddr};


reg             [8:0]           rdata_cnt_dly;

always @(posedge axi_clk)
begin
    rdata_cnt_dly <= rdata_cnt;
end

always @(*)
begin
    if(rd_state == S_READ_DATA)
        begin
        if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1))
            rdata_cnt = rdata_cnt_dly + 1'b1;
        else
            rdata_cnt = rdata_cnt_dly;
        end
    else
        rdata_cnt = 9'b0;
end

assign axi_rready = 1; 

always @(posedge axi_clk)
begin
    if(axi_reset == 1'b1)
        rfifo_wenb <= 1'b0;
    else
    begin
        if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1))
            rfifo_wenb <= 1'b1;
        else
            rfifo_wenb <= 1'b0;
    end
end

always @(posedge axi_clk)
begin
    rfifo_wdata <= axi_rdata;
end

end endgenerate
endmodule
