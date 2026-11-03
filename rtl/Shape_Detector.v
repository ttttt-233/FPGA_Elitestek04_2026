`timescale 1ns/1ps
// Full-frame bounded-work contour classifier; no extra DDR transactions.
// OR-pooled final edges -> independent 8-connected contours -> 16-angle models.
// Unknown/clipped/oversized/timeout inputs produce NONE, never a circle fallback.
// One to eight current-frame results. start aborts unfinished analysis.
module Shape_Detector #(
    parameter integer WIDTH=1280, HEIGHT=720,
    parameter integer SAMPLE_SHIFT=2, MIN_SIZE=24, MAX_POINTS=4096,
    parameter integer MAX_ANALYSIS_CYCLES=5000000, TARGETS=1, TRIANGLE_ENABLE=0,
    // CNN work budget is independent of final slots so later better targets
    // can replace early winners. Preserve the prototype's eight-call bound.
    parameter integer DIGIT_ENABLE=0, MAX_DIGIT_CANDIDATES=8, DIGIT_DEBUG_ENABLE=0,
    parameter integer POINT_PIPELINE=1
)(
    input wire clk,rst_n,start,pixel_valid,pixel_edge,
    input wire [11:0] pixel_x,pixel_y,
    output reg result_valid,
    output reg [49:0] result,
    output reg [49:0] result_second,
    output reg [1:0] result_quality,second_quality,
    output reg [TARGETS*51-1:0] results,
    output reg [TARGETS*2-1:0] qualities,
    output reg [TARGETS*4-1:0] digits,
    input wire debug_enable,debug_ready,
    input wire [3:0] debug_select,
    input wire [11:0] debug_threshold,
    output wire debug_valid,debug_bit,debug_last,
    output wire [9:0] debug_address,
    output reg [207:0] debug_metadata
);
    localparam SAMPLE=1<<SAMPLE_SHIFT;
    localparam CW=(WIDTH+SAMPLE-1)/SAMPLE, CH=(HEIGHT+SAMPLE-1)/SAMPLE;
    localparam XB=(CW<2)?1:$clog2(CW), YB=(CH<2)?1:$clog2(CH);
    localparam GB=(CW*CH<2)?1:$clog2(CW*CH), QB=$clog2(MAX_POINTS);
    localparam PB=$clog2(CW+CH)+10, MB=PB+27;
    localparam MERGE_SLOTS=4;
    localparam IDLE=0,STREAM=1,SCAN_READ=2,SCAN_TEST=3,SCAN_NEXT=4,
        QUEUE_READ=5,QUEUE_POP=6,NEIGHBOR=7,NEIGHBOR_READ=8,NEIGHBOR_TEST=9,
        NEIGHBOR_NEXT=10,COMPONENT=11,ANGLE_START=12,POINT_READ=13,
        POINT_MULT=14,POINT_SUM=15,POINT_USE=16,SPAN=17,SPAN_CHECK=18,
        DIV_START=19,DIVIDE=20,TOLERANCE=21,NORMAL_DELTA=22,NORMAL_MULT=23,
        NORMAL_SHIFT=24,RADIUS=25,SCORE=26,DECIDE=27,ACCEPT=28,TRI_START=29,TRI_WAIT=30,
        POINT_STREAM=31,BANK_WAIT=32,WINDOW_SCAN=33,MERGE_SCAN=34,
        NORM_INIT=35,NORM_READ=36,NORM_USE=37,NORM_XSTEP=38,NORM_YSTEP=39,
        CNN_START=40,CNN_WAIT=41,FINAL_ACCEPT=42,FINAL_WAIT=43,PUBLISH=44,
        SUPPRESS_SCAN=45,WINDOW_APPEND=46,WINDOW_REMOVE=47,WINDOW_FLUSH=48,NORM_CLEAR=49,
        DEBUG_WAIT=50,PARENT_SCAN=51,WINDOW_PARENT_SCAN=52;
    reg [5:0] state;
    reg [31:0] elapsed;
    // Every completed raster overwrites every cell; no frame-RAM reset required.
    reg [CW-1:0] row_acc;
    reg group_acc;
    wire [XB-1:0] input_x=pixel_x>>SAMPLE_SHIFT;
    wire [YB-1:0] input_y=pixel_y>>SAMPLE_SHIFT;
    wire group_end=((pixel_x&(SAMPLE-1))==SAMPLE-1) || pixel_x==WIDTH-1;
    wire row_end=((pixel_y&(SAMPLE-1))==SAMPLE-1) || pixel_y==HEIGHT-1;
    wire pooled=row_acc[input_x] | group_acc | pixel_edge;
    reg grid[0:CW*CH-1];
    reg visited[0:CW*CH-1];
    reg grid_q,visited_q;
    reg [GB-1:0] neighbor_address;
    reg [XB-1:0] scan_x,nx,point_x,left,right;
    reg [YB-1:0] scan_y,ny,point_y,top,bottom;
    reg [XB-1:0] norm_x,norm_right;
    reg [YB-1:0] norm_y,norm_bottom;
    wire seed=state==SCAN_TEST && grid_q && !visited_q;
    wire neighbor_hit=state==NEIGHBOR_TEST && grid_q && !visited_q;
    wire grid_write=state==STREAM && pixel_valid && group_end && row_end;
    wire visited_write=grid_write || seed || neighbor_hit;
    wire [GB-1:0] grid_address=state==STREAM ? input_y*CW+input_x :
        (state==NORM_READ || state==NORM_USE || state==NORM_XSTEP || state==NORM_YSTEP) ?
        norm_y*CW+norm_x :
        (state==SCAN_READ || state==SCAN_TEST) ? scan_y*CW+scan_x : neighbor_address;
    always @(posedge clk) begin
        if(grid_write) grid[grid_address] <= pooled;
        if(visited_write) visited[grid_address]<=!grid_write;
        grid_q<=grid[grid_address];
        visited_q<=visited[grid_address];
    end
    // Ring FIFO drains oversized contours but discards their classification.
    // For contours <= MAX_POINTS all points remain intact for model scoring.
    reg [XB+YB-1:0] points[0:MAX_POINTS-1];
    reg [XB+YB-1:0] point_q;
    reg [QB-1:0] head,tail,index;
    reg [QB:0] queued,count;
    reg oversized;
    wire queue_overflow=neighbor_hit && queued==MAX_POINTS;
    wire point_write=seed || (neighbor_hit && !queue_overflow);
    wire [QB-1:0] point_write_address=seed ? {QB{1'b0}} : tail;
    wire [XB+YB-1:0] point_write_data=seed ? {scan_y,scan_x} : {ny,nx};
    wire [QB-1:0] triangle_address;
    // The triangle reader shares point RAM only during its analysis states.
    wire [QB-1:0] point_read_address=(state==TRI_START || state==TRI_WAIT)?triangle_address:
        (state==QUEUE_READ || state==QUEUE_POP)?head:index;
    always @(posedge clk) begin
        if(point_write) points[point_write_address]<=point_write_data;
        point_q<=points[point_read_address];
    end
    reg [2:0] neighbor_index;
    reg signed [XB+1:0] next_x;
    reg signed [YB+1:0] next_y;
    always @(*) begin
        next_x=$signed({1'b0,point_x});next_y=$signed({1'b0,point_y});
        case(neighbor_index)
            0:begin next_x=next_x-1;next_y=next_y-1;end
            1:next_y=next_y-1;
            2:begin next_x=next_x+1;next_y=next_y-1;end
            3:next_x=next_x-1;
            4:next_x=next_x+1;
            5:begin next_x=next_x-1;next_y=next_y+1;end
            6:next_y=next_y+1;
            7:begin next_x=next_x+1;next_y=next_y+1;end
        endcase
    end
    function [8:0] cosine;
        input [3:0] a;
        begin case(a)
            0:cosine=256;1:cosine=255;2:cosine=251;3:cosine=245;
            4:cosine=237;5:cosine=226;6:cosine=213;7:cosine=198;
            8:cosine=181;9:cosine=162;10:cosine=142;11:cosine=121;
            12:cosine=98;13:cosine=74;14:cosine=50;15:cosine=25;
        endcase end
    endfunction
    function [8:0] sine;
        input [3:0] a;
        begin case(a)
            0:sine=0;1:sine=25;2:sine=50;3:sine=74;
            4:sine=98;5:sine=121;6:sine=142;7:sine=162;
            8:sine=181;9:sine=198;10:sine=213;11:sine=226;
            12:sine=237;13:sine=245;14:sine=251;15:sine=255;
        endcase end
    endfunction
    function [2:0] pop4;
        input [3:0] value;
        begin pop4={2'b0,value[0]}+{2'b0,value[1]}+{2'b0,value[2]}+{2'b0,value[3]};end
    endfunction
    function [4:0] pop16;
        input [15:0] value;
        begin pop16={2'b0,pop4(value[3:0])}+{2'b0,pop4(value[7:4])}+
                    {2'b0,pop4(value[11:8])}+{2'b0,pop4(value[15:12])};end
    endfunction
    reg [3:0] angle;
    reg scoring,ellipse_seen,straight_seen;
    reg [1:0] component_kind;
    reg [1:0] component_quality,ellipse_quality;
    reg signed [PB-1:0] xc,xs,yc,ys,u,v,lo_u,hi_u,lo_v,hi_v;
    reg [PB-1:0] du,dv;
    reg signed [PB+1:0] delta_u,delta_v;
    reg signed [MB-1:0] product_u,product_v;
    reg signed [9:0] norm_u,norm_v;
    wire [8:0] abs_u=norm_u<0 ? -norm_u : norm_u;
    wire [8:0] abs_v=norm_v<0 ? -norm_v : norm_v;
    // Exact floor(2^24/span), shared restoring divider, no division operator.
    reg div_axis;
    reg [4:0] div_bit;
    reg [24:0] quotient,inv_u,inv_v;
    reg [PB:0] remainder;
    reg [PB-1:0] divisor;
    wire [PB:0] shifted_remainder={remainder[PB-1:0],div_bit==24};
    wire subtract=shifted_remainder>={1'b0,divisor};
    reg [8:0] tol_u,tol_v,strict_u,strict_v;
    reg [16:0] radius_tolerance;
    reg [18:0] radius;
    wire near_u=abs_u>=9'd256-tol_u,near_v=abs_v>=9'd256-tol_v;
    wire straight_point=abs_u>=9'd256-strict_u || abs_v>=9'd256-strict_v;
    wire on_ellipse=radius>=19'd65536-radius_tolerance && radius<=19'd65536+radius_tolerance;
    wire signed [10:0] shifted_u=$signed(norm_u)+11'sd256;
    wire signed [10:0] shifted_v=$signed(norm_v)+11'sd256;
    wire [1:0] bin_u=shifted_u>=512?2'd3:shifted_u<0?2'd0:shifted_u[8:7];
    wire [1:0] bin_v=shifted_v>=512?2'd3:shifted_v<0?2'd0:shifted_v[8:7];
    wire [3:0] u_bin={1'b0,!norm_u[9],bin_v};
    wire [3:0] v_bin={1'b1,!norm_v[9],bin_u};
    wire [1:0] corner_index={norm_v[9],norm_u[9]};
    wire [1:0] quadrant_bin=(abs_v*256<abs_u*106)?0:
        (abs_v<abs_u)?1:(abs_v*106<abs_u*256)?2:3;
    wire [3:0] sector=norm_v[9] ? (norm_u[9]?4'd8+quadrant_bin:4'd15-quadrant_bin) :
        (norm_u[9]?4'd7-quadrant_bin:{2'b0,quadrant_bin});
    reg [QB:0] rect_hits,ellipse_hits,straight_hits;
    reg [15:0] rect_mask,ellipse_mask;
    reg [3:0] corners;
    // One synchronous RAM read per clock, followed by the existing arithmetic
    // registers. Valid/last tags drain the pass before using its reductions.
    // No second point RAM, approximation or change of fixed-point widths.
    reg point_issue;
    reg [6:0] point_valid,point_last;
    reg point_rect,point_straight;
    reg [15:0] point_rect_mask;
    reg [3:0] point_corners,point_sector;
    wire rectangle_ok=rect_hits*100>=count*80 && corners==15 &&
        pop4(rect_mask[3:0])>=3 && pop4(rect_mask[7:4])>=3 &&
        pop4(rect_mask[11:8])>=3 && pop4(rect_mask[15:12])>=3;
    wire ellipse_ok=ellipse_hits*100>=count*80 && pop16(ellipse_mask)>=14 && corners!=15;
    wire rectangle_like=straight_hits*100>=count*85 && pop4(corners)>=2;
    // Ordinal quality: hit fraction plus complete side/sector coverage.
    wire [1:0] rect_quality=rect_hits*100>=count*95 && rect_mask==16'hffff ? 3:
        rect_hits*100>=count*90?2:1;
    wire [1:0] ellipse_grade=ellipse_hits*100>=count*95 && ellipse_mask==16'hffff ? 3:
        ellipse_hits*100>=count*90?2:1;
    reg [31:0] candidate_area,best_area;
    wire [2:0] window_count;
    wire [47:0] window_head_box,window_read_box;
    wire [1:0] window_head_geometry,window_head_grade,window_read_geometry,window_read_grade;
    wire [19:0] window_read_area;
    reg [2:0] merge_index,shift_index;
    reg [2:0] parent_index,active_parent_index;
    wire [TARGETS*51-1:0] selected_results;
    reg [5:0] remove_return;
    reg [47:0] pending_box;
    reg [1:0] pending_geometry,pending_grade;
    reg [19:0] pending_area;
    reg pending_valid,scan_done;
    reg [3:0] cnn_attempts;
    reg [50:0] merge_box;
    reg [1:0] merge_grade,final_grade;
    reg [2:0] merge_geometry,final_kind;
    reg [3:0] final_digit;
    reg [XB:0] norm_width,norm_remainder_x;
    reg [YB:0] norm_height,norm_remainder_y;
    reg [4:0] norm_bin_x,norm_bin_y;
    reg [9:0] norm_clear_address;
    wire cnn_busy,cnn_done,cnn_digit_valid;
    wire [3:0] cnn_digit;
    wire [1:0] cnn_confidence;
    wire [21:0] cnn_cycles;
    wire cnn_debug_done;
    wire [3:0] cnn_prediction;
    wire signed [31:0] cnn_best,cnn_second,cnn_other,cnn_margin;
    reg [5:0] after_debug_state;
    wire capture_debug=DIGIT_DEBUG_ENABLE && debug_enable && debug_ready &&
        (debug_select==15 || debug_select==cnn_attempts-1'b1);
    reg [50:0] bank_scan_box;
    integer parent_slot;
    always @(*) begin
        bank_scan_box=0;
        for(parent_slot=0;parent_slot<TARGETS;parent_slot=parent_slot+1)
            if(parent_index==parent_slot)bank_scan_box=selected_results[parent_slot*51 +: 51];
    end
    wire [50:0] compare_box=state==PARENT_SCAN ? bank_scan_box:{3'd0,window_read_box};
    wire [1:0] compare_grade=window_read_grade;
    wire [50:0] relation_box=(state==MERGE_SCAN || state==PARENT_SCAN)?{3'd0,pending_box}:merge_box;
    wire [11:0] union_left=relation_box[47:36]<compare_box[47:36]?relation_box[47:36]:compare_box[47:36];
    wire [11:0] union_top=relation_box[35:24]<compare_box[35:24]?relation_box[35:24]:compare_box[35:24];
    wire [11:0] union_right=relation_box[23:12]>compare_box[23:12]?relation_box[23:12]:compare_box[23:12];
    wire [11:0] union_bottom=relation_box[11:0]>compare_box[11:0]?relation_box[11:0]:compare_box[11:0];
    wire [12:0] merge_w={1'b0,relation_box[23:12]}-{1'b0,relation_box[47:36]}+1'b1;
    wire [12:0] merge_h={1'b0,relation_box[11:0]}-{1'b0,relation_box[35:24]}+1'b1;
    wire [12:0] compare_w={1'b0,compare_box[23:12]}-{1'b0,compare_box[47:36]}+1'b1;
    wire [12:0] compare_h={1'b0,compare_box[11:0]}-{1'b0,compare_box[35:24]}+1'b1;
    wire [25:0] merge_area=merge_w*merge_h;
    wire [25:0] compare_area=state==PARENT_SCAN?compare_w*compare_h:{6'd0,window_read_area};
    wire contains_compare=relation_box[47:36]<=compare_box[47:36] &&
        relation_box[35:24]<=compare_box[35:24] && relation_box[23:12]>=compare_box[23:12] &&
        relation_box[11:0]>=compare_box[11:0];
    wire compare_contains=compare_box[47:36]<=relation_box[47:36] &&
        compare_box[35:24]<=relation_box[35:24] && compare_box[23:12]>=relation_box[23:12] &&
        compare_box[11:0]>=relation_box[11:0];
    wire [12:0] center_delta_x=({1'b0,relation_box[47:36]}+relation_box[23:12] >
        {1'b0,compare_box[47:36]}+compare_box[23:12]) ?
        ({1'b0,relation_box[47:36]}+relation_box[23:12])-({1'b0,compare_box[47:36]}+compare_box[23:12]):
        ({1'b0,compare_box[47:36]}+compare_box[23:12])-({1'b0,relation_box[47:36]}+relation_box[23:12]);
    wire [12:0] center_delta_y=({1'b0,relation_box[35:24]}+relation_box[11:0] >
        {1'b0,compare_box[35:24]}+compare_box[11:0]) ?
        ({1'b0,relation_box[35:24]}+relation_box[11:0])-({1'b0,compare_box[35:24]}+compare_box[11:0]):
        ({1'b0,compare_box[35:24]}+compare_box[11:0])-({1'b0,relation_box[35:24]}+relation_box[11:0]);
    wire nested=(contains_compare || compare_contains) &&
        ((merge_area>compare_area*2 && center_delta_x<=merge_w && center_delta_y<=merge_h) ||
         (compare_area>merge_area*2 && center_delta_x<=compare_w && center_delta_y<=compare_h));
    wire near_x=relation_box[47:36]<=compare_box[23:12]+12'd8 &&
        compare_box[47:36]<=relation_box[23:12]+12'd8;
    wire near_y=relation_box[35:24]<=compare_box[11:0]+12'd8 &&
        compare_box[35:24]<=relation_box[11:0]+12'd8;
    wire [12:0] union_w={1'b0,union_right}-{1'b0,union_left}+1'b1;
    wire [12:0] union_h={1'b0,union_bottom}-{1'b0,union_top}+1'b1;
    wire split_stroke=near_x && near_y &&
        ((merge_area>compare_area*4) || (compare_area>merge_area*4)) &&
        union_w<=union_h*3 && union_h<=union_w*4;
    wire merge_this=merge_index<window_count &&
        !contains_compare && !compare_contains && split_stroke;
    wire [12:0] pending_center_dx=
        ({1'b0,merge_box[47:36]}+merge_box[23:12] >
         {1'b0,pending_box[47:36]}+pending_box[23:12]) ?
        ({1'b0,merge_box[47:36]}+merge_box[23:12])-
        ({1'b0,pending_box[47:36]}+pending_box[23:12]):
        ({1'b0,pending_box[47:36]}+pending_box[23:12])-
        ({1'b0,merge_box[47:36]}+merge_box[23:12]);
    wire [12:0] pending_center_dy=
        ({1'b0,merge_box[35:24]}+merge_box[11:0] >
         {1'b0,pending_box[35:24]}+pending_box[11:0]) ?
        ({1'b0,merge_box[35:24]}+merge_box[11:0])-
        ({1'b0,pending_box[35:24]}+pending_box[11:0]):
        ({1'b0,pending_box[35:24]}+pending_box[11:0])-
        ({1'b0,merge_box[35:24]}+merge_box[11:0]);
    wire suppress_pending=pending_valid &&
        merge_box[47:36]<=pending_box[47:36] &&
        merge_box[35:24]<=pending_box[35:24] &&
        merge_box[23:12]>=pending_box[23:12] &&
        merge_box[11:0]>=pending_box[11:0] &&
        merge_area>({6'd0,pending_area}<<1) &&
        pending_center_dx<=merge_w && pending_center_dy<=merge_h;
    reg [XB-1:0] best_left,best_right;
    reg [YB-1:0] best_top,best_bottom;
    reg [49:0] best_result;
    wire [11:0] out_left=left*SAMPLE,out_top=top*SAMPLE;
    wire [11:0] out_right=(right+1)*SAMPLE>=WIDTH ? WIDTH-1 : (right+1)*SAMPLE-1;
    wire [11:0] out_bottom=(bottom+1)*SAMPLE>=HEIGHT ? HEIGHT-1 : (bottom+1)*SAMPLE-1;
    // Prefer an inner target to a much larger surrounding paper/frame. Nearby
    // inner/outer strokes of one hollow shape still select its outer contour.
    wire inside_best=left>=best_left && right<=best_right && top>=best_top && bottom<=best_bottom;
    wire contains_best=left<=best_left && right>=best_right && top<=best_top && bottom>=best_bottom;
    wire choose_candidate=best_area==0 || (inside_best && best_area>candidate_area*2) ||
        (candidate_area>best_area && !(contains_best && candidate_area>best_area*2));
    wire [49:0] selected_first,selected_second;
    wire [1:0] selected_quality,selected_second_quality;
    wire [TARGETS*2-1:0] selected_qualities;
    wire [TARGETS*4-1:0] selected_digits;
    wire bank_ready;
    wire [11:0] merged_out_left=merge_box[47:36],merged_out_top=merge_box[35:24];
    wire [11:0] merged_out_right=merge_box[23:12],merged_out_bottom=merge_box[11:0];
    wire triangle_done,triangle_ok;
    wire [1:0] triangle_quality;
    generate if(TRIANGLE_ENABLE) begin: g_triangle
        Shape_Triangle #(.XB(XB),.YB(YB),.QB(QB)) triangle (
            .clk(clk),.rst_n(rst_n),.cancel(start),.start(state==TRI_START),.count(count),
            .point_address(triangle_address),.point_data(point_q),
            .done(triangle_done),.valid(triangle_ok),.quality(triangle_quality));
    end else begin: g_no_triangle
        assign triangle_address=0;assign triangle_done=1;assign triangle_ok=0;assign triangle_quality=0;
    end endgenerate
    generate if(TARGETS>1 || TRIANGLE_ENABLE) begin: g_targets
        Shape_Target_Bank #(.TARGETS(TARGETS),
            .AREA_BITS(DIGIT_ENABLE?20:32),
            .ALLOW_CROSS_KIND_CONTAINMENT(DIGIT_ENABLE)) select_targets (
            .clk(clk),.rst_n(rst_n),.clear(start),
            .accept(DIGIT_ENABLE?state==FINAL_ACCEPT:state==ACCEPT),
            .candidate(DIGIT_ENABLE?{final_kind,merged_out_left,merged_out_top,
                merged_out_right,merged_out_bottom}:
                {1'b0,component_kind,out_left,out_top,out_right,out_bottom}),
            .candidate_digit(DIGIT_ENABLE?final_digit:4'd0),
            .quality(DIGIT_ENABLE?final_grade:component_quality),
            .area(DIGIT_ENABLE?{6'd0,merge_area}:candidate_area),
            .results(selected_results),.qualities(selected_qualities),.digits(selected_digits),.ready(bank_ready));
        assign selected_first=selected_results[49:0];assign selected_quality=selected_qualities[1:0];
        if(TARGETS>1) begin: g_second
            assign selected_second=selected_results[100:51];assign selected_second_quality=selected_qualities[3:2];
        end else begin: g_no_second
            assign selected_second=0;assign selected_second_quality=0;
        end
    end else begin: g_single
        assign bank_ready=1;
        assign selected_first=best_result;assign selected_second=0;
        assign selected_quality=0;assign selected_second_quality=0;
        assign selected_results={1'b0,best_result};assign selected_qualities=0;
        assign selected_digits=0;
    end endgenerate
    generate if(DIGIT_ENABLE) begin: g_digit
        Shape_Digit_Merge_Window merge_window(.clk(clk),.rst_n(rst_n),.clear(start),
            .append(state==WINDOW_APPEND),
            .shift(state==WINDOW_REMOVE && shift_index+1<window_count),
            .pop(state==WINDOW_REMOVE && !(shift_index+1<window_count)),
            .shift_index(shift_index),.read_index(merge_index),
            .input_box(pending_box),.input_geometry(pending_geometry),
            .input_grade(pending_grade),.input_area(pending_area),.count(window_count),
            .read_box(window_read_box),.read_geometry(window_read_geometry),
            .read_grade(window_read_grade),.read_area(window_read_area),
            .head_box(window_head_box),.head_geometry(window_head_geometry),.head_grade(window_head_grade));
        Shape_Digit_CNN #(.DEBUG_ENABLE(DIGIT_DEBUG_ENABLE)) classifier(.clk(clk),.rst_n(rst_n),
            .cancel(start || (state==CNN_WAIT && elapsed>=MAX_ANALYSIS_CYCLES-1)),
            .start(state==CNN_START),
            .bitmap_write(state==NORM_CLEAR || (state==NORM_USE && grid_q)),
            .bitmap_write_address(state==NORM_CLEAR?norm_clear_address:norm_bin_y*10'd28+norm_bin_x),
            .bitmap_write_data(state!=NORM_CLEAR),.busy(cnn_busy),
            .done(cnn_done),.digit_valid(cnn_digit_valid),.digit_value(cnn_digit),
            .confidence(cnn_confidence),.cycles(cnn_cycles),
            .debug_request(state==CNN_WAIT && cnn_done && capture_debug),
            .debug_valid(debug_valid),.debug_bit(debug_bit),.debug_last(debug_last),
            .debug_address(debug_address),.debug_done(cnn_debug_done),
            .debug_prediction(cnn_prediction),.debug_best(cnn_best),.debug_second(cnn_second),
            .debug_other(cnn_other),.debug_margin(cnn_margin));
    end else begin: g_no_digit
        assign window_count=0;assign window_read_box=0;assign window_read_geometry=0;
        assign window_read_grade=0;assign window_read_area=0;
        assign window_head_box=0;assign window_head_geometry=0;assign window_head_grade=0;
        assign cnn_busy=0;assign cnn_done=0;assign cnn_digit_valid=0;
        assign cnn_digit=0;assign cnn_confidence=0;assign cnn_cycles=0;
        assign debug_valid=0;assign debug_bit=0;assign debug_last=0;assign debug_address=0;
        assign cnn_debug_done=0;assign cnn_prediction=0;
        assign cnn_best=0;assign cnn_second=0;assign cnn_other=0;assign cnn_margin=0;
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;result_valid<=0;result<=0;elapsed<=0;
            result_second<=0;result_quality<=0;second_quality<=0;
            results<=0;qualities<=0;digits<=0;
            row_acc<=0;group_acc<=0;scan_x<=0;scan_y<=0;neighbor_address<=0;
            nx<=0;ny<=0;point_x<=0;point_y<=0;left<=0;right<=0;top<=0;bottom<=0;
            head<=0;tail<=0;index<=0;queued<=0;count<=0;oversized<=0;neighbor_index<=0;
            angle<=0;scoring<=0;ellipse_seen<=0;straight_seen<=0;component_kind<=0;
            component_quality<=0;ellipse_quality<=0;
            xc<=0;xs<=0;yc<=0;ys<=0;u<=0;v<=0;lo_u<=0;hi_u<=0;lo_v<=0;hi_v<=0;
            du<=0;dv<=0;delta_u<=0;delta_v<=0;product_u<=0;product_v<=0;norm_u<=0;norm_v<=0;
            div_axis<=0;div_bit<=0;quotient<=0;inv_u<=0;inv_v<=0;remainder<=0;divisor<=0;
            tol_u<=0;tol_v<=0;strict_u<=0;strict_v<=0;radius_tolerance<=0;radius<=0;
            rect_hits<=0;ellipse_hits<=0;straight_hits<=0;rect_mask<=0;ellipse_mask<=0;corners<=0;
            point_issue<=0;point_valid<=0;point_last<=0;point_rect<=0;point_straight<=0;
            point_rect_mask<=0;point_corners<=0;point_sector<=0;
            candidate_area<=0;best_area<=0;best_result<=0;
            best_left<=0;best_right<=0;best_top<=0;best_bottom<=0;
            merge_index<=0;shift_index<=0;remove_return<=0;parent_index<=0;active_parent_index<=0;
            pending_box<=0;pending_geometry<=0;pending_grade<=0;pending_area<=0;
            pending_valid<=0;scan_done<=0;cnn_attempts<=0;merge_box<=0;
            debug_metadata<=0;after_debug_state<=FINAL_ACCEPT;
            merge_grade<=0;final_grade<=0;merge_geometry<=0;final_kind<=0;final_digit<=0;
            norm_x<=0;norm_right<=0;norm_y<=0;norm_bottom<=0;
            norm_width<=0;norm_height<=0;norm_remainder_x<=0;norm_remainder_y<=0;
            norm_bin_x<=0;norm_bin_y<=0;norm_clear_address<=0;
        end else if(start) begin
            state<=STREAM;result_valid<=0;result<=0;elapsed<=0;
            result_second<=0;result_quality<=0;second_quality<=0;
            results<=0;qualities<=0;digits<=0;
            row_acc<=0;group_acc<=0;scan_x<=0;scan_y<=0;
            best_result<=0;best_area<=0;
            merge_index<=0;shift_index<=0;
            pending_valid<=0;scan_done<=0;cnn_attempts<=0;
            point_issue<=0;point_valid<=0;point_last<=0;
        end else if(state>=SCAN_READ && state<=BANK_WAIT &&
                    (elapsed>=MAX_ANALYSIS_CYCLES || queue_overflow)) begin
            // Never publish a partial winner after a work/capacity failure.
            state<=IDLE;result<=0;result_valid<=1;
            result_second<=0;result_quality<=0;second_quality<=0;
            results<=0;qualities<=0;digits<=0;
            point_issue<=0;point_valid<=0;point_last<=0;
        end else begin
            if(state>=SCAN_READ) elapsed<=elapsed+1'b1;
            case(state)
                STREAM: if(pixel_valid) begin
                    if(group_end) begin row_acc[input_x]<=row_end?1'b0:pooled;group_acc<=0;end
                    else group_acc<=group_acc|pixel_edge;
                    if(pixel_x==WIDTH-1 && pixel_y==HEIGHT-1) state<=SCAN_READ;
                end
                SCAN_READ:state<=SCAN_TEST;
                SCAN_TEST:if(seed) begin
                    head<=0;tail<=1;queued<=1;count<=1;oversized<=0;
                    left<=scan_x;right<=scan_x;top<=scan_y;bottom<=scan_y;state<=QUEUE_READ;
                end else state<=SCAN_NEXT;
                SCAN_NEXT:begin
                    if(scan_x==CW-1) begin
                        scan_x<=0;
                        if(scan_y==CH-1) begin
                            if(DIGIT_ENABLE) begin scan_done<=1;state<=WINDOW_SCAN;end
                            else begin
                                state<=IDLE;result_valid<=1;result<=selected_first;result_second<=selected_second;
                                result_quality<=selected_quality;second_quality<=selected_second_quality;
                                results<=selected_results;qualities<=selected_qualities;digits<=selected_digits;
                            end
                        end
                        else begin scan_y<=scan_y+1'b1;state<=SCAN_READ;end
                    end else begin scan_x<=scan_x+1'b1;state<=SCAN_READ;end
                end
                QUEUE_READ:state<=QUEUE_POP;
                QUEUE_POP:begin
                    point_x<=point_q[XB-1:0];point_y<=point_q[XB+YB-1:XB];
                    if(point_q[XB-1:0]<left) left<=point_q[XB-1:0];
                    if(point_q[XB-1:0]>right) right<=point_q[XB-1:0];
                    if(point_q[XB+YB-1:XB]<top) top<=point_q[XB+YB-1:XB];
                    if(point_q[XB+YB-1:XB]>bottom) bottom<=point_q[XB+YB-1:XB];
                    head<=head+1'b1;queued<=queued-1'b1;neighbor_index<=0;state<=NEIGHBOR;
                end
                NEIGHBOR:begin
                    if(next_x<0 || next_y<0 || next_x>=CW || next_y>=CH) state<=NEIGHBOR_NEXT;
                    else begin nx<=next_x;ny<=next_y;neighbor_address<=next_y*CW+next_x;state<=NEIGHBOR_READ;end
                end
                NEIGHBOR_READ:state<=NEIGHBOR_TEST;
                NEIGHBOR_TEST:begin
                    if(neighbor_hit) begin
                        tail<=tail+1'b1;queued<=queued+1'b1;
                        if(count==MAX_POINTS) oversized<=1;else count<=count+1'b1;
                    end
                    state<=NEIGHBOR_NEXT;
                end
                NEIGHBOR_NEXT:begin
                    if(neighbor_index==7) state<=queued!=0?QUEUE_READ:COMPONENT;
                    else begin neighbor_index<=neighbor_index+1'b1;state<=NEIGHBOR;end
                end
                COMPONENT:begin
                    if(oversized || (DIGIT_ENABLE &&
                       (count<6 || (right-left+1)*SAMPLE>WIDTH*3/4 ||
                        (bottom-top+1)*SAMPLE>HEIGHT*3/4 ||
                        right-left+1>(bottom-top+1)*8 ||
                        bottom-top+1>(right-left+1)*8)) || (!DIGIT_ENABLE &&
                       (left==0 || top==0 || right==CW-1 || bottom==CH-1)) ||
                       (right-left+1)*SAMPLE<(DIGIT_ENABLE?8:MIN_SIZE) ||
                       (bottom-top+1)*SAMPLE<(DIGIT_ENABLE?8:MIN_SIZE)) state<=SCAN_NEXT;
                    else if(DIGIT_ENABLE && ((right-left+1)*SAMPLE<MIN_SIZE ||
                                              (bottom-top+1)*SAMPLE<MIN_SIZE)) begin
                        candidate_area<=(right-left+1)*(bottom-top+1);
                        component_kind<=0;component_quality<=0;state<=ACCEPT;
                    end
                    else begin
                        candidate_area<=(right-left+1)*(bottom-top+1);
                        angle<=0;ellipse_seen<=0;straight_seen<=0;component_kind<=0;state<=ANGLE_START;
                        component_quality<=0;ellipse_quality<=0;
                    end
                end
                ANGLE_START:begin
                    lo_u<={1'b0,{(PB-1){1'b1}}};lo_v<={1'b0,{(PB-1){1'b1}}};
                    hi_u<={1'b1,{(PB-1){1'b0}}};hi_v<={1'b1,{(PB-1){1'b0}}};
                    index<=0;scoring<=0;state<=POINT_PIPELINE?POINT_STREAM:POINT_READ;
                    point_issue<=1;point_valid<=0;point_last<=0;
                end
                POINT_STREAM:begin
                    point_valid<={point_valid[5:0],point_issue};
                    point_last<={point_last[5:0],point_issue && {1'b0,index}+1==count};
                    if(point_issue)begin
                        if({1'b0,index}+1==count)point_issue<=0;
                        else index<=index+1'b1;
                    end
                    if(point_valid[0])begin
                        xc<=$signed({1'b0,point_q[XB-1:0]})*$signed({1'b0,cosine(angle)});
                        xs<=$signed({1'b0,point_q[XB-1:0]})*$signed({1'b0,sine(angle)});
                        yc<=$signed({1'b0,point_q[XB+YB-1:XB]})*$signed({1'b0,cosine(angle)});
                        ys<=$signed({1'b0,point_q[XB+YB-1:XB]})*$signed({1'b0,sine(angle)});
                    end
                    if(point_valid[1])begin u<=xc+ys;v<=yc-xs;end
                    if(point_valid[2])begin
                        if(scoring)begin
                            delta_u<=($signed(u)*2)-$signed(lo_u)-$signed(hi_u);
                            delta_v<=($signed(v)*2)-$signed(lo_v)-$signed(hi_v);
                        end else begin
                            if(u<lo_u)lo_u<=u;if(u>hi_u)hi_u<=u;
                            if(v<lo_v)lo_v<=v;if(v>hi_v)hi_v<=v;
                            if(point_last[2])state<=SPAN;
                        end
                    end
                    if(scoring)begin
                        if(point_valid[3])begin
                            product_u<=delta_u*$signed({1'b0,inv_u});
                            product_v<=delta_v*$signed({1'b0,inv_v});
                        end
                        if(point_valid[4])begin
                            norm_u<=product_u>>>16;norm_v<=product_v>>>16;
                            radius_tolerance<=((tol_u>tol_v?tol_u:tol_v)*512)>30000?30000:
                                ((tol_u>tol_v?tol_u:tol_v)*512)<12000?12000:(tol_u>tol_v?tol_u:tol_v)*512;
                        end
                        if(point_valid[5])begin
                            radius<=abs_u*abs_u+abs_v*abs_v;
                            // Align all normalized-coordinate features with
                            // the registered radius of the SAME point.
                            point_rect<=near_u || near_v;point_straight<=straight_point;
                            point_rect_mask<=(near_u?(16'b1<<u_bin):16'b0) | (near_v?(16'b1<<v_bin):16'b0);
                            point_corners<=abs_u>=208 && abs_v>=208?(4'b1<<corner_index):4'd0;
                            point_sector<=sector;
                        end
                        if(point_valid[6])begin
                            if(point_rect)rect_hits<=rect_hits+1'b1;
                            if(point_straight)straight_hits<=straight_hits+1'b1;
                            rect_mask<=rect_mask | point_rect_mask;corners<=corners | point_corners;
                            if(on_ellipse)begin ellipse_hits<=ellipse_hits+1'b1;ellipse_mask<=ellipse_mask|(16'b1<<point_sector);end
                            if(point_last[6])state<=DECIDE;
                        end
                    end
                end
                POINT_READ:state<=POINT_MULT;
                POINT_MULT:begin
                    xc<=$signed({1'b0,point_q[XB-1:0]})*$signed({1'b0,cosine(angle)});
                    xs<=$signed({1'b0,point_q[XB-1:0]})*$signed({1'b0,sine(angle)});
                    yc<=$signed({1'b0,point_q[XB+YB-1:XB]})*$signed({1'b0,cosine(angle)});
                    ys<=$signed({1'b0,point_q[XB+YB-1:XB]})*$signed({1'b0,sine(angle)});
                    state<=POINT_SUM;
                end
                POINT_SUM:begin u<=xc+ys;v<=yc-xs;state<=POINT_USE;end
                POINT_USE:if(scoring) state<=NORMAL_DELTA;
                else begin
                    if(u<lo_u) lo_u<=u;if(u>hi_u) hi_u<=u;
                    if(v<lo_v) lo_v<=v;if(v>hi_v) hi_v<=v;
                    if({1'b0,index}+1==count) state<=SPAN;
                    else begin index<=index+1'b1;state<=POINT_READ;end
                end
                SPAN:begin du<=hi_u-lo_u;dv<=hi_v-lo_v;state<=SPAN_CHECK;end
                SPAN_CHECK:begin
                    if(du*SAMPLE<(MIN_SIZE-1)*256 || dv*SAMPLE<(MIN_SIZE-1)*256 || du>dv*8 || dv>du*8) begin
                        if(angle==15) begin
                            component_kind<=(ellipse_seen && !straight_seen)?2:0;
                            component_quality<=ellipse_quality;
                            state<=TRIANGLE_ENABLE && right-left>=8 && bottom-top>=8?TRI_START:ACCEPT;
                        end
                        else begin angle<=angle+1'b1;state<=ANGLE_START;end
                    end else begin div_axis<=0;state<=DIV_START;end
                end
                DIV_START:begin divisor<=div_axis?dv:du;div_bit<=24;quotient<=0;remainder<=0;state<=DIVIDE;end
                DIVIDE:begin
                    remainder<=subtract?shifted_remainder-{1'b0,divisor}:shifted_remainder;
                    quotient[div_bit]<=subtract;
                    if(div_bit==0) begin
                        if(div_axis) begin inv_v<={quotient[24:1],subtract};state<=TOLERANCE;end
                        else begin inv_u<={quotient[24:1],subtract};div_axis<=1;state<=DIV_START;end
                    end else div_bit<=div_bit-1'b1;
                end
                TOLERANCE:begin
                    tol_u<=(inv_u>>5)<32?32:(inv_u>>5)>80?80:(inv_u>>5);
                    tol_v<=(inv_v>>5)<32?32:(inv_v>>5)>80?80:(inv_v>>5);
                    strict_u<=(inv_u>>6)<16?16:(inv_u>>6)>48?48:(inv_u>>6);
                    strict_v<=(inv_v>>6)<16?16:(inv_v>>6)>48?48:(inv_v>>6);
                    index<=0;scoring<=1;rect_hits<=0;ellipse_hits<=0;straight_hits<=0;
                    rect_mask<=0;ellipse_mask<=0;corners<=0;state<=POINT_PIPELINE?POINT_STREAM:POINT_READ;
                    point_issue<=1;point_valid<=0;point_last<=0;
                end
                NORMAL_DELTA:begin
                    delta_u<=($signed(u)*2)-$signed(lo_u)-$signed(hi_u);
                    delta_v<=($signed(v)*2)-$signed(lo_v)-$signed(hi_v);state<=NORMAL_MULT;
                end
                NORMAL_MULT:begin
                    product_u<=delta_u*$signed({1'b0,inv_u});
                    product_v<=delta_v*$signed({1'b0,inv_v});state<=NORMAL_SHIFT;
                end
                NORMAL_SHIFT:begin
                    norm_u<=product_u>>>16;norm_v<=product_v>>>16;
                    radius_tolerance<=((tol_u>tol_v?tol_u:tol_v)*512)>30000?30000:
                        ((tol_u>tol_v?tol_u:tol_v)*512)<12000?12000:(tol_u>tol_v?tol_u:tol_v)*512;
                    state<=RADIUS;
                end
                RADIUS:begin radius<=abs_u*abs_u+abs_v*abs_v;state<=SCORE;end
                SCORE:begin
                    if(near_u || near_v) rect_hits<=rect_hits+1'b1;
                    if(straight_point) straight_hits<=straight_hits+1'b1;
                    rect_mask<=rect_mask | (near_u?(16'b1<<u_bin):16'b0) | (near_v?(16'b1<<v_bin):16'b0);
                    if(abs_u>=208 && abs_v>=208) corners<=corners|(4'b1<<corner_index);
                    if(on_ellipse) begin ellipse_hits<=ellipse_hits+1'b1;ellipse_mask<=ellipse_mask|(16'b1<<sector);end
                    if({1'b0,index}+1==count) state<=DECIDE;
                    else begin index<=index+1'b1;state<=POINT_READ;end
                end
                DECIDE:begin
                    ellipse_seen<=ellipse_seen|ellipse_ok;straight_seen<=straight_seen|rectangle_like;
                    if(ellipse_ok && ellipse_grade>ellipse_quality) ellipse_quality<=ellipse_grade;
                    if(rectangle_ok) begin component_kind<=1;component_quality<=rect_quality;state<=ACCEPT;end
                    else if(angle==15) begin
                        component_kind<=((ellipse_seen || ellipse_ok) && !(straight_seen || rectangle_like))?2:0;
                        component_quality<=(ellipse_ok && ellipse_grade>ellipse_quality)?ellipse_grade:ellipse_quality;
                        state<=TRIANGLE_ENABLE && right-left>=8 && bottom-top>=8?TRI_START:ACCEPT;
                    end else begin angle<=angle+1'b1;state<=ANGLE_START;end
                end
                TRI_START:state<=TRI_WAIT;
                TRI_WAIT:if(triangle_done) begin
                    if(triangle_ok) begin component_kind<=3;component_quality<=triangle_quality;end
                    state<=ACCEPT;
                end
                ACCEPT:if(DIGIT_ENABLE || bank_ready) begin
                    if(component_kind!=0 && choose_candidate) begin
                        best_area<=candidate_area;best_result<={component_kind,out_left,out_top,out_right,out_bottom};
                        best_left<=left;best_right<=right;best_top<=top;best_bottom<=bottom;
                    end
                    if(DIGIT_ENABLE) begin
                        pending_box<={out_left,out_top,out_right,out_bottom};
                        pending_geometry<=component_kind;
                        pending_grade<=component_kind==0?2'd1:component_quality;
                        pending_area<=candidate_area[19:0]<<(2*SAMPLE_SHIFT);
                        pending_valid<=1;parent_index<=0;state<=PARENT_SCAN;
                    end else state<=TARGETS>1 || TRIANGLE_ENABLE?BANK_WAIT:SCAN_NEXT;
                end
                BANK_WAIT:if(bank_ready) state<=SCAN_NEXT;
                // The online Bank retains completed digit parents after they
                // leave the four-entry window. Suppress late inner contours
                // before spending a CNN call or publishing a geometry hole.
                PARENT_SCAN:begin
                    if(parent_index==TARGETS)begin merge_index<=0;state<=MERGE_SCAN;end
                    else if(compare_box[50:48]==4 && compare_contains &&
                        compare_area>merge_area*2 && center_delta_x<=compare_w && center_delta_y<=compare_h)begin
                        pending_valid<=0;state<=WINDOW_SCAN;
                    end else parent_index<=parent_index+1'b1;
                end
                WINDOW_SCAN:begin
                    if(pending_valid && window_count<MERGE_SLOTS) state<=WINDOW_APPEND;
                    else if(window_count!=0 && (scan_done || pending_valid ||
                            {1'b0,scan_y}>{1'b0,window_head_box[11:0]>>SAMPLE_SHIFT}+2))
                        state<=WINDOW_FLUSH;
                    else if(scan_done) state<=PUBLISH;
                    else state<=SCAN_NEXT;
                end
                MERGE_SCAN:begin
                    if(merge_index==window_count) state<=WINDOW_SCAN;
                    else if(merge_this) begin
                        pending_box<={union_left,union_top,union_right,union_bottom};
                        pending_area<=union_w*union_h;
                        if(compare_grade>pending_grade) pending_grade<=compare_grade;
                        if(window_read_geometry!=0 &&
                           (pending_geometry==0 || compare_grade>=pending_grade))
                            pending_geometry<=window_read_geometry;
                        shift_index<=merge_index;remove_return<=MERGE_SCAN;
                        state<=WINDOW_REMOVE;
                    end else merge_index<=merge_index+1'b1;
                end
                WINDOW_APPEND:begin
                    pending_valid<=0;state<=WINDOW_SCAN;
                end
                WINDOW_REMOVE:begin
                    if(shift_index+1<window_count) begin
                        shift_index<=shift_index+1'b1;
                    end else begin
                        state<=remove_return;
                    end
                end
                WINDOW_FLUSH:begin
                    merge_box<={3'd0,window_head_box};
                    merge_grade<=window_head_grade;
                    merge_geometry<={1'b0,window_head_geometry};
                    active_parent_index<=0;merge_index<=1;state<=WINDOW_PARENT_SCAN;
                end
                // A split-stroke merge can move a parent behind its child.
                // Classify the outer entry in place; do not duplicate a swap
                // network across every field in the window.
                WINDOW_PARENT_SCAN:begin
                    if(merge_index==window_count)state<=NORM_INIT;
                    else begin
                        if(compare_contains && compare_area>merge_area*2 &&
                           center_delta_x<=compare_w && center_delta_y<=compare_h)begin
                            merge_box<={3'd0,window_read_box};merge_grade<=window_read_grade;
                            merge_geometry<={1'b0,window_read_geometry};active_parent_index<=merge_index;
                        end
                        merge_index<=merge_index+1'b1;
                    end
                end
                NORM_INIT:begin
                    // Bounded work: a slow ROI/CNN falls back to the geometric
                    // result and cannot keep the frame publication waiting.
                    if(cnn_attempts>=MAX_DIGIT_CANDIDATES || MAX_ANALYSIS_CYCLES<=400000 ||
                       elapsed>=MAX_ANALYSIS_CYCLES-400000) begin
                        final_kind<=merge_geometry;final_grade<=merge_grade;final_digit<=0;
                        state<=FINAL_ACCEPT;
                    end else begin
                        norm_clear_address<=0;
                        norm_x<=merge_box[47:36]>>SAMPLE_SHIFT;
                        norm_y<=merge_box[35:24]>>SAMPLE_SHIFT;
                        norm_right<=merge_box[23:12]>>SAMPLE_SHIFT;
                        norm_bottom<=merge_box[11:0]>>SAMPLE_SHIFT;
                        norm_width<=(merge_box[23:12]>>SAMPLE_SHIFT)-
                            (merge_box[47:36]>>SAMPLE_SHIFT)+1'b1;
                        norm_height<=(merge_box[11:0]>>SAMPLE_SHIFT)-
                            (merge_box[35:24]>>SAMPLE_SHIFT)+1'b1;
                        norm_remainder_x<=0;norm_remainder_y<=0;
                        norm_bin_x<=0;norm_bin_y<=0;state<=NORM_CLEAR;
                    end
                end
                NORM_CLEAR:begin
                    if(norm_clear_address==783) state<=NORM_READ;
                    else norm_clear_address<=norm_clear_address+1'b1;
                end
                NORM_READ:state<=NORM_USE;
                NORM_USE:begin
                    if(norm_x==norm_right) begin
                        if(norm_y==norm_bottom) state<=CNN_START;
                        else begin
                            norm_x<=merge_box[47:36]>>SAMPLE_SHIFT;
                            norm_bin_x<=0;norm_remainder_x<=0;
                            norm_y<=norm_y+1'b1;
                            norm_remainder_y<=norm_remainder_y+7'd28;
                            state<=NORM_YSTEP;
                        end
                    end else begin
                        norm_x<=norm_x+1'b1;
                        norm_remainder_x<=norm_remainder_x+7'd28;
                        state<=NORM_XSTEP;
                    end
                end
                NORM_XSTEP:begin
                    if(norm_remainder_x>=norm_width) begin
                        norm_remainder_x<=norm_remainder_x-norm_width;
                        if(norm_bin_x<27) norm_bin_x<=norm_bin_x+1'b1;
                    end else state<=NORM_READ;
                end
                NORM_YSTEP:begin
                    if(norm_remainder_y>=norm_height) begin
                        norm_remainder_y<=norm_remainder_y-norm_height;
                        if(norm_bin_y<27) norm_bin_y<=norm_bin_y+1'b1;
                    end else state<=NORM_READ;
                end
                CNN_START:begin cnn_attempts<=cnn_attempts+1'b1;state<=CNN_WAIT;end
                CNN_WAIT:begin
                    if(cnn_done) begin
                        final_kind<=cnn_digit_valid?3'd4:merge_geometry;
                        final_digit<=cnn_digit_valid?cnn_digit:4'd0;
                        final_grade<=cnn_digit_valid?cnn_confidence:merge_grade;
                        if(cnn_digit_valid) begin merge_index<=0;state<=SUPPRESS_SCAN;end
                        else state<=FINAL_ACCEPT;
                        if(capture_debug) begin
                            debug_metadata<={cnn_attempts-4'd1,merge_box[47:0],cnn_prediction,
                                merge_geometry,cnn_digit_valid?3'd4:merge_geometry,
                                cnn_digit_valid?cnn_digit:4'd0,
                                cnn_digit_valid?cnn_confidence:merge_grade,
                                cnn_best,cnn_second,cnn_other,cnn_margin,debug_threshold};
                            after_debug_state<=cnn_digit_valid?SUPPRESS_SCAN:FINAL_ACCEPT;
                            state<=DEBUG_WAIT;
                        end
                    end else if(elapsed>=MAX_ANALYSIS_CYCLES-1) begin
                        final_kind<=merge_geometry;final_digit<=0;final_grade<=merge_grade;
                        state<=FINAL_ACCEPT;
                    end
                end
                DEBUG_WAIT:if(cnn_debug_done) state<=after_debug_state;
                SUPPRESS_SCAN:begin
                    if(merge_index==window_count) begin
                        if(suppress_pending) pending_valid<=0;
                        state<=FINAL_ACCEPT;
                    end else if(merge_index!=active_parent_index && contains_compare && merge_area>compare_area*2 &&
                                center_delta_x<=merge_w && center_delta_y<=merge_h) begin
                        if(merge_index<active_parent_index)active_parent_index<=active_parent_index-1'b1;
                        shift_index<=merge_index;remove_return<=SUPPRESS_SCAN;
                        state<=WINDOW_REMOVE;
                    end else merge_index<=merge_index+1'b1;
                end
                FINAL_ACCEPT:if(bank_ready) state<=FINAL_WAIT;
                FINAL_WAIT:if(bank_ready) begin
                    shift_index<=active_parent_index;remove_return<=WINDOW_SCAN;state<=WINDOW_REMOVE;
                end
                PUBLISH:begin
                    state<=IDLE;result_valid<=1;result<=selected_first;result_second<=selected_second;
                    result_quality<=selected_quality;second_quality<=selected_second_quality;
                    results<=selected_results;qualities<=selected_qualities;digits<=selected_digits;
                end
                default:state<=IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial if(WIDTH<8 || HEIGHT<8 || WIDTH>4096 || HEIGHT>4096 ||
               SAMPLE_SHIFT<0 || SAMPLE_SHIFT>4 || MIN_SIZE<2 || MAX_POINTS<8 ||
               (1<<QB)!=MAX_POINTS || MAX_ANALYSIS_CYCLES<1 || TARGETS<1 || TARGETS>8)
        $fatal(1,"Invalid shape detector geometry/capacity");
    // synthesis translate_on
endmodule

// Shared point RAM reader. Three extreme vertices are refined by maximum
// triangle area, then every contour point is checked against all three sides.
// A wide band tolerates thin hollow strokes; a narrow band must cover >=85%
// of EVERY side's projected cells, with no gap longer than two cells.
// Relative altitude rejects diagonal ribbons. No divider or extra frame RAM.
module Shape_Triangle #(parameter XB=9,YB=8,QB=12)(
    input wire clk,rst_n,cancel,start,
    input wire [QB:0] count,
    output wire [QB-1:0] point_address,
    input wire [XB+YB-1:0] point_data,
    output reg done,valid,
    output reg [1:0] quality
);
    localparam D=(XB>YB?XB:YB)+1,P=2*D+1,SIDE_CELLS=1<<(D-1);
    localparam IDLE=0,SEED_READ=1,SEED_USE=2,PASS=3,READ=4,DELTA=5,MULT=6,USE=7,
        STORE=8,FIT=9,EDGE=10,DECIDE=11,CLEAR=12,CHECK_START=13,CHECK_READ=14,CHECK_USE=15;
    reg [3:0] state;
    reg [2:0] pass;
    reg [1:0] side;
    reg scoring,point_hit;
    reg [QB-1:0] index;
    reg [QB:0] hits;
    reg [D-2:0] vx[0:2],vy[0:2];
    reg [D-2:0] ax,ay,bx,by,best_x,best_y;
    reg [P-1:0] best_score,triangle_area;
    reg signed [D-1:0] dx,dy,px,py;
    reg signed [2*D-1:0] product0,product1;
    reg [2*D+1:0] span_square;
    reg [D-2:0] side_extent[0:2];
    reg support[0:3*SIDE_CELLS-1];
    reg support_q,complete_sides;
    reg [D:0] clear_address;
    reg [1:0] check_side,gap;
    reg [D-2:0] check_position;
    reg [D:0] covered;
    wire [D-2:0] x=point_data[XB-1:0],y=point_data[XB+YB-1:XB];
    wire signed [P-1:0] cross_value=$signed({product0[2*D-1],product0})-$signed({product1[2*D-1],product1});
    wire [P-1:0] cross_abs=cross_value<0?-cross_value:cross_value;
    wire [D-1:0] adx=dx<0?-dx:dx,ady=dy<0?-dy:dy;
    wire [D-1:0] apx=px<0?-px:px,apy=py<0?-py:py;
    wire [D:0] span={1'b0,adx}+{1'b0,ady};
    wire [P-1:0] score=pass==0?({1'b0,apx}+{1'b0,apy}):cross_abs;
    wire signed [D:0] projection=ady>adx?(dy<0?-$signed({py[D-1],py}):$signed({py[D-1],py})):
        (dx<0?-$signed({px[D-1],px}):$signed({px[D-1],px}));
    wire [D-1:0] extent=ady>adx?ady:adx;
    wire in_segment=projection>=0 && projection<=$signed({1'b0,extent});
    wire near_side=cross_abs<=span && in_segment;
    wire point_supported=cross_abs<=span*2 && in_segment;
    wire [P+2:0] scaled_area=triangle_area*6;
    wire [D:0] support_address=state==CLEAR?clear_address:
        (state==CHECK_READ || state==CHECK_USE)?{check_side,check_position}:{side,projection[D-2:0]};
    always @(posedge clk) begin
        if(state==CLEAR) support[support_address]<=0;
        else if(state==USE && scoring && near_side) support[support_address]<=1;
        support_q<=support[support_address];
    end
    wire [D:0] covered_next=covered+support_q;
    assign point_address=index;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;valid<=0;quality<=0;pass<=0;side<=0;scoring<=0;point_hit<=0;
            index<=0;hits<=0;vx[0]<=0;vx[1]<=0;vx[2]<=0;vy[0]<=0;vy[1]<=0;vy[2]<=0;
            ax<=0;ay<=0;bx<=0;by<=0;best_x<=0;best_y<=0;best_score<=0;
            dx<=0;dy<=0;px<=0;py<=0;product0<=0;product1<=0;span_square<=0;triangle_area<=0;
            side_extent[0]<=0;side_extent[1]<=0;side_extent[2]<=0;
            clear_address<=0;check_side<=0;check_position<=0;covered<=0;gap<=0;complete_sides<=0;
        end else if(cancel) begin state<=IDLE;done<=0;valid<=0;quality<=0;index<=0;end
        // The detector only submits real contours; reject a short direct invocation too.
        else if(start) begin state<=count<3?IDLE:SEED_READ;done<=count<3;valid<=0;quality<=0;index<=0;scoring<=0;pass<=0;end
        else case(state)
            SEED_READ:state<=SEED_USE;
            SEED_USE:begin vx[0]<=x;vy[0]<=y;state<=PASS;end
            PASS:begin
                case(pass)
                    0,1,4:begin ax<=vx[0];ay<=vy[0];bx<=vx[1];by<=vy[1];end
                    2:begin ax<=vx[1];ay<=vy[1];bx<=vx[2];by<=vy[2];end
                    3:begin ax<=vx[2];ay<=vy[2];bx<=vx[0];by<=vy[0];end
                endcase
                index<=0;best_score<=0;best_x<={D-1{1'b1}};best_y<={D-1{1'b1}};state<=READ;
            end
            READ:state<=DELTA;
            DELTA:begin
                dx<=$signed({1'b0,bx})-$signed({1'b0,ax});dy<=$signed({1'b0,by})-$signed({1'b0,ay});
                px<=$signed({1'b0,x})-$signed({1'b0,ax});py<=$signed({1'b0,y})-$signed({1'b0,ay});
                state<=MULT;
            end
            MULT:begin product0<=dx*py;product1<=dy*px;span_square<=span*span;state<=USE;end
            USE:if(!scoring) begin
                if(score>best_score || (score==best_score && {y,x}<{best_y,best_x})) begin
                    best_score<=score;best_x<=x;best_y<=y;
                end
                if({1'b0,index}+1==count) state<=STORE;
                else begin index<=index+1'b1;state<=READ;end
            end else if(span<8 || triangle_area<span*4 || scaled_area<span_square) begin valid<=0;done<=1;state<=IDLE;end
            else begin
                side_extent[side]<=extent[D-2:0];
                if(side==2) begin
                    if(point_hit || point_supported) hits<=hits+1'b1;
                    point_hit<=0;side<=0;
                    if({1'b0,index}+1==count) state<=CHECK_START;
                    else begin index<=index+1'b1;state<=EDGE;end
                end else begin point_hit<=point_hit|point_supported;side<=side+1'b1;state<=EDGE;end
            end
            STORE:begin
                case(pass)
                    0,3:begin vx[1]<=best_x;vy[1]<=best_y;end
                    1,4:begin vx[2]<=best_x;vy[2]<=best_y;end
                    2:begin vx[0]<=best_x;vy[0]<=best_y;end
                endcase
                if(pass==4) begin
                    triangle_area<=best_score;
                    if(best_score<64) begin done<=1;state<=IDLE;end else state<=FIT;
                end else begin pass<=pass+1'b1;state<=PASS;end
            end
            FIT:begin scoring<=1;index<=0;side<=0;hits<=0;point_hit<=0;clear_address<=0;state<=CLEAR;end
            CLEAR:if(clear_address==3*SIDE_CELLS-1) state<=EDGE;else clear_address<=clear_address+1'b1;
            EDGE:begin
                ax<=vx[side];ay<=vy[side];bx<=vx[side==2?0:side+1'b1];by<=vy[side==2?0:side+1'b1];state<=READ;
            end
            CHECK_START:begin check_side<=0;check_position<=0;covered<=0;gap<=0;complete_sides<=1;state<=CHECK_READ;end
            CHECK_READ:state<=CHECK_USE;
            CHECK_USE:begin
                covered<=covered_next;gap<=support_q?0:gap+1'b1;
                if(!support_q) complete_sides<=0;
                if(!support_q && gap==2) begin done<=1;valid<=0;state<=IDLE;end
                else if(check_position==side_extent[check_side]) begin
                    if(covered_next*100<({1'b0,side_extent[check_side]}+1)*85) begin done<=1;valid<=0;state<=IDLE;end
                    else if(check_side==2) state<=DECIDE;
                    else begin check_side<=check_side+1'b1;check_position<=0;covered<=0;gap<=0;state<=CHECK_READ;end
                end else begin check_position<=check_position+1'b1;state<=CHECK_READ;end
            end
            DECIDE:begin
                valid<=hits*100>=count*82;
                quality<=hits*100>=count*82?(hits*100>=count*95 && complete_sides?3:hits*100>=count*90?2:1):0;
                done<=1;state<=IDLE;
            end
            default:state<=IDLE;
        endcase
    end
endmodule

// Confirm two consecutive successfully published observations. Always use the
// CURRENT contour's box; never paint a held/stale rectangle over a new frame.
// Aborted/failed captures do not advance evidence because they do not publish.
module Shape_Confirm (
    input wire clk,rst_n,publish,
    input wire [49:0] raw_result,
    output wire [49:0] confirmed_result
);
    reg [49:0] previous;
    wire [11:0] l=raw_result[47:36],t=raw_result[35:24],r=raw_result[23:12],b=raw_result[11:0];
    wire [11:0] pl=previous[47:36],pt=previous[35:24],pr=previous[23:12],pb=previous[11:0];
    wire [12:0] w={1'b0,r}-{1'b0,l}+13'd1,h={1'b0,b}-{1'b0,t}+13'd1;
    wire [12:0] pw={1'b0,pr}-{1'b0,pl}+13'd1,ph={1'b0,pb}-{1'b0,pt}+13'd1;
    wire [11:0] il=l>pl?l:pl,ir=r<pr?r:pr,it=t>pt?t:pt,ib=b<pb?b:pb;
    wire [12:0] iw={1'b0,ir}-{1'b0,il}+13'd1,ih={1'b0,ib}-{1'b0,it}+13'd1;
    wire match=raw_result[49:48]!=0 && raw_result[49:48]==previous[49:48] &&
        il<=ir && it<=ib && iw*2>=w && iw*2>=pw && ih*2>=h && ih*2>=ph;
    assign confirmed_result=match?raw_result:50'd0;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) previous<=0;
        else if(publish) previous<=raw_result;
endmodule
