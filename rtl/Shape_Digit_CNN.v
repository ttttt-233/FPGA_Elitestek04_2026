`timescale 1ns/1ps
// One serial INT8 engine for every component. One RAM bitmap is written by
// the caller while idle, then read synchronously by Conv1 through done.
// Conv1 3x3x1x4 -> ReLU/pool, Conv2 3x3x4x8 -> ReLU/pool,
// 7x7x8 -> 4x4 spatial averages -> FC 128x11. One signed MAC is reused.
// Geometry remains the fallback when digit_valid is zero (OTHER/low margin).
module Shape_Digit_CNN #(parameter DEBUG_ENABLE=0)(
    input wire clk,rst_n,cancel,start,
    input wire bitmap_write,bitmap_write_data,
    input wire [9:0] bitmap_write_address,
    output wire busy,
    output reg done,digit_valid,
    output reg [3:0] digit_value,
    output reg [1:0] confidence,
    output reg [21:0] cycles,
    input wire debug_request,
    output reg debug_valid,debug_last,debug_done,
    output reg [9:0] debug_address,
    output reg debug_bit,
    output wire [3:0] debug_prediction,
    output wire signed [31:0] debug_best,debug_second,debug_other,debug_margin
);
    localparam IDLE=0,C1_INIT=1,C1_LOAD=2,C1_ACC=3,C1_POST=4,C1_WRITE=5,
        C2_INIT=6,C2_LOAD=7,C2_ACC=8,C2_POST=9,C2_WRITE=10,
        SCORE_INIT=11,GAP_LOAD=12,GAP_ACC=13,GAP_POST=14,
        FC_LOAD=15,FC_ACC=16,FIND_INIT=17,FIND_SCAN=18,FINISH=19;
    reg [4:0] state;
    reg debug_active,debug_read_valid;
    reg [9:0] debug_issue_address,debug_read_address;
    assign busy=state!=IDLE || debug_active || debug_read_valid;
    reg [3:0] out_x,out_y,out_ch;
    reg [1:0] sub,kx,ky,in_ch;
    reg signed [31:0] sum;
    reg [7:0] pooled_max;
    reg [7:0] fmap1[0:783];
    reg [7:0] fmap2[0:391];
    reg [7:0] fmap1_q,fmap2_q;
    reg bitmap[0:783];
    reg bitmap_q;
    reg [3:0] gap_ch;
    reg [1:0] gap_qx,gap_qy,gap_dx,gap_dy;
    reg [11:0] gap_sum;
    reg [7:0] feature_value;
    reg [3:0] class_index,scan_class,best_digit;
    reg signed [31:0] scores[0:10];
    reg signed [31:0] score_q,other_score;
    reg signed [31:0] best_score,runner_score;
    wire signed [31:0] rival_score=other_score>runner_score?other_score:runner_score;
    wire signed [31:0] margin_score=best_score-rival_score;
    assign debug_prediction=other_score>best_score?4'd10:best_digit;
    assign debug_best=best_score;assign debug_second=rival_score;
    assign debug_other=other_score;assign debug_margin=margin_score;
    wire [6:0] c1_x={2'b0,out_x,1'b0}+{6'b0,sub[0]}+{5'b0,kx};
    wire [6:0] c1_y={2'b0,out_y,1'b0}+{6'b0,sub[1]}+{5'b0,ky};
    wire c1_inside=c1_x>0 && c1_x<=28 && c1_y>0 && c1_y<=28;
    wire [9:0] bitmap_address=(c1_y-1'b1)*10'd28+(c1_x-1'b1);
    wire [5:0] c2_x={1'b0,out_x,1'b0}+{5'b0,sub[0]}+{4'b0,kx};
    wire [5:0] c2_y={1'b0,out_y,1'b0}+{5'b0,sub[1]}+{4'b0,ky};
    wire c2_inside=c2_x>0 && c2_x<=14 && c2_y>0 && c2_y<=14;
    wire [9:0] fmap1_read_address=in_ch*10'd196+(c2_y-1'b1)*10'd14+(c2_x-1'b1);
    wire [9:0] fmap1_write_address=out_ch*10'd196+out_y*10'd14+out_x;
    wire [8:0] fmap2_write_address=out_ch*9'd49+out_y*9'd7+out_x;
    function [2:0] pool_start;
        input [1:0] part;
        begin case(part) 0:pool_start=0;1:pool_start=1;2:pool_start=3;default:pool_start=5;endcase end
    endfunction
    function [1:0] pool_length;
        input [1:0] part;
        begin pool_length=(part==0 || part==3)?2:3;end
    endfunction
    wire [2:0] pool_x=pool_start(gap_qx)+{1'b0,gap_dx};
    wire [2:0] pool_y=pool_start(gap_qy)+{1'b0,gap_dy};
    wire [8:0] fmap2_read_address=gap_ch*9'd49+pool_y*9'd7+pool_x;
    wire [3:0] pool_area=pool_length(gap_qx)*pool_length(gap_qy);
    wire [6:0] feature_index={gap_ch[2:0],gap_qy,gap_qx};
    wire [10:0] weight_address=state==C1_LOAD?out_ch*11'd9+ky*11'd3+kx:
        state==C2_LOAD?11'd36+out_ch*11'd36+in_ch*11'd9+ky*11'd3+kx:
        11'd324+class_index*11'd128+feature_index;
    wire [4:0] bias_address=state==C1_INIT?{1'b0,out_ch}:
        state==C2_INIT?5'd4+{1'b0,out_ch}:5'd12+{1'b0,scan_class};
    wire signed [7:0] weight;
    wire signed [31:0] bias;
    wire [4:0] shift1,shift2;
    wire [31:0] margin;
    Shape_Digit_Weights weights(.clk(clk),.address(weight_address),.weight(weight),
        .bias_address(bias_address),.bias(bias),.shift1(shift1),.shift2(shift2),.margin(margin));
    wire signed [31:0] shifted=state==C1_POST?(sum >>> shift1):(sum >>> shift2);
    wire [7:0] activated=sum<=0?8'd0:shifted>127?8'd127:shifted[7:0];
    wire [3:0] score_read_address=state==FIND_INIT?4'd0:
        state==FIND_SCAN?scan_class+4'd1:class_index;
    wire score_write=state==SCORE_INIT || state==FC_ACC;
    wire [3:0] score_write_address=state==SCORE_INIT?scan_class:class_index;
    wire signed [31:0] score_write_data=state==SCORE_INIT?bias:
        score_q+$signed({1'b0,feature_value})*weight;
    always @(posedge clk) begin
        if(bitmap_write) bitmap[bitmap_write_address]<=bitmap_write_data;
        bitmap_q<=bitmap[debug_active?debug_issue_address:bitmap_address];
        if(score_write) scores[score_write_address]<=score_write_data;
        score_q<=scores[score_read_address];
        if(state==C1_WRITE) fmap1[fmap1_write_address]<=pooled_max;
        if(state==C2_WRITE) fmap2[fmap2_write_address]<=pooled_max;
        fmap1_q<=fmap1[fmap1_read_address];
        fmap2_q<=fmap2[fmap2_read_address];
    end
    // Read the exact Conv1 RAM through the SAME synchronous read port while
    // classification is idle. No reference bitmap or second normalizer exists.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            debug_active<=0;debug_read_valid<=0;debug_issue_address<=0;
            debug_read_address<=0;debug_valid<=0;debug_last<=0;debug_done<=0;
            debug_address<=0;debug_bit<=0;
        end else if(cancel || !DEBUG_ENABLE) begin
            debug_active<=0;debug_read_valid<=0;debug_valid<=0;
            debug_last<=0;debug_done<=0;
        end else begin
            debug_valid<=debug_read_valid;debug_last<=0;debug_done<=0;
            debug_read_valid<=debug_active;
            if(debug_read_valid) begin
                debug_address<=debug_read_address;debug_bit<=bitmap_q;
                debug_last<=debug_read_address==783;debug_done<=debug_read_address==783;
            end
            if(debug_active) begin
                debug_read_address<=debug_issue_address;
                if(debug_issue_address==783) debug_active<=0;
                else debug_issue_address<=debug_issue_address+1'b1;
            end
            if(debug_request && state==IDLE && !debug_active && !debug_read_valid) begin
                debug_active<=1;debug_issue_address<=0;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;digit_valid<=0;digit_value<=0;confidence<=0;cycles<=0;
            out_x<=0;out_y<=0;out_ch<=0;sub<=0;kx<=0;ky<=0;in_ch<=0;
            sum<=0;pooled_max<=0;
            gap_ch<=0;gap_qx<=0;gap_qy<=0;gap_dx<=0;gap_dy<=0;gap_sum<=0;
            feature_value<=0;class_index<=0;scan_class<=0;best_digit<=0;
            best_score<=0;runner_score<=0;other_score<=0;
        end else if(cancel) begin
            state<=IDLE;done<=0;digit_valid<=0;confidence<=0;
        end else begin
            done<=0;
            if(state!=IDLE) cycles<=cycles+1'b1;
            case(state)
                IDLE:if(start) begin
                    out_x<=0;out_y<=0;out_ch<=0;sub<=0;kx<=0;ky<=0;in_ch<=0;
                    digit_valid<=0;confidence<=0;cycles<=0;state<=C1_INIT;
                end
                C1_INIT:begin
                    sum<=bias;kx<=0;ky<=0;
                    if(sub==0) pooled_max<=0;
                    state<=C1_LOAD;
                end
                C1_LOAD:begin
                    state<=C1_ACC;
                end
                C1_ACC:begin
                    if(c1_inside && bitmap_q) sum<=sum+weight;
                    if(kx==2) begin
                        kx<=0;
                        if(ky==2) begin ky<=0;state<=C1_POST;end
                        else begin ky<=ky+1'b1;state<=C1_LOAD;end
                    end else begin kx<=kx+1'b1;state<=C1_LOAD;end
                end
                C1_POST:begin
                    if(activated>pooled_max) pooled_max<=activated;
                    if(sub==3) state<=C1_WRITE;
                    else begin sub<=sub+1'b1;state<=C1_INIT;end
                end
                C1_WRITE:begin
                    sub<=0;
                    if(out_ch==3) begin
                        out_ch<=0;
                        if(out_x==13) begin
                            out_x<=0;
                            if(out_y==13) begin out_y<=0;state<=C2_INIT;end
                            else begin out_y<=out_y+1'b1;state<=C1_INIT;end
                        end else begin out_x<=out_x+1'b1;state<=C1_INIT;end
                    end else begin out_ch<=out_ch+1'b1;state<=C1_INIT;end
                end
                C2_INIT:begin
                    sum<=bias;kx<=0;ky<=0;in_ch<=0;
                    if(sub==0) pooled_max<=0;
                    state<=C2_LOAD;
                end
                C2_LOAD:state<=C2_ACC;
                C2_ACC:begin
                    if(c2_inside) sum<=sum+$signed({1'b0,fmap1_q})*weight;
                    if(kx==2) begin
                        kx<=0;
                        if(ky==2) begin
                            ky<=0;
                            if(in_ch==3) begin in_ch<=0;state<=C2_POST;end
                            else begin in_ch<=in_ch+1'b1;state<=C2_LOAD;end
                        end else begin ky<=ky+1'b1;state<=C2_LOAD;end
                    end else begin kx<=kx+1'b1;state<=C2_LOAD;end
                end
                C2_POST:begin
                    if(activated>pooled_max) pooled_max<=activated;
                    if(sub==3) state<=C2_WRITE;
                    else begin sub<=sub+1'b1;state<=C2_INIT;end
                end
                C2_WRITE:begin
                    sub<=0;
                    if(out_ch==7) begin
                        out_ch<=0;
                        if(out_x==6) begin
                            out_x<=0;
                            if(out_y==6) begin
                                out_y<=0;scan_class<=0;state<=SCORE_INIT;
                            end else begin out_y<=out_y+1'b1;state<=C2_INIT;end
                        end else begin out_x<=out_x+1'b1;state<=C2_INIT;end
                    end else begin out_ch<=out_ch+1'b1;state<=C2_INIT;end
                end
                SCORE_INIT:begin
                    if(scan_class==10) begin
                        other_score<=bias;
                        gap_ch<=0;gap_qx<=0;gap_qy<=0;gap_dx<=0;gap_dy<=0;gap_sum<=0;
                        state<=GAP_LOAD;
                    end else scan_class<=scan_class+1'b1;
                end
                GAP_LOAD:state<=GAP_ACC;
                GAP_ACC:begin
                    gap_sum<=gap_sum+fmap2_q;
                    if(gap_dx+1==pool_length(gap_qx)) begin
                        gap_dx<=0;
                        if(gap_dy+1==pool_length(gap_qy)) begin gap_dy<=0;state<=GAP_POST;end
                        else begin gap_dy<=gap_dy+1'b1;state<=GAP_LOAD;end
                    end else begin gap_dx<=gap_dx+1'b1;state<=GAP_LOAD;end
                end
                GAP_POST:begin
                    feature_value<=pool_area==4 ? gap_sum>>2 :
                        pool_area==6 ? gap_sum/6 : gap_sum/9;
                    gap_sum<=0;class_index<=0;
                    state<=FC_LOAD;
                end
                FC_LOAD:state<=FC_ACC;
                FC_ACC:begin
                    if(class_index==10) other_score<=score_write_data;
                    if(class_index==10) begin
                        if(gap_qx==3) begin
                            gap_qx<=0;
                            if(gap_qy==3) begin
                                gap_qy<=0;
                                if(gap_ch==7) state<=FIND_INIT;
                                else begin gap_ch<=gap_ch+1'b1;state<=GAP_LOAD;end
                            end else begin gap_qy<=gap_qy+1'b1;state<=GAP_LOAD;end
                        end else begin gap_qx<=gap_qx+1'b1;state<=GAP_LOAD;end
                    end else begin class_index<=class_index+1'b1;state<=FC_LOAD;end
                end
                FIND_INIT:begin
                    scan_class<=0;best_score<=-32'sd2147483647;
                    runner_score<=-32'sd2147483647;best_digit<=0;state<=FIND_SCAN;
                end
                FIND_SCAN:begin
                    if(score_q>best_score) begin
                        runner_score<=best_score;best_score<=score_q;best_digit<=scan_class;
                    end else if(score_q>runner_score) runner_score<=score_q;
                    if(scan_class==9) state<=FINISH;
                    else scan_class<=scan_class+1'b1;
                end
                FINISH:begin
                    digit_valid<=margin_score>=$signed(margin);
                    digit_value<=best_digit;
                    confidence<=margin_score>=$signed(margin<<1)?2'd3:
                        margin_score>=$signed(margin)?2'd2:2'd0;
                    done<=1;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
