`timescale 1ns/1ps
// Four local component summaries. No rank, final descriptor, CNN state or
// metadata is duplicated here. Comparison and eviction policy live in the
// detector's serial state machine; removal shifts one fixed neighbour/cycle.
module Shape_Digit_Merge_Window(
    input wire clk,rst_n,clear,append,shift,pop,
    input wire [2:0] shift_index,read_index,
    input wire [47:0] input_box,
    input wire [1:0] input_geometry,input_grade,
    input wire [19:0] input_area,
    output reg [2:0] count,
    output reg [47:0] read_box,
    output reg [1:0] read_geometry,read_grade,
    output reg [19:0] read_area,
    output wire [47:0] head_box,
    output wire [1:0] head_geometry,head_grade
);
    reg [47:0] boxes[0:3];
    reg [1:0] geometry[0:3],grades[0:3];
    reg [19:0] areas[0:3];
    assign head_box=boxes[0];
    assign head_geometry=geometry[0];
    assign head_grade=grades[0];
    always @(posedge clk or negedge rst_n)
        if(!rst_n) count<=0;
        else if(clear) count<=0;
        else if(append) count<=count+1'b1;
        else if(pop) count<=count-1'b1;
    integer n;
    always @(*) begin
        read_box=0;read_geometry=0;read_grade=0;read_area=0;
        for(n=0;n<4;n=n+1) if(read_index==n && n<count) begin
            read_box=boxes[n];read_geometry=geometry[n];
            read_grade=grades[n];read_area=areas[n];
        end
    end
    genvar g;
    generate for(g=0;g<4;g=g+1) begin: g_slot
        if(g<3) begin: g_shift
            always @(posedge clk)
                if(append && count==g) begin
                    boxes[g]<=input_box;geometry[g]<=input_geometry;
                    grades[g]<=input_grade;areas[g]<=input_area;
                end else if(shift && shift_index==g) begin
                    boxes[g]<=boxes[g+1];geometry[g]<=geometry[g+1];
                    grades[g]<=grades[g+1];areas[g]<=areas[g+1];
                end
        end else begin: g_tail
            always @(posedge clk) if(append && count==g) begin
                boxes[g]<=input_box;geometry[g]<=input_geometry;
                grades[g]<=input_grade;areas[g]<=input_area;
            end
        end
    end endgenerate
endmodule
