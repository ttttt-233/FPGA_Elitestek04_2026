`timescale 1ns/1ps
// Measure actual raw camera FRAME_VALID edges in the system clock domain.
// Unlike capture completion this counts the sensor even if processing skips a
// frame. FRAME_VALID is high/low for many system clocks on both profiles.
module Video_Input_Frame_Rate #(parameter integer CLOCK_HZ=96000000,AVERAGE_LOG2=4)(
    input wire clk,rst_n,camera_vsync,
    output wire [9:0] fps_x10,
    output reg [31:0] period_cycles,
    output reg [31:0] frame_count
);
    (* async_reg="true" *) reg [2:0] vsync_sync;
    reg seen;
    reg [31:0] interval;
    wire frame_edge=vsync_sync[1] && !vsync_sync[2];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin vsync_sync<=0;interval<=0;period_cycles<=0;frame_count<=0;seen<=0;end
        else begin
            vsync_sync<={vsync_sync[1:0],camera_vsync};
            if(interval!=32'hffffffff)interval<=interval+1'b1;
            if(frame_edge) begin
                interval<=0;frame_count<=frame_count+1'b1;seen<=1;
                if(seen)period_cycles<=interval+1'b1;
            end
        end
    end
    Video_Frame_Rate #(.CLOCK_HZ(CLOCK_HZ),.AVERAGE_LOG2(AVERAGE_LOG2)) rate (
        .clk(clk),.rst_n(rst_n),.frame_done(frame_edge),.fps_x10(fps_x10));
endmodule
