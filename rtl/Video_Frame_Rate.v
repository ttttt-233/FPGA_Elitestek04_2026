`timescale 1ns/1ps
// Measure complete-result publications, not HDMI VSYNC. Optionally average
// 2**AVERAGE_LOG2 intervals. Sum first, then divide; never average reciprocals.
// The average period is rounded down by <1 clock before the 32-cycle division.
module Video_Frame_Rate #(parameter integer CLOCK_HZ=96000000,AVERAGE_LOG2=0)(
    input wire clk,rst_n,frame_done,
    output reg [9:0] fps_x10
);
    localparam [31:0] NUMERATOR=CLOCK_HZ*10;
    localparam [31:0] TIMEOUT=CLOCK_HZ*10;
    localparam integer COUNT_BITS=AVERAGE_LOG2>0?AVERAGE_LOG2:1;
    localparam integer WINDOW=1<<AVERAGE_LOG2;
    reg [31:0] interval,divisor,quotient;
    reg [COUNT_BITS-1:0] sample_count;
    reg [31+AVERAGE_LOG2:0] window_cycles;
    wire [31+AVERAGE_LOG2:0] window_total=window_cycles+{{AVERAGE_LOG2{1'b0}},interval}+1'b1;
    wire [31:0] average_period=window_total>>AVERAGE_LOG2;
    reg [32:0] remainder;
    reg [5:0] steps;
    reg seen,busy;
    wire [32:0] shifted={remainder[31:0],quotient[31]};
    wire subtract=shifted>={1'b0,divisor};
    wire [31:0] next_quotient={quotient[30:0],subtract};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            interval<=0;divisor<=1;quotient<=0;remainder<=0;
            steps<=0;seen<=0;busy<=0;fps_x10<=0;
            sample_count<=0;window_cycles<=0;
        end else begin
            if(interval<TIMEOUT) interval<=interval+1'b1;
            if(busy) begin
                remainder<=subtract?shifted-{1'b0,divisor}:shifted;
                quotient<=next_quotient;
                if(steps==31) begin
                    busy<=0;
                    fps_x10<=next_quotient>999?10'd999:next_quotient[9:0];
                end else steps<=steps+1'b1;
            end
            if(interval>=TIMEOUT) begin
                fps_x10<=0;seen<=0;busy<=0;sample_count<=0;window_cycles<=0;
            end
            if(frame_done) begin
                interval<=0;seen<=1;
                if(seen) begin
                    window_cycles<=window_total;sample_count<=sample_count+1'b1;
                    if(sample_count==WINDOW-1) begin
                        window_cycles<=0;sample_count<=0;
                        if(!busy) begin
                            divisor<=average_period;quotient<=NUMERATOR;remainder<=0;
                            steps<=0;busy<=1;
                        end
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial if(AVERAGE_LOG2<0 || AVERAGE_LOG2>8 || CLOCK_HZ<=0 || CLOCK_HZ>429496729)
        $fatal(1,"Invalid FPS parameters");
    // synthesis translate_on
endmodule
