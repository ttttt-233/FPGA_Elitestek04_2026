`timescale 1ns/1ps
// MT9M001: 8-bit register address, 16-bit value, BA write address.
// SDA is open-drain. Every byte is ACK-checked; a NACK retries the same LUT
// entry after 1 ms rather than falsely reporting successful initialization.
module MT9M001_I2C_Master #(
    parameter CLK_FREQ = 96000000,
    parameter I2C_FREQ = 50000,
    parameter STARTUP_CYCLES = CLK_FREQ / 100,
    parameter GAP_CYCLES = CLK_FREQ / 1000
)(
    input clk, input rst_n,
    output reg i2c_sclk,
    input i2c_sdat_i, output i2c_sdat_o, output i2c_sdat_oe,
    input [23:0] config_data, input [7:0] config_size,
    output reg [7:0] config_index,
    output config_done, output reg config_error
);
    localparam DIV = CLK_FREQ / (I2C_FREQ * 4);
    localparam WAIT=0, START=1, LOW=2, RISE=3, HIGH=4, FALL=5,
        ACK_LOW=6, ACK_RISE=7, ACK_SAMPLE=8, ACK_FALL=9,
        STOP_LOW=10, STOP_RISE=11, STOP_RELEASE=12, GAP=13;
    reg [3:0] state;
    reg [31:0] wait_count;
    reg [15:0] divider;
    reg sda_release, nack;
    reg [1:0] byte_index;
    reg [2:0] bit_index;
    reg [31:0] transaction;
    wire [7:0] current_byte = transaction[31:24];
    assign i2c_sdat_o = 1'b0;
    assign i2c_sdat_oe = !sda_release;
    assign config_done = config_index >= config_size;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=WAIT; wait_count<=0; divider<=0; i2c_sclk<=1;
            sda_release<=1; nack<=0; config_error<=0; config_index<=0;
            byte_index<=0; bit_index<=7; transaction<=0;
        end else if (state==WAIT || state==GAP) begin
            divider<=0;
            if (wait_count < ((state==WAIT) ? STARTUP_CYCLES : GAP_CYCLES)) begin
                wait_count<=wait_count+1'b1;
            end else if (!config_done) begin
                wait_count<=0; transaction<={8'hBA,config_data};
                byte_index<=0; bit_index<=7; nack<=0; state<=START;
            end
        end else if (divider==DIV-1) begin
            divider<=0;
            case (state)
                START: begin i2c_sclk<=1; sda_release<=0; state<=LOW; end
                LOW: begin i2c_sclk<=0; sda_release<=current_byte[bit_index]; state<=RISE; end
                RISE: begin i2c_sclk<=1; state<=HIGH; end
                HIGH: state<=FALL;
                FALL: begin
                    i2c_sclk<=0;
                    if (bit_index==0) state<=ACK_LOW;
                    else begin bit_index<=bit_index-1'b1; state<=LOW; end
                end
                ACK_LOW: begin i2c_sclk<=0; sda_release<=1; state<=ACK_RISE; end
                ACK_RISE: begin i2c_sclk<=1; state<=ACK_SAMPLE; end
                ACK_SAMPLE: begin nack<=nack | i2c_sdat_i; state<=ACK_FALL; end
                ACK_FALL: begin
                    i2c_sclk<=0;
                    if (byte_index==3) state<=STOP_LOW;
                    else begin
                        transaction<={transaction[23:0],8'd0};
                        byte_index<=byte_index+1'b1; bit_index<=7; state<=LOW;
                    end
                end
                STOP_LOW: begin i2c_sclk<=0; sda_release<=0; state<=STOP_RISE; end
                STOP_RISE: begin i2c_sclk<=1; state<=STOP_RELEASE; end
                STOP_RELEASE: begin
                    sda_release<=1; config_error<=nack;
                    if (!nack) config_index<=config_index+1'b1;
                    wait_count<=0; state<=GAP;
                end
                default: state<=WAIT;
            endcase
        end else divider<=divider+1'b1;
    end
endmodule
