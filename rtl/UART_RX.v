`timescale 1ns/1ps
// Mid-bit 8N1 receiver. The free-running baud timer also clocks the parser's
// short timeout; no second wide timeout counter or FIFO is needed.
module UART_RX #(parameter CLOCK_HZ=96000000,BAUD=2000000)(
    input wire clk,rst_n,rx,
    output wire [7:0] data,
    output reg valid,error,bit_tick
);
    localparam DIV=CLOCK_HZ/BAUD,TB=DIV<2?1:$clog2(DIV);
    localparam IDLE=0,START=1,DATA=2,STOP=3;
    (* async_reg="true" *) reg [1:0] sync_rx;
    reg [1:0] state;
    reg [TB-1:0] timer;
    reg [2:0] bit_index;
    reg [7:0] shift;
    assign data=shift;
    always @(posedge clk) if(state==DATA && timer==0) shift<={sync_rx[1],shift[7:1]};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sync_rx<=3;state<=IDLE;timer<=0;bit_index<=0;
            valid<=0;error<=0;bit_tick<=0;
        end else begin
            sync_rx<={sync_rx[0],rx};valid<=0;error<=0;bit_tick<=0;
            if(state==IDLE && !sync_rx[1]) begin timer<=DIV/2-1;state<=START;end
            else if(timer!=0) timer<=timer-1'b1;
            else begin
                timer<=DIV-1;bit_tick<=1;
                case(state)
                    START:if(sync_rx[1]) begin error<=1;state<=IDLE;end
                        else begin bit_index<=0;state<=DATA;end
                    DATA:begin
                        if(bit_index==7) state<=STOP;else bit_index<=bit_index+1'b1;
                    end
                    STOP:begin valid<=sync_rx[1];error<=!sync_rx[1];state<=IDLE;end
                    default:state<=IDLE;
                endcase
            end
        end
    end
    // synthesis translate_off
    initial if(DIV<8) $fatal(1,"UART RX needs at least eight clocks per bit");
    // synthesis translate_on
endmodule
