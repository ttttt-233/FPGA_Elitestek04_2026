`timescale 1ns/1ps

// Coherent low-rate status transfer. The bundled data stays fixed until the
// destination acknowledges; rapid changes coalesce to the most recent state.
module Video_Status_Sync #(
    parameter integer WIDTH = 21,
    parameter [WIDTH-1:0] DEFAULT_STATUS = 21'h30600
)(
    input wire rst_n,
    input wire source_clk,
    input wire [WIDTH-1:0] source_status,
    input wire pixel_clk,
    output reg [WIDTH-1:0] pixel_status
);
    reg [WIDTH-1:0] payload_hold;
    reg request_toggle, acknowledge_toggle;
    (* async_reg = "true" *) reg [1:0] acknowledge_sync;
    (* async_reg = "true" *) reg [2:0] request_sync;

    always @(posedge source_clk or negedge rst_n) begin
        if (!rst_n) begin
            payload_hold <= DEFAULT_STATUS;
            request_toggle <= 0;
            acknowledge_sync <= 0;
        end else begin
            acknowledge_sync <= {acknowledge_sync[0], acknowledge_toggle};
            if (request_toggle == acknowledge_sync[1] && payload_hold != source_status) begin
                payload_hold <= source_status;
                request_toggle <= ~request_toggle;
            end
        end
    end

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            request_sync <= 0;
            acknowledge_toggle <= 0;
            pixel_status <= DEFAULT_STATUS;
        end else begin
            request_sync <= {request_sync[1:0], request_toggle};
            if (request_sync[2] != acknowledge_toggle) begin
                pixel_status <= payload_hold;
                acknowledge_toggle <= request_sync[2];
            end
        end
    end
endmodule
