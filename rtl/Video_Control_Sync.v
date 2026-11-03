`timescale 1ns/1ps

// Transfer one coherent mode/algorithm/threshold bundle using a request/acknowledge
// handshake. The source payload remains fixed until the camera acknowledges it.
// Rapid requests may coalesce, but the latest request is eventually delivered.
// Apply pending controls only when BOTH ends of the image pipeline are idle.
module Video_Control_Sync #(
    parameter [2:0] DEFAULT_MODE = 3'd3,
    parameter DEFAULT_EDGE_SELECT = 1'b0,
    parameter [11:0] DEFAULT_THRESHOLD = 12'd96,
    parameter RUNTIME_FILTER = 0
)(
    input wire rst_n,
    input wire source_clk,
    input wire [2:0] mode_request,
    input wire edge_request,
    input wire [11:0] threshold_request,
    input wire adaptive_request,
    input wire pixel_clk,
    input wire frame_idle,
    output reg [2:0] mode_active,
    output reg edge_active,
    output reg [11:0] threshold_active,
    output reg adaptive_active,
    input wire gaussian_request,
    output reg gaussian_active
);
    reg [17:0] payload_hold;
    wire requested_filter=RUNTIME_FILTER && gaussian_request;
    reg request_toggle;
    reg acknowledge_toggle;
    (* async_reg = "true" *) reg [1:0] acknowledge_sync;
    (* async_reg = "true" *) reg [2:0] request_sync;
    reg [17:0] pending_controls;

    always @(posedge source_clk or negedge rst_n) begin
        if (!rst_n) begin
            acknowledge_sync <= 0;
            payload_hold <= {1'b0, DEFAULT_MODE, DEFAULT_EDGE_SELECT, DEFAULT_THRESHOLD, 1'b0};
            request_toggle <= 0;
        end else begin
            acknowledge_sync <= {acknowledge_sync[0], acknowledge_toggle};
            if (request_toggle == acknowledge_sync[1] &&
                payload_hold != {requested_filter, mode_request, edge_request, threshold_request, adaptive_request}) begin
                payload_hold <= {requested_filter, mode_request, edge_request, threshold_request, adaptive_request};
                request_toggle <= ~request_toggle;
            end
        end
    end

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            request_sync <= 0;
            acknowledge_toggle <= 0;
            pending_controls <= {1'b0, DEFAULT_MODE, DEFAULT_EDGE_SELECT, DEFAULT_THRESHOLD, 1'b0};
            mode_active <= DEFAULT_MODE;
            edge_active <= DEFAULT_EDGE_SELECT;
            threshold_active <= DEFAULT_THRESHOLD;
            adaptive_active <= 1'b0;
            gaussian_active <= 1'b0;
        end else begin
            // Three destination clocks allow the held multi-bit payload to
            // settle before it is sampled. Only the toggle passes through FFs.
            request_sync <= {request_sync[1:0], request_toggle};
            if (request_sync[2] != acknowledge_toggle) begin
                pending_controls <= payload_hold;
                acknowledge_toggle <= request_sync[2];
            end
            if (frame_idle) begin
                gaussian_active <= pending_controls[17];
                mode_active <= pending_controls[16:14];
                edge_active <= pending_controls[13];
                threshold_active <= pending_controls[12:1];
                adaptive_active <= pending_controls[0];
            end
        end
    end
endmodule
