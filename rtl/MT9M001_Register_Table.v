`timescale 1ns/1ps
// Board wiring/crop starts at GBRG (BAYER_PATTERN=1).
module MT9M001_Register_Table #(
    parameter [15:0] ACTIVE_WIDTH = 16'd1280,
    parameter [15:0] ACTIVE_HEIGHT = 16'd720,
    parameter [15:0] ROW_START = 16'd164,
    parameter [15:0] COLUMN_START = 16'd20,
    parameter [15:0] HORIZONTAL_BLANKING = 16'd19,
    parameter [15:0] VERTICAL_BLANKING = 16'd25,
    parameter ROW_MIRROR = 0,
    parameter COLUMN_MIRROR = 0,
    parameter [15:0] EXPOSURE_LINES = 16'd720,
    parameter [15:0] ANALOG_GAIN = 16'h0008 // 1x; 0x0020 is 4x
)(
    input [7:0] LUT_INDEX,
    output reg [23:0] LUT_DATA,
    output [7:0] LUT_SIZE
);
    assign LUT_SIZE = 8'd13;
    always @(*) begin
        case (LUT_INDEX)
            0: LUT_DATA={8'h0D,16'h0001}; // Software reset
            1: LUT_DATA={8'h0D,16'h0000}; // Release reset (1 ms later)
            2: LUT_DATA={8'h07,16'h0000}; // Disable output while configuring
            3: LUT_DATA={8'h01,ROW_START};
            4: LUT_DATA={8'h02,COLUMN_START};
            5: LUT_DATA={8'h03,ACTIVE_HEIGHT-1'b1};
            6: LUT_DATA={8'h04,ACTIVE_WIDTH-1'b1};
            7: LUT_DATA={8'h05,HORIZONTAL_BLANKING}; // Minimum horizontal blanking
            8: LUT_DATA={8'h06,VERTICAL_BLANKING}; // Vertical blanking
            // Only use the documented mirror bits. No legacy 0x1104 read-mode
            // flags are mixed in. Bit 15 reverses rows; bit 14 reverses columns.
            9: LUT_DATA={8'h20,((ROW_MIRROR    ? 16'h8000 : 16'h0000) |
                                (COLUMN_MIRROR ? 16'h4000 : 16'h0000))};
            10:LUT_DATA={8'h09,EXPOSURE_LINES};
            11:LUT_DATA={8'h35,ANALOG_GAIN};
            12:LUT_DATA={8'h07,16'h0002}; // Enable continuous output
            default: LUT_DATA=24'd0;
        endcase
    end
endmodule
