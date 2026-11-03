`timescale 1ns/1ps

// Streaming 3x3 window generator with two line memories.
module Window_3x3 #(
    parameter DATA_WIDTH = 8,
    parameter ACTIVE_WIDTH = 1280,
    parameter ACTIVE_HEIGHT = 720
)(
    input wire clk, input wire rst_n,
    input wire input_vsync, input wire input_de,
    input wire [DATA_WIDTH-1:0] input_data,
    output reg output_vsync, output reg output_de, output reg window_valid,
    output reg [11:0] output_x, output reg [11:0] output_y,
    output reg [DATA_WIDTH-1:0] p00,p01,p02,p10,p11,p12,p20,p21,p22
);
    // Alternating row banks match the Efinity synchronous block-RAM template.
    reg [DATA_WIDTH-1:0] even_row_memory [0:ACTIVE_WIDTH-1];
    reg [DATA_WIDTH-1:0] odd_row_memory  [0:ACTIVE_WIDTH-1];
    reg [DATA_WIDTH-1:0] even_row_q, odd_row_q, input_data_q;
    reg [11:0] x_count, y_count, x_q, y_q;
    reg de_q, vsync_q, de_previous;

    // Memories intentionally have no reset; window_valid masks stale rows.
    // Both banks are read, while only the bank belonging to the current row is
    // written. This avoids a memory-to-memory write path and infers BRAM.
    always @(posedge clk) begin
        if (input_vsync && input_de && x_count < ACTIVE_WIDTH) begin
            even_row_q <= even_row_memory[x_count];
            odd_row_q <= odd_row_memory[x_count];
            if (y_count[0]) odd_row_memory[x_count] <= input_data;
            else even_row_memory[x_count] <= input_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_count<=0; y_count<=0; x_q<=0; y_q<=0;
            de_q<=0; vsync_q<=0; de_previous<=0; input_data_q<=0;
        end else begin
            de_previous <= input_de;
            if (!input_vsync) begin x_count<=0; y_count<=0; end
            else begin
                if (input_de) begin
                    if (x_count < ACTIVE_WIDTH-1) x_count <= x_count + 1'b1;
                end else x_count <= 0;
                if (de_previous && !input_de && y_count < ACTIVE_HEIGHT-1)
                    y_count <= y_count + 1'b1;
            end
            input_data_q<=input_data; x_q<=x_count; y_q<=y_count;
            de_q<=input_de && input_vsync; vsync_q<=input_vsync;
        end
    end

    wire [DATA_WIDTH-1:0] middle_row_q = y_q[0] ? even_row_q : odd_row_q;
    wire [DATA_WIDTH-1:0] top_row_q    = y_q[0] ? odd_row_q  : even_row_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p00<=0;p01<=0;p02<=0;p10<=0;p11<=0;p12<=0;p20<=0;p21<=0;p22<=0;
            output_vsync<=0;output_de<=0;window_valid<=0;output_x<=0;output_y<=0;
        end else begin
            output_vsync<=vsync_q; output_de<=de_q;
            window_valid<=de_q && (x_q>=2) && (y_q>=2);
            output_x<=x_q; output_y<=y_q;
            if (de_q) begin
                p00<=p01;p01<=p02;p02<=top_row_q;
                p10<=p11;p11<=p12;p12<=middle_row_q;
                p20<=p21;p21<=p22;p22<=input_data_q;
            end else begin
                p00<=0;p01<=0;p02<=0;p10<=0;p11<=0;p12<=0;p20<=0;p21<=0;p22<=0;
            end
        end
    end
endmodule

// Center-only sideband for the same full-width raster as Window_3x3. One
// READ_FIRST row plus two horizontal registers matches p11 on valid windows.
// Keep optional sidebands in separate instances so unused lanes can be pruned
// before memory packing. No reset on RAM: the caller must use window_valid.
module Window_Center_Delay #(
    parameter DATA_WIDTH=16, parameter ACTIVE_WIDTH=1280
)(
    input wire clk, input wire rst_n,
    input wire input_vsync, input wire input_de,
    input wire [DATA_WIDTH-1:0] input_data,
    output reg [DATA_WIDTH-1:0] center
);
    reg [DATA_WIDTH-1:0] row_memory [0:ACTIVE_WIDTH-1];
    reg [DATA_WIDTH-1:0] row_q, right;
    reg [11:0] x_count;
    reg de_q;
    always @(posedge clk) begin
        if (input_vsync && input_de && x_count < ACTIVE_WIDTH) begin
            row_q <= row_memory[x_count];
            row_memory[x_count] <= input_data;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin x_count<=0;de_q<=0;right<=0;center<=0;end
        else begin
            if (!input_vsync || !input_de) x_count<=0;
            else if (x_count < ACTIVE_WIDTH-1) x_count<=x_count+1'b1;
            de_q<=input_vsync && input_de;
            if (de_q) begin right<=row_q;center<=right;end
            else begin right<=0;center<=0;end
        end
    end
endmodule
