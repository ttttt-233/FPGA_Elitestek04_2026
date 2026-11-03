`timescale 1ns/1ps

// Small fixed-content OSD using an 8x16 font (8x8 glyphs doubled vertically).
// Only glyphs required by the first UI are implemented, avoiding a full ASCII
// font ROM. The panel is intentionally opaque for readability over live video.
module Video_OSD #(
    parameter integer OSD_X = 16,
    parameter integer OSD_Y = 16,
    parameter integer FPS_VALUE = 30,
    parameter integer LIVE_FPS = 0,
    parameter integer PROFILE_ENABLE = 0,
    parameter integer PROFILE_INTERVAL = 0,
    parameter integer OUT_PROFILE_ENABLE = 0,
    parameter integer DIAGNOSTIC_ENABLE = 0,
    parameter integer EDGE_DIAGNOSTIC_ENABLE = 0,
    parameter integer STARTUP_DIAGNOSTIC_ENABLE = 0,
    parameter integer FRAME_BLANK_ENABLE = 0,
    parameter integer MENU_ENABLE = 0,
    parameter integer RUNTIME_FILTER = 0,
    parameter [1:0] BUILD_PROFILE_ID = 2'd0,
    parameter integer PROFILE_MENU_ENABLE = 0,
    parameter integer PHOTO_CAPACITY = 8,
    parameter integer SCREEN_WIDTH = 1280,
    parameter integer SCREEN_HEIGHT = 720,
    // 74.25 MHz pixel clock: feedback 0.5 s, border 0.15 s.
    parameter integer FEEDBACK_COUNT = 37_125_000,
    parameter integer BORDER_COUNT = 11_137_500,
    parameter integer FEEDBACK_COUNTER_WIDTH = 26,
    parameter integer SHAPE_ENABLE = 0,
    parameter integer SHAPE_TARGETS = 1,
    parameter [23:0] FOREGROUND_COLOR = 24'hffffff,
    parameter [23:0] BACKGROUND_COLOR = 24'h101820
)(
    input  wire        pixel_clk,
    input  wire        rst_n,
    input  wire        input_hsync,
    input  wire        input_vsync,
    input  wire        frame_blanking,
    input  wire        input_de,
    input  wire [11:0] input_x,
    input  wire [11:0] input_y,
    input  wire [23:0] input_rgb888,
    input  wire        edge_select,
    input  wire [1:0]  shape_type,
    input  wire [1:0]  shape_type_second,
    input  wire        osd_enable,
    input  wire        control_notice,
    input  wire [2:0]  control_group,
    input  wire [2:0]  osd_mask,
    input  wire [1:0]  control_item,
    input  wire [3:0]  reserved_action,
    input  wire [2:0]  display_mode,
    input  wire        osd_all_enable,
    input  wire [11:0] manual_threshold,
    input  wire        filter_active,
    input  wire        gaussian_select,
    input  wire        shape_enable,
    input  wire [1:0]  requested_profile,
    input  wire [3:0]  profile_switch_state,
    // Future media backend, already coherent in pixel_clk:
    // [3:0] SNAP count; [4] recording; [16:5] REC NN.N BCD;
    // [20:17] media index; [24:21] media count; [25] video;
    // [26] playing; [38:27] elapsed NN.N BCD; [50:39] total NN.N BCD;
    // [51] media valid; [54:52] SNAP/count/index high bits. No capture/playback implementation lives in this OSD.
    input  wire [54:0] media_status,
    // One-cycle success events: 1 SNAP, 2 PRE, 3 CLIP, 4 PLAY,
    // 5 PAUSE, 6 VIDEO END. Requests alone never generate feedback.
    input  wire [2:0]  feedback_event,
    input  wire        auto_threshold_enable,
    input  wire [11:0] low_threshold,
    input  wire [11:0] high_threshold,
    input  wire [9:0]  measured_fps_x10,
    input  wire [9:0]  camera_fps_x10,
    input  wire [143:0] profile_ms_bcd,
    input  wire profile_valid,
    input  wire [143:0] out_profile_bcd,
    input  wire out_profile_valid,
    input  wire [143:0] schedule_bcd,
    input  wire schedule_valid,
    input  wire [191:0] edge_stats_bcd,
    input  wire edge_stats_valid,
    input  wire [14:0] startup_status,
    output reg         output_hsync,
    output reg         output_vsync,
    output reg         output_de,
    output reg  [23:0] output_rgb888
);
    localparam integer TEXT_COLUMNS = 20;
    localparam integer TEXT_LINES = 7;
    localparam integer TEXT_WIDTH = TEXT_COLUMNS * 8;
    localparam integer TEXT_HEIGHT = TEXT_LINES * 16;
    localparam integer NOTICE_COLUMNS = MENU_ENABLE ? 32 : 24;
    localparam integer NOTICE_HEIGHT = MENU_ENABLE ? 176 : 16;
    localparam integer NOTICE_WIDTH = NOTICE_COLUMNS * 8;
    localparam integer NOTICE_Y = OSD_Y + TEXT_HEIGHT + 8;
    localparam integer FPS_TENS = (FPS_VALUE / 10) % 10;
    localparam integer FPS_ONES = FPS_VALUE % 10;
    localparam [11:0] OSD_X_VALUE = OSD_X;
    localparam [11:0] OSD_Y_VALUE = OSD_Y;
    localparam [11:0] NOTICE_Y_VALUE = NOTICE_Y;

    // Inputs have crossed coherently to pixel_clk before arriving here.
    // Commit text, visibility and playback status only in vertical blanking.
    // This layer is after DDR, so browsing saved media still updates text.
    reg frame_edge,frame_osd,frame_notice,frame_auto;
    reg frame_debug;
    reg frame_gaussian;
    reg [1:0] frame_requested_profile;
    reg [3:0] frame_profile_switch_state;
    reg frame_global,frame_all,frame_filter,frame_context_notice,frame_shape_enable;
    reg [11:0] frame_manual;
    reg [54:0] frame_media;
    reg [2:0] frame_feedback,frame_border;
    reg [2:0] feedback_code;
    reg [FEEDBACK_COUNTER_WIDTH-1:0] feedback_counter,border_counter;
    reg [2:0] frame_mask,frame_mode;
    reg [1:0] frame_item;
    reg [3:0] frame_action;
    reg [2:0] frame_group;
    reg [11:0] frame_low,frame_high;
    reg [9:0] frame_fps,frame_camera_fps;
    reg [143:0] frame_profile;
    reg frame_profile_valid;
    reg [143:0] frame_out_profile;
    reg frame_out_profile_valid;
    reg [143:0] frame_schedule;
    reg frame_schedule_valid;
    reg [191:0] frame_edge_stats;
    reg frame_edge_stats_valid;
    reg [14:0] frame_startup;
    reg [1:0] frame_shape,frame_shape_second;
    always @(posedge pixel_clk or negedge rst_n) begin
        if(!rst_n) begin
            frame_edge<=0;frame_osd<=1;frame_notice<=0;frame_group<=0;
            frame_debug<=!MENU_ENABLE;frame_mask<=3;frame_mode<=0;frame_item<=0;frame_action<=0;
            frame_global<=1;frame_all<=1;frame_filter<=1;frame_manual<=96;frame_context_notice<=0;
            frame_shape_enable<=1;frame_gaussian<=0;
            frame_requested_profile<=BUILD_PROFILE_ID;frame_profile_switch_state<=0;
            frame_media<=0;frame_feedback<=0;frame_border<=0;
            frame_auto<=0;
            frame_low<=32;frame_high<=96;frame_fps<=0;frame_camera_fps<=0;
            frame_profile<=0;frame_profile_valid<=0;
            frame_out_profile<=0;frame_out_profile_valid<=0;
            frame_schedule<=0;frame_schedule_valid<=0;
            frame_edge_stats<=0;frame_edge_stats_valid<=0;
            frame_startup<=0;
            frame_shape<=0;frame_shape_second<=0;
        end else if(!input_vsync || (FRAME_BLANK_ENABLE && frame_blanking)) begin
            frame_edge<=edge_select;
            frame_osd<=osd_enable && (!MENU_ENABLE || osd_mask[0]);
            frame_notice<=MENU_ENABLE ? osd_enable && osd_mask[1] : control_notice;
            frame_debug<=osd_enable && (!MENU_ENABLE || osd_mask[2]);
            frame_global<=osd_enable;frame_all<=osd_all_enable;
            frame_context_notice<=control_notice;
            frame_shape_enable<=shape_enable;
            frame_requested_profile<=PROFILE_MENU_ENABLE?requested_profile:BUILD_PROFILE_ID;
            frame_profile_switch_state<=PROFILE_MENU_ENABLE?profile_switch_state:4'd0;
            frame_gaussian<=RUNTIME_FILTER && gaussian_select;
            frame_filter<=filter_active;frame_manual<=manual_threshold;frame_media<=media_status;
            frame_feedback<=feedback_counter!=0?feedback_code:3'd0;
            frame_border<=border_counter!=0?feedback_code:3'd0;
            frame_mask<=osd_mask;frame_item<=control_item;frame_action<=reserved_action;frame_mode<=display_mode;
            frame_auto<=auto_threshold_enable;
            frame_group<=control_group;frame_low<=low_threshold;frame_high<=high_threshold;
            frame_fps<=LIVE_FPS?measured_fps_x10:10'd0;
            frame_camera_fps<=PROFILE_MENU_ENABLE?camera_fps_x10:10'd0;
            frame_profile<=PROFILE_ENABLE?profile_ms_bcd:144'd0;
            frame_profile_valid<=PROFILE_ENABLE && profile_valid;
            frame_out_profile<=OUT_PROFILE_ENABLE?out_profile_bcd:144'd0;
            frame_out_profile_valid<=OUT_PROFILE_ENABLE && out_profile_valid;
            frame_schedule<=DIAGNOSTIC_ENABLE?schedule_bcd:144'd0;
            frame_schedule_valid<=DIAGNOSTIC_ENABLE && schedule_valid;
            frame_edge_stats<=EDGE_DIAGNOSTIC_ENABLE?edge_stats_bcd:192'd0;
            frame_edge_stats_valid<=EDGE_DIAGNOSTIC_ENABLE && edge_stats_valid;
            frame_startup<=STARTUP_DIAGNOSTIC_ENABLE?startup_status:15'd0;
            frame_shape<=SHAPE_ENABLE?shape_type:2'd0;
            frame_shape_second<=SHAPE_ENABLE && SHAPE_TARGETS==2?shape_type_second:2'd0;
        end
    end

    // Display-only timers. Events acknowledge the media UI state. Events in
    // the other four levels are discarded, so they can never create a toast.
    always @(posedge pixel_clk or negedge rst_n) begin
        if(!rst_n) begin feedback_counter<=0;border_counter<=0;feedback_code<=0;end
        else if(!MENU_ENABLE || control_group>=2) begin
            feedback_counter<=0;border_counter<=0;feedback_code<=0;
        end else if((control_group==0 && feedback_event>=1 && feedback_event<=3) ||
                    (control_group==1 && feedback_event>=4 && feedback_event<=6)) begin
            feedback_code<=feedback_event;
            feedback_counter<=FEEDBACK_COUNT;
            border_counter<=(feedback_event==1 || feedback_event==2)?BORDER_COUNT:0;
        end else begin
            if(feedback_counter!=0) feedback_counter<=feedback_counter-1'b1;
            if(border_counter!=0) border_counter<=border_counter-1'b1;
        end
    end

    wire panel_active = !MENU_ENABLE && frame_osd && input_de &&
                        input_x >= OSD_X-4 && input_x < OSD_X+TEXT_WIDTH+4 &&
                        input_y >= OSD_Y-4 && input_y < OSD_Y+TEXT_HEIGHT+4;
    wire text_active = !MENU_ENABLE && frame_osd && input_de &&
                       input_x >= OSD_X && input_x < OSD_X+TEXT_WIDTH &&
                       input_y >= OSD_Y && input_y < OSD_Y+TEXT_HEIGHT;
    wire notice_panel_active = !MENU_ENABLE && frame_notice && input_de &&
                               input_x >= OSD_X-4 && input_x < OSD_X+NOTICE_WIDTH+4 &&
                               input_y >= NOTICE_Y-4 && input_y < NOTICE_Y+NOTICE_HEIGHT+4;
    wire notice_text_active = !MENU_ENABLE && frame_notice && input_de &&
                              input_x >= OSD_X && input_x < OSD_X+NOTICE_WIDTH &&
                              input_y >= NOTICE_Y && input_y < NOTICE_Y+NOTICE_HEIGHT;
    wire [11:0] text_x = input_x - OSD_X_VALUE;
    wire [11:0] text_y = input_y - OSD_Y_VALUE;
    wire [4:0] character_column = text_x[7:3];
    wire [2:0] text_line = text_y[6:4];
    wire [2:0] glyph_column = text_x[2:0];
    wire [2:0] glyph_row = text_y[3:1];
    wire [11:0] notice_x = input_x - OSD_X_VALUE;
    wire [11:0] notice_y = input_y - NOTICE_Y_VALUE;
    wire [4:0] notice_column = notice_x[7:3];
    wire [2:0] notice_glyph_column = notice_x[2:0];
    wire [2:0] notice_glyph_row = notice_y[3:1];
    localparam integer DEBUG_COLUMNS=(SCREEN_WIDTH-OSD_X-256-32)/144 >= 4 ? 4 : 2;
    localparam integer DEBUG_X=SCREEN_WIDTH-16-DEBUG_COLUMNS*144;
    localparam integer LOWER_DEBUG_Y=OSD_Y+160;
    localparam integer EDGE_STATS_Y=MENU_ENABLE && DEBUG_COLUMNS==2?LOWER_DEBUG_Y:OSD_Y;
    localparam integer OUT_PROFILE_Y=MENU_ENABLE && DEBUG_COLUMNS==2?LOWER_DEBUG_Y:OSD_Y;
    localparam integer PROFILE_X=MENU_ENABLE?DEBUG_X:OSD_X+200;
    wire profile_panel_active=PROFILE_ENABLE && frame_debug && input_de &&
        input_x>=PROFILE_X-4 && input_x<PROFILE_X+132 &&
        input_y>=OSD_Y-4 && input_y<OSD_Y+116;
    wire profile_text_active=PROFILE_ENABLE && frame_debug && input_de &&
        input_x>=PROFILE_X && input_x<PROFILE_X+128 &&
        input_y>=OSD_Y && input_y<OSD_Y+112;
    wire [11:0] profile_x=input_x-PROFILE_X;
    wire [3:0] profile_column=profile_x[6:3];
    wire [2:0] profile_line=text_y[6:4];
    wire [2:0] profile_index=profile_line==0?3'd0:profile_line-3'd1;
    wire [3:0] profile_wait_index={1'b0,profile_index}+4'd6;
    wire [11:0] profile_time=frame_profile[profile_index*12 +: 12];
    wire [11:0] profile_wait=frame_profile[profile_wait_index*12 +: 12];
    localparam integer SCHEDULE_X=MENU_ENABLE?DEBUG_X+144:OSD_X+352;
    localparam integer OUT_PROFILE_X=MENU_ENABLE?DEBUG_X+(DEBUG_COLUMNS==2?144:432):OSD_X+656;
    wire out_profile_panel_active=OUT_PROFILE_ENABLE && frame_debug && input_de &&
        input_x>=OUT_PROFILE_X-4 && input_x<OUT_PROFILE_X+132 &&
        input_y>=OUT_PROFILE_Y-4 && input_y<OUT_PROFILE_Y+116;
    wire out_profile_text_active=OUT_PROFILE_ENABLE && frame_debug && input_de &&
        input_x>=OUT_PROFILE_X && input_x<OUT_PROFILE_X+128 &&
        input_y>=OUT_PROFILE_Y && input_y<OUT_PROFILE_Y+112;
    wire [11:0] out_profile_x=input_x-OUT_PROFILE_X;
    wire [11:0] out_profile_y=input_y-OUT_PROFILE_Y;
    wire [2:0] out_profile_line=out_profile_y[6:4];
    wire [2:0] out_profile_index=out_profile_line==0?3'd0:out_profile_line-1'b1;
    wire [3:0] out_profile_wait_index={1'b0,out_profile_index}+4'd6;
    wire [11:0] out_profile_time=frame_out_profile[out_profile_index*12 +: 12];
    wire [11:0] out_profile_wait=frame_out_profile[out_profile_wait_index*12 +: 12];
    wire schedule_panel_active=DIAGNOSTIC_ENABLE && frame_debug && input_de &&
        input_x>=SCHEDULE_X-4 && input_x<SCHEDULE_X+132 &&
        input_y>=OSD_Y-4 && input_y<OSD_Y+116;
    wire schedule_text_active=DIAGNOSTIC_ENABLE && frame_debug && input_de &&
        input_x>=SCHEDULE_X && input_x<SCHEDULE_X+128 &&
        input_y>=OSD_Y && input_y<OSD_Y+112;
    wire [11:0] schedule_x=input_x-SCHEDULE_X;
    wire [3:0] schedule_index={profile_index,1'b0};
    wire [3:0] schedule_index_b=schedule_index+4'd1;
    wire [11:0] schedule_a=frame_schedule[schedule_index*12 +: 12];
    wire [11:0] schedule_b=frame_schedule[schedule_index_b*12 +: 12];

    localparam integer EDGE_STATS_X=MENU_ENABLE?DEBUG_X+(DEBUG_COLUMNS==2?0:288):OSD_X+504;
    wire edge_stats_panel_active=EDGE_DIAGNOSTIC_ENABLE && frame_debug && input_de &&
        input_x>=EDGE_STATS_X-4 && input_x<EDGE_STATS_X+132 &&
        input_y>=EDGE_STATS_Y-4 && input_y<EDGE_STATS_Y+148;
    wire edge_stats_text_active=EDGE_DIAGNOSTIC_ENABLE && frame_debug && input_de &&
        input_x>=EDGE_STATS_X && input_x<EDGE_STATS_X+128 &&
        input_y>=EDGE_STATS_Y && input_y<EDGE_STATS_Y+144;
    wire [11:0] edge_stats_x=input_x-EDGE_STATS_X;
    wire [11:0] edge_stats_y=input_y-EDGE_STATS_Y;
    wire [3:0] edge_stats_line=edge_stats_y[7:4];
    wire [2:0] edge_stats_index=edge_stats_line==0?3'd0:edge_stats_line-4'd1;
    wire [23:0] edge_stats_value=frame_edge_stats[edge_stats_index*24 +: 24];
    function [7:0] edge_stats_character;
        input [3:0] line_number,column_number;
        input [23:0] value_bcd;
        input valid_report;
        reg [127:0] label_text;
        begin
            case(line_number)
                0:label_text="EDGE COUNT      ";
                1:label_text="CAND            ";
                2:label_text="SEED            ";
                3:label_text="JOIN            ";
                4:label_text="CUT C           ";
                5:label_text="CUT P           ";
                6:label_text="BYP C           ";
                7:label_text="OVER            ";
                8:label_text="OUT P           ";
                default:label_text="                ";
            endcase
            edge_stats_character=label_text[127-column_number*8 -: 8];
            if(line_number!=0 && column_number>=8 && column_number<=13)
                edge_stats_character=valid_report?
                    8'h30+value_bcd[(13-column_number)*4 +: 4]:8'h2d;
        end
    endfunction

    // Convert the live 12-bit thresholds to four decimal digits without a ROM.
    function [15:0] binary_to_bcd;
        input [11:0] binary_value;
        integer i;
        reg [27:0] shift;
        begin
            shift = 28'd0;
            shift[11:0] = binary_value;
            for (i=0; i<12; i=i+1) begin
                if (shift[15:12] >= 5) shift[15:12] = shift[15:12] + 3;
                if (shift[19:16] >= 5) shift[19:16] = shift[19:16] + 3;
                if (shift[23:20] >= 5) shift[23:20] = shift[23:20] + 3;
                if (shift[27:24] >= 5) shift[27:24] = shift[27:24] + 3;
                shift = shift << 1;
            end
            binary_to_bcd = shift[27:12];
        end
    endfunction

    function [7:0] notice_character;
        input [2:0] group_number;
        input [4:0] column_number;
        reg [191:0] label_text;
        begin
            case(group_number)
                0:label_text="CONTROL: CAPTURE        ";
                1:label_text="CONTROL: PLAYBACK       ";
                2:label_text="CONTROL: DISPLAY        ";
                3:label_text="CONTROL: THRESHOLD      ";
                4:label_text="CONTROL: ALGORITHM      ";
                5:label_text="CONTROL: OSD            ";
                default:label_text="                        ";
            endcase
            notice_character=label_text[191-column_number*8 -: 8];
        end
    endfunction

    function [7:0] decimal_character;
        input [15:0] bcd_value;
        input [3:0] column_number;
        begin
            decimal_character = 8'h20;
            case (column_number)
                5: begin
                    if (bcd_value[15:12] != 0)
                        decimal_character = 8'h30 + bcd_value[15:12];
                    else if (bcd_value[11:8] != 0)
                        decimal_character = 8'h30 + bcd_value[11:8];
                end
                6: begin
                    if (bcd_value[15:12] != 0)
                        decimal_character = 8'h30 + bcd_value[11:8];
                    else if (bcd_value[11:8] != 0)
                        decimal_character = 8'h30 + bcd_value[7:4];
                    else if (bcd_value[7:4] != 0)
                        decimal_character = 8'h30 + bcd_value[7:4];
                end
                7: begin
                    if (bcd_value[15:12] != 0)
                        decimal_character = 8'h30 + bcd_value[7:4];
                    else
                        decimal_character = 8'h30 + bcd_value[3:0];
                end
                8: begin
                    if (bcd_value[15:12] != 0)
                        decimal_character = 8'h30 + bcd_value[3:0];
                end
                default: decimal_character = 8'h20;
            endcase
        end
    endfunction

    wire [15:0] low_threshold_bcd = binary_to_bcd(frame_low);
    wire [15:0] high_threshold_bcd = binary_to_bcd(frame_high);


    function [7:0] text_character;
        input [2:0] line_number;
        input [3:0] column_number;
        input       selected_canny;
        input       selected_gaussian;
        input [15:0] low_bcd;
        input [15:0] high_bcd;
        begin
            text_character = 8'h20;
            case (line_number)
                3'd0: begin // MODE: SOBEL / MODE: CANNY
                    case (column_number)
                        0:text_character=8'h4d; 1:text_character=8'h4f;
                        2:text_character=8'h44; 3:text_character=8'h45;
                        4:text_character=8'h3a;
                        6:text_character=selected_canny?8'h43:8'h53;
                        7:text_character=selected_canny?8'h41:8'h4f;
                        8:text_character=selected_canny?8'h4e:8'h42;
                        9:text_character=selected_canny?8'h4e:8'h45;
                        10:text_character=selected_canny?8'h59:8'h4c;
                        default:text_character=8'h20;
                    endcase
                end
                3'd1: begin // LOW:  32
                    case (column_number)
                        0:text_character=8'h4c; 1:text_character=8'h4f;
                        2:text_character=8'h57; 3:text_character=8'h3a;
                        default:text_character=decimal_character(low_bcd,column_number);
                    endcase
                end
                3'd2: begin // HIGH: 96
                    case (column_number)
                        0:text_character=8'h48; 1:text_character=8'h49;
                        2:text_character=8'h47; 3:text_character=8'h48;
                        4:text_character=8'h3a;
                        default:text_character=decimal_character(high_bcd,column_number);
                    endcase
                end
                3'd3: begin // FPS:  30
                    case (column_number)
                        0:text_character=8'h46; 1:text_character=8'h50;
                        2:text_character=8'h53; 3:text_character=8'h3a;
                        6:text_character=8'h30+FPS_TENS;
                        7:text_character=8'h30+FPS_ONES;
                        default:text_character=8'h20;
                    endcase
                end
                3'd4: begin // FILTER: MEDIAN
                    case (column_number)
                        0:text_character=8'h46; 1:text_character=8'h49;
                        2:text_character=8'h4c; 3:text_character=8'h54;
                        4:text_character=8'h45; 5:text_character=8'h52;
                        6:text_character=8'h3a;
                        7:text_character=selected_gaussian?"G":"M"; 8:text_character=selected_gaussian?"A":"E";
                        9:text_character=selected_gaussian?"U":"D"; 10:text_character=selected_gaussian?"S":"I";
                        11:text_character=selected_gaussian?"S":"A"; 12:text_character=selected_gaussian?"I":"N";
                        13:text_character=selected_gaussian?"A":" "; 14:text_character=selected_gaussian?"N":" ";
                        default:text_character=8'h20;
                    endcase
                end
                default:text_character=8'h20;
            endcase
        end
    endfunction

    function [7:0] fps_character;
        input [3:0] column_number;
        input [15:0] value_bcd;
        begin
            case(column_number)
                0:fps_character=8'h46; 1:fps_character=8'h50;
                2:fps_character=8'h53; 3:fps_character=8'h3a;
                6:fps_character=8'h30+value_bcd[11:8];
                7:fps_character=8'h30+value_bcd[7:4];
                8:fps_character=8'h2e;
                9:fps_character=8'h30+value_bcd[3:0];
                default:fps_character=8'h20;
            endcase
        end
    endfunction
    function [7:0] profile_character;
        input [2:0] line_number;
        input [3:0] column_number;
        input [11:0] time_bcd,wait_bcd;
        input valid_report;
        input [1:0] table_kind;
        reg [127:0] label_text;
        begin
            case(line_number)
                0:label_text=PROFILE_INTERVAL?"PIPE  MS  DDR   ":"TIME  MS  DDR   ";
                1:label_text="WAIT            ";
                2:label_text="CAP             ";
                3:label_text="SCAN            ";
                4:label_text="ROOT            ";
                5:label_text="OUT             ";
                6:label_text=PROFILE_INTERVAL?"GAP             ":"TOTAL           ";
                default:label_text="                ";
            endcase
            if(table_kind==1) case(line_number)
                0:label_text="DIAG  A   B     ";
                1:label_text="CAM             ";
                2:label_text="CAP             ";
                3:label_text="GAP             ";
                4:label_text="IDLE            ";
                5:label_text="ERR             ";
                6:label_text="SLOW            ";
                default:label_text="                ";
            endcase
            if(table_kind==2) case(line_number)
                0:label_text="OUT   MS  DDR   ";
                1:label_text="READ            ";
                2:label_text="MIX             ";
                3:label_text="WRITE           ";
                4:label_text="SHAPE           ";
                5:label_text="REST            ";
                6:label_text="TOTAL           ";
                default:label_text="                ";
            endcase
            profile_character=label_text[(15-column_number)*8 +: 8];
            if(line_number!=0) begin
                case(column_number)
                    6:profile_character=valid_report?8'h30+time_bcd[11:8]:8'h2d;
                    7:profile_character=valid_report?8'h30+time_bcd[7:4]:8'h2d;
                    8:profile_character=valid_report?8'h30+time_bcd[3:0]:8'h2d;
                    10:profile_character=valid_report?8'h30+wait_bcd[11:8]:8'h2d;
                    11:profile_character=valid_report?8'h30+wait_bcd[7:4]:8'h2d;
                    12:profile_character=valid_report?8'h30+wait_bcd[3:0]:8'h2d;
                endcase
            end
        end
    endfunction
    // D/P = calibration done/pass; camera P/V/D = clock/frame/data seen.
    // CAP/PROC/OUT are sticky milestones, not timing or throughput reports.
    function [7:0] startup_character;
        input [2:0] line;
        input [3:0] column;
        input [14:0] health;
        reg [127:0] label;
        begin
            case(line)
                0:label="STARTUP STATUS  ";
                1:label="I2C D0 E0       ";
                2:label="DDR D0 P0       ";
                3:label="CAM P0 V0 D0    ";
                4:label="CAP S0 D0 E0    ";
                5:label="PROC S0 E0      ";
                6:label="OUT S0 R0 F0    ";
                default:label="                ";
            endcase
            startup_character=label[127-column*8 -: 8];
            case(line)
                1:case(column) 5:startup_character=8'h30+health[2];8:startup_character=8'h30+health[3];endcase
                2:case(column) 5:startup_character=8'h30+health[1];8:startup_character=8'h30+health[0];endcase
                3:case(column) 5:startup_character=8'h30+health[4];8:startup_character=8'h30+health[5];11:startup_character=8'h30+health[6];endcase
                4:case(column) 5:startup_character=8'h30+health[7];8:startup_character=8'h30+health[8];11:startup_character=8'h30+health[9];endcase
                5:case(column) 6:startup_character=8'h30+health[10];9:startup_character=8'h30+health[11];endcase
                6:case(column) 5:startup_character=8'h30+health[12];8:startup_character=8'h30+health[13];11:startup_character=8'h30+health[14];endcase
            endcase
        end
    endfunction
    function [63:0] font_glyph;
        input [7:0] character;
        begin
            case (character)
                8'h2d:font_glyph=64'h0000007e00000000;
                8'h2e:font_glyph=64'h0000000000181800;
                8'h30:font_glyph=64'h3c42464a52623c00;
                8'h31:font_glyph=64'h1838181818183c00;
                8'h32:font_glyph=64'h3c42020c30407e00;
                8'h33:font_glyph=64'h3c42021c02423c00;
                8'h34:font_glyph=64'h0c1424447e040400;
                8'h35:font_glyph=64'h7e407c0202423c00;
                8'h36:font_glyph=64'h1c20407c42423c00;
                8'h37:font_glyph=64'h7e02040810101000;
                8'h38:font_glyph=64'h3c42423c42423c00;
                8'h39:font_glyph=64'h3c42423e02043800;
                8'h2b:font_glyph=64'h0010107c10100000;
                8'h3e:font_glyph=64'h4020100810204000;
                8'h3a:font_glyph=64'h0018180018180000;
                8'h2f:font_glyph=64'h0204081020400000;
                8'h41:font_glyph=64'h1824427e42424200;
                8'h42:font_glyph=64'h7c42427c42427c00;
                8'h43:font_glyph=64'h3c42404040423c00;
                8'h44:font_glyph=64'h7844424242447800;
                8'h45:font_glyph=64'h7e40407c40407e00;
                8'h46:font_glyph=64'h7e40407c40404000;
                8'h47:font_glyph=64'h3c42404e42423c00;
                8'h48:font_glyph=64'h4242427e42424200;
                8'h58:font_glyph=64'h4242241824424200;
                8'h4b:font_glyph=64'h4244487048444200;
                8'h49:font_glyph=64'h3c18181818183c00;
                8'h4c:font_glyph=64'h4040404040407e00;
                8'h4d:font_glyph=64'h42665a5a42424200;
                8'h4e:font_glyph=64'h4262524a46424200;
                8'h4f:font_glyph=64'h3c42424242423c00;
                8'h50:font_glyph=64'h7c42427c40404000;
                8'h52:font_glyph=64'h7c42427c48444200;
                8'h53:font_glyph=64'h3c42403c02423c00;
                8'h54:font_glyph=64'h7e18181818181800;
                8'h55:font_glyph=64'h4242424242423c00;
                8'h4a:font_glyph=64'h1e04040444443800;
                8'h56:font_glyph=64'h4242424242241800;
                8'h57:font_glyph=64'h4242425a5a664200;
                8'h59:font_glyph=64'h4242241818181800;
                8'h5a:font_glyph=64'h7e02040810207e00;
                8'h2a:font_glyph=64'h0010543854100000;
                8'h3c:font_glyph=64'h0204081008040200;
                8'h5b:font_glyph=64'h3c20202020203c00;
                8'h5d:font_glyph=64'h3c04040404043c00;
                8'h7c:font_glyph=64'h1818181818181800;
                default:font_glyph=64'd0;
            endcase
        end
    endfunction

    function [7:0] select_glyph_row;
        input [63:0] glyph;
        input [2:0] row_number;
        begin
            case (row_number)
                0:select_glyph_row=glyph[63:56];
                1:select_glyph_row=glyph[55:48];
                2:select_glyph_row=glyph[47:40];
                3:select_glyph_row=glyph[39:32];
                4:select_glyph_row=glyph[31:24];
                5:select_glyph_row=glyph[23:16];
                6:select_glyph_row=glyph[15:8];
                default:select_glyph_row=glyph[7:0];
            endcase
        end
    endfunction

    // HUD regions: A STATUS, C CONTEXT, D GUIDE, E FEEDBACK. B DEBUG
    // reuses the existing diagnostic functions and coherent frame registers.
    // All function dependencies are explicit; no hidden global sensitivity.
    localparam [9:0] HUD_STATIC_FPS=FPS_VALUE*10;
    localparam [31:0] HUD_RESOLUTION=BUILD_PROFILE_ID==1 ? "480P" : "720P";
    localparam integer CONTEXT_Y=OSD_Y+56;
    localparam integer GUIDE_Y=SCREEN_HEIGHT-80;
    localparam integer FEEDBACK_X=(SCREEN_WIDTH-256)/2;
    localparam integer FEEDBACK_Y=SCREEN_HEIGHT-112;
    wire [128:0] hud_state={frame_camera_fps,frame_requested_profile,frame_profile_switch_state,frame_gaussian,frame_shape_enable,frame_context_notice,frame_osd,frame_notice,frame_global,frame_all,frame_mask,
        frame_group,frame_item,frame_mode,frame_edge,frame_filter,frame_auto,
        frame_manual,frame_high,(LIVE_FPS?frame_fps:HUD_STATIC_FPS),frame_media,frame_feedback};
    function [255:0] hud_line;
        input [1:0] region;
        input [2:0] row_number;
        input [128:0] state;
        reg [9:0] camera_fps;
        reg [1:0] target_profile;
        reg [3:0] switch_state;
        reg gaussian_on;
        reg shape_on;
        reg context_notice;
        reg status_on;
        reg guide_on;
        reg global_on;
        reg all_on;
        reg [2:0] mask;
        reg [2:0] group;
        reg [1:0] item;
        reg [2:0] mode;
        reg edge_on;
        reg filter_on;
        reg auto_on;
        reg [11:0] manual;
        reg [11:0] high;
        reg [9:0] fps;
        reg [54:0] media;
        reg [2:0] feedback;
        reg [255:0] line_text;
        reg [15:0] hb,mb,fb,ib,nb,cb,sb;
        reg [255:0] snap_text;
        begin
            {camera_fps,target_profile,switch_state,gaussian_on,shape_on,context_notice,status_on,guide_on,global_on,all_on,mask,group,item,mode,edge_on,filter_on,auto_on,manual,high,fps,media,feedback}=state;
            cb=binary_to_bcd({2'd0,camera_fps});
            hb=binary_to_bcd(high);mb=binary_to_bcd(manual);
            fb=binary_to_bcd({2'd0,fps});ib=binary_to_bcd({7'd0,media[52],media[20:17]});nb=binary_to_bcd({7'd0,media[53],media[24:21]});
            sb=binary_to_bcd({7'd0,media[54],media[3:0]});
            if(PHOTO_CAPACITY>=10) begin
                snap_text="SNAP 00/16                      ";
                snap_text[255-5*8 -: 8]=8'h30+sb[7:4];
                snap_text[255-6*8 -: 8]=8'h30+sb[3:0];
                snap_text[255-8*8 -: 8]=8'h30+PHOTO_CAPACITY/10;
                snap_text[255-9*8 -: 8]=8'h30+PHOTO_CAPACITY%10;
            end else begin
                snap_text="SNAP 0/8                        ";
                snap_text[255-5*8 -: 8]=8'h30+sb[3:0];
                snap_text[255-7*8 -: 8]=8'h30+PHOTO_CAPACITY;
            end
            line_text="                                ";
            case(region)
                0: case(row_number)
                    0: begin
                        line_text=edge_on ? "CANNY | MEDIAN | AUTO           " : "SOBEL | MEDIAN | AUTO           ";
                        if(!filter_on) line_text[255-8*8 -: 6*8]="NONE  ";
                        if(!auto_on) line_text[255-17*8 -: 4*8]="MAN ";
                        line_text[255-22*8 -: 5*8]={"|",HUD_RESOLUTION};
                        if(filter_on && gaussian_on) begin
                            line_text=edge_on ? "CANNY | GAUSSIAN | AUTO         " : "SOBEL | GAUSSIAN | AUTO         ";
                            if(!auto_on) line_text[255-19*8 -: 4*8]="MAN ";
                            line_text[255-24*8 -: 5*8]={"|",HUD_RESOLUTION};
                        end
                    end
                    1: begin
                        line_text="H:              FPS 00.0        ";
                        line_text[255-2*8 -: 8]=high>=1000?8'h30+hb[15:12]:8'h20;
                        line_text[255-3*8 -: 8]=high>=100?8'h30+hb[11:8]:8'h20;
                        line_text[255-4*8 -: 8]=high>=10?8'h30+hb[7:4]:8'h20;
                        line_text[255-5*8 -: 8]=8'h30+hb[3:0];
                        line_text[255-20*8 -: 8]=8'h30+fb[11:8];
                        line_text[255-21*8 -: 8]=8'h30+fb[7:4];
                        line_text[255-23*8 -: 8]=8'h30+fb[3:0];
                    end
                endcase
                1: case(group)
                    0: case(row_number)
                        0: if(context_notice) line_text="CAPTURE                         ";
                        1: line_text=snap_text;
                        2: if(media[4]) begin
                            line_text="  REC 0.0S                      ";
                        line_text[255-6*8 -: 8]=8'h30+media[12:9];
                        line_text[255-8*8 -: 8]=8'h30+media[8:5];
                        end
                    endcase
                    1: case(row_number)
                        0: if(context_notice) line_text="PLAYBACK                        ";
                        1: begin
                            line_text="NO MEDIA                        ";
                            if(media[51]) begin
                                line_text=media[25]?"00/00 VIDEO                     " : "00/00 IMG                       ";
                        line_text[255-0*8 -: 8]=8'h30+ib[7:4];
                        line_text[255-1*8 -: 8]=8'h30+ib[3:0];
                        line_text[255-3*8 -: 8]=8'h30+nb[7:4];
                        line_text[255-4*8 -: 8]=8'h30+nb[3:0];
                            end
                        end
                        2: if(media[51] && media[25]) begin
                            // Video length is at most 256 frames (~8.5s at
                            // 30fps). Render N.N directly; no overflow logic.
                            line_text=media[26]?"  0.0 / 0.0 S                   " : "II PAUSE 0.0 / 0.0 S            ";
                            if(media[26]) begin
                        line_text[255-2*8 -: 8]=8'h30+media[34:31];
                        line_text[255-4*8 -: 8]=8'h30+media[30:27];
                        line_text[255-8*8 -: 8]=8'h30+media[46:43];
                        line_text[255-10*8 -: 8]=8'h30+media[42:39];
                            end else begin
                        line_text[255-9*8 -: 8]=8'h30+media[34:31];
                        line_text[255-11*8 -: 8]=8'h30+media[30:27];
                        line_text[255-15*8 -: 8]=8'h30+media[46:43];
                        line_text[255-17*8 -: 8]=8'h30+media[42:39];
                            end
                        end
                    endcase
                    2: case(row_number)
                        0: begin line_text="DISPLAY 1/7                     ";line_text[255-8*8 -: 8]=8'h31+mode;end
                        1: case(mode)
                            0: line_text="COLOR                           ";
                            1: line_text="FILTERED GRAY                   ";
                            2: line_text="EDGE                            ";
                            3: line_text="COLOR / EDGE                    ";
                            4: line_text="OVERLAY                         ";
                            5: line_text="RAW GRAY / EDGE                 ";
                            6: line_text="GRADIENT OVERLAY                ";
                        endcase
                    endcase
                    3: case(row_number)
                        0: line_text="THRESHOLD                       ";
                        1: begin line_text="HIGH  096                       ";
                        line_text[255-5*8 -: 8]=manual>=1000?8'h30+mb[15:12]:8'h20;
                        line_text[255-6*8 -: 8]=8'h30+mb[11:8];
                        line_text[255-7*8 -: 8]=8'h30+mb[7:4];
                        line_text[255-8*8 -: 8]=8'h30+mb[3:0];
                        end
                        2: line_text=auto_on?"AUTO ON                         " : "AUTO OFF                        ";
                    endcase
                    4: begin
                        case(row_number)
                            0: line_text="ALGORITHM                       ";
                            1: line_text=edge_on?"  EDGE      CANNY               " : "  EDGE      SOBEL               ";
                            2: line_text=!filter_on ? "  FILTER    NONE                " :
                                gaussian_on ? "  FILTER    GAUSSIAN            " : "  FILTER    MEDIAN              ";
                            3: line_text=BUILD_PROFILE_ID==1?"  OUTPUT    480P60              ":"  OUTPUT    720P                ";
                            5: if(PROFILE_MENU_ENABLE && switch_state!=0) begin
                                case(switch_state)
                                    8: line_text="CAUSE CFG                       ";
                                    9: line_text="CAUSE DDR                       ";
                                    10: line_text="CAUSE SEQUENCE                  ";
                                    11: line_text="CAUSE TIMEOUT                   ";
                                    default: line_text=target_profile==1?"TO 480P60                       ":"TO 720P                         ";
                                endcase
                            end
                            6: if(PROFILE_MENU_ENABLE) case(switch_state)
                                1,2,3,4,5: line_text="SWITCH BUSY                     ";
                                6: line_text="FLASH REQUIRED                  ";
                                7,8,9,10,11: line_text="SWITCH FAILED                   ";
                            endcase
                            7: if(PROFILE_MENU_ENABLE && item==2) begin
                                line_text="CAM 00.0  PUB 00.0              ";
                                line_text[255-4*8 -: 8]=8'h30+cb[11:8];
                                line_text[255-5*8 -: 8]=8'h30+cb[7:4];
                                line_text[255-7*8 -: 8]=8'h30+cb[3:0];
                                line_text[255-14*8 -: 8]=8'h30+fb[11:8];
                                line_text[255-15*8 -: 8]=8'h30+fb[7:4];
                                line_text[255-17*8 -: 8]=8'h30+fb[3:0];
                            end
                            4: line_text=shape_on?"  SHAPE     ON                  ":"  SHAPE     OFF                 ";
                        endcase
                        if(row_number=={1'b0,item}+3'd1) line_text[255:248]=8'h3e;
                    end
                    5: begin
                        case(row_number)
                            0: line_text="OSD                             ";
                            1: line_text=mask[0]?"  STATUS   ON                   " : "  STATUS   OFF                  ";
                            2: line_text=mask[1]?"  GUIDE    ON                   " : "  GUIDE    OFF                  ";
                            3: line_text=mask[2]?"  DEBUG    ON                   " : "  DEBUG    OFF                  ";
                            4: line_text=all_on?"  ALL      ON                   " : "  ALL      OFF                  ";
                        endcase
                        if(row_number=={1'b0,item}+3'd1) line_text[255:248]=8'h3e;
                    end
                endcase
                2: case(row_number)
                    0: line_text="        [K1]        [K2]        ";
                    1: case(group)
                        0: line_text="TAP     IN          OUT         ";
                        1: line_text="TAP     PREV        NEXT        ";
                        2: line_text="TAP     <           >           ";
                        3: line_text="TAP     -16         +16         ";
                        4: line_text="TAP     < ITEM      ITEM >      ";
                        5: line_text="TAP     < ITEM      ITEM >      ";
                    endcase
                    2: line_text=group==0 ? "2X      OSD         >SNAP<      " : "2X      OSD         --          ";
                    3: case(group)
                        0: line_text="HOLD    LEVEL       >PRE<       ";
                        1: line_text=(media[51] && media[25])?"HOLD    LEVEL       >PLAY/PAUSE<":"HOLD    LEVEL       --          ";
                        2: line_text="HOLD    LEVEL       --          ";
                        3: line_text="HOLD    LEVEL       >AUTO<      ";
                        4: line_text=(item==0 || (RUNTIME_FILTER && item==1) || (PROFILE_MENU_ENABLE && item==2))?"HOLD    LEVEL       >CHANGE<    ":
                                     item==3?"HOLD    LEVEL       >ON/OFF<    ":"HOLD    LEVEL       --          ";
                        5: line_text="HOLD    LEVEL       >ON/OFF<    ";
                    endcase
                endcase
                3: begin
                    if(group==0) case(feedback)
                        1: line_text=snap_text;
                        2: line_text="PRE DONE                        ";
                        3: line_text="CLIP SAVED                      ";
                    endcase
                    if(group==1) case(feedback)
                        4: line_text="PLAY                            ";
                        5: line_text="PAUSE                           ";
                        6: line_text="VIDEO END                       ";
                    endcase
                end
            endcase
            // Media status stays in fixed rows after its title expires. Other
            // contexts, including their level titles, expire together at 1.5s.
            if(region==1 && group>=2 && !context_notice)
                line_text="                                ";
            hud_line=line_text;
        end
    endfunction
    function hud_dot;
        input [11:0] px,py;
        input [128:0] state;
        reg active;
        reg [1:0] region;
        reg [11:0] dx,dy;
        reg [255:0] line_text;
        reg [63:0] glyph_value;
        reg [7:0] pixels;
        begin
            active=0;region=0;dx=0;dy=0;
            // Feedback takes priority if parameterized regions ever overlap.
            // Fields above media move with its three appended count bits.
            if(state[107] && state[102:100]<2 && state[2:0]!=0 &&
               px>=FEEDBACK_X && px<FEEDBACK_X+256 && py>=FEEDBACK_Y && py<FEEDBACK_Y+16) begin
                active=1;region=3;dx=px-FEEDBACK_X;dy=py-FEEDBACK_Y;
            end else if(state[109] && px>=OSD_X && px<OSD_X+256 && py>=OSD_Y && py<OSD_Y+32) begin
                active=1;region=0;dx=px-OSD_X;dy=py-OSD_Y;
            end else if(state[109] && px>=OSD_X && px<OSD_X+256 && py>=CONTEXT_Y &&
                        py<CONTEXT_Y+((PROFILE_MENU_ENABLE && state[102:100]==4)?128:80)) begin
                active=1;region=1;dx=px-OSD_X;dy=py-CONTEXT_Y;
            end else if(state[108] && px>=OSD_X && px<OSD_X+256 && py>=GUIDE_Y && py<GUIDE_Y+64) begin
                active=1;region=2;dx=px-OSD_X;dy=py-GUIDE_Y;
            end
            line_text=hud_line(region,dy[6:4],state);
            glyph_value=font_glyph(line_text[255-dx[7:3]*8 -: 8]);
            pixels=select_glyph_row(glyph_value,dy[3:1]);
            hud_dot=active && pixels[7-dx[2:0]];
        end
    endfunction
    wire hud_glyph_active=MENU_ENABLE && input_de && hud_dot(input_x,input_y,hud_state);
    wire hud_shadow_active=MENU_ENABLE && input_de && input_x!=0 && input_y!=0 &&
        hud_dot(input_x-12'd1,input_y-12'd1,hud_state);
    wire hud_rec=MENU_ENABLE && frame_osd && frame_group==0 && frame_media[4] && input_de &&
        input_x>=OSD_X && input_x<OSD_X+6 && input_y>=CONTEXT_Y+36 && input_y<CONTEXT_Y+42;
    wire hud_play=MENU_ENABLE && frame_osd && frame_group==1 && frame_media[51] && frame_media[25] && frame_media[26] && input_de &&
        input_x>=OSD_X && input_x<OSD_X+6 && input_y>=CONTEXT_Y+36 && input_y<CONTEXT_Y+42;
    wire hud_flash=MENU_ENABLE && frame_global && frame_group==0 && (frame_border==1 || frame_border==2) && input_de &&
        input_x<SCREEN_WIDTH && input_y<SCREEN_HEIGHT &&
        (input_x<2 || input_x>=SCREEN_WIDTH-2 || input_y<2 || input_y>=SCREEN_HEIGHT-2);
    // Four unoriented gradient axes (x right, y down), sharing one center.
    // Beside the existing GUIDE text; visibility never depends on UI level/notice.
    localparam integer DIRECTION_X=OSD_X+288,DIRECTION_Y=GUIDE_Y+32;
    wire [11:0] direction_dx=input_x>=DIRECTION_X?input_x-DIRECTION_X:DIRECTION_X-input_x;
    wire [11:0] direction_dy=input_y>=DIRECTION_Y?input_y-DIRECTION_Y:DIRECTION_Y-input_y;
    wire direction_horizontal=direction_dy==0 && direction_dx<=24;
    wire direction_vertical=direction_dx==0 && direction_dy<=24;
    wire direction_diagonal=direction_dx==direction_dy && direction_dx<=18;
    wire direction_guide=MENU_ENABLE && frame_global && frame_mask[1] && frame_mode==6 && input_de &&
        (direction_horizontal || direction_vertical || direction_diagonal);
    wire [23:0] direction_guide_color=direction_horizontal?24'hff0000:
        direction_vertical?24'h0000ff:
        ((input_x<DIRECTION_X)==(input_y<DIRECTION_Y))?24'h00ff00:24'hffff00;
    wire [23:0] debug_background=MENU_ENABLE ? {1'b0,input_rgb888[23:17],1'b0,input_rgb888[15:9],1'b0,input_rgb888[7:1]} : BACKGROUND_COLOR;

    // Legacy fixed-panel shape text remains available for compatibility.
    localparam SHAPE_Y=NOTICE_Y+NOTICE_HEIGHT+16;
    wire shape_panel_active=!MENU_ENABLE && SHAPE_ENABLE && frame_osd && input_de &&
        input_x>=OSD_X-4 && input_x<OSD_X+132 && input_y>=SHAPE_Y-4 && input_y<SHAPE_Y+16*SHAPE_TARGETS+4;
    wire shape_text_active=!MENU_ENABLE && SHAPE_ENABLE && frame_osd && input_de &&
        input_x>=OSD_X && input_x<OSD_X+128 && input_y>=SHAPE_Y && input_y<SHAPE_Y+16*SHAPE_TARGETS;
    wire [11:0] shape_y=input_y-SHAPE_Y;
    wire [1:0] line_shape=shape_y[4]?frame_shape_second:frame_shape;
    wire [127:0] single_shape_text=frame_shape==1?"SHAPE: RECT     ":
                                  frame_shape==2?"SHAPE: CIRCLE   ":"SHAPE: NONE     ";
    wire [127:0] multi_shape_text={"SHAPE",(shape_y[4]?8'h32:8'h31),": ",
        (line_shape==1?64'h5245435420202020:line_shape==2?64'h434952434c452020:64'h4e4f4e4520202020)};
    wire [127:0] shape_text=SHAPE_TARGETS==2?multi_shape_text:single_shape_text;
    wire [7:0] shape_character=shape_text[(15-character_column)*8 +: 8];
    wire [63:0] shape_glyph=font_glyph(shape_character);
    wire [7:0] shape_pixels=select_glyph_row(shape_glyph,shape_y[3:1]);
    wire shape_glyph_active=shape_text_active && shape_pixels[7-glyph_column];

    wire [15:0] fps_bcd=binary_to_bcd({2'b0,frame_fps});
    wire [159:0] auto_text=frame_auto?"AUTO THRESHOLD: ON  ":"AUTO THRESHOLD: OFF ";
    wire [127:0] media_label="CAPTURE/PLAYBACK";
    wire [7:0] character = (text_line==5)?auto_text[159-character_column*8 -: 8]:
                          (character_column>=16)?8'h20:
                          (LIVE_FPS && text_line==3)?fps_character(character_column[3:0],fps_bcd):
                          (text_line==6)?media_label[127-character_column[3:0]*8 -: 8]:
                          text_character(text_line, character_column[3:0], frame_edge, frame_gaussian,
                                         low_threshold_bcd, high_threshold_bcd);
    wire [63:0] glyph = font_glyph(character);
    wire [7:0] glyph_pixels = select_glyph_row(glyph, glyph_row);
    wire glyph_active = text_active && glyph_pixels[7-glyph_column];
    wire [7:0] notice_character_value=notice_character(frame_group,notice_column);
    wire [63:0] notice_glyph = font_glyph(notice_character_value);
    wire [7:0] notice_glyph_pixels = select_glyph_row(notice_glyph,notice_glyph_row);
    wire notice_glyph_active = notice_text_active &&
                               notice_glyph_pixels[7-notice_glyph_column];
    wire [63:0] profile_glyph=font_glyph(profile_character(profile_line,profile_column,
        profile_time,profile_wait,frame_profile_valid,2'd0));
    wire [7:0] profile_pixels=select_glyph_row(profile_glyph,glyph_row);
    wire profile_glyph_active=profile_text_active && profile_pixels[7-profile_x[2:0]];
    wire [63:0] schedule_glyph=font_glyph(
        STARTUP_DIAGNOSTIC_ENABLE && !frame_schedule_valid?
        startup_character(profile_line,schedule_x[6:3],frame_startup):
        profile_character(profile_line,schedule_x[6:3],
        schedule_a,schedule_b,frame_schedule_valid,2'd1));
    wire [7:0] schedule_pixels=select_glyph_row(schedule_glyph,glyph_row);
    wire schedule_glyph_active=schedule_text_active && schedule_pixels[7-schedule_x[2:0]];
    wire [63:0] out_profile_glyph=font_glyph(profile_character(out_profile_line,out_profile_x[6:3],
        out_profile_time,out_profile_wait,frame_out_profile_valid,2'd2));
    wire [7:0] out_profile_pixels=select_glyph_row(out_profile_glyph,out_profile_y[3:1]);
    wire out_profile_glyph_active=out_profile_text_active && out_profile_pixels[7-out_profile_x[2:0]];
    wire [63:0] edge_stats_glyph=font_glyph(edge_stats_character(edge_stats_line,edge_stats_x[6:3],
        edge_stats_value,frame_edge_stats_valid));
    wire [7:0] edge_stats_pixels=select_glyph_row(edge_stats_glyph,edge_stats_y[3:1]);
    wire edge_stats_glyph_active=edge_stats_text_active && edge_stats_pixels[7-edge_stats_x[2:0]];

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            output_hsync <= 1'b0;
            output_vsync <= 1'b0;
            output_de <= 1'b0;
            output_rgb888 <= 24'd0;
        end else begin
            output_hsync <= input_hsync;
            output_vsync <= input_vsync;
            output_de <= input_de;
            if (!input_de)
                output_rgb888 <= 24'd0;
            else if(hud_flash)
                output_rgb888<=frame_border==1?24'hffffff:24'h00ff00;
            else if(direction_guide)
                output_rgb888<=direction_guide_color;
            else if(hud_glyph_active)
                output_rgb888<=FOREGROUND_COLOR;
            else if(hud_rec)
                output_rgb888<=24'hff3030;
            else if(hud_play)
                output_rgb888<=24'h00ff60;
            else if(hud_shadow_active)
                output_rgb888<=24'h000000;
            else if (notice_glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (notice_panel_active)
                output_rgb888 <= BACKGROUND_COLOR;
            else if (profile_glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (profile_panel_active)
                output_rgb888 <= debug_background;
            else if (schedule_glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (schedule_panel_active)
                output_rgb888 <= debug_background;
            else if (out_profile_glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (out_profile_panel_active)
                output_rgb888 <= debug_background;
            else if (edge_stats_glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (edge_stats_panel_active)
                output_rgb888 <= debug_background;
            else if (shape_glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (shape_panel_active)
                output_rgb888 <= BACKGROUND_COLOR;
            else if (glyph_active)
                output_rgb888 <= FOREGROUND_COLOR;
            else if (panel_active)
                output_rgb888 <= BACKGROUND_COLOR;
            else
                output_rgb888 <= input_rgb888;
        end
    end
endmodule
