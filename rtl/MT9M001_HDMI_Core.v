`timescale 1ns/1ps

// Functional core: MT9M001 DVP -> lightweight ISP -> source/process hooks
// -> DDR frame buffer -> 720p timing -> HDMI TMDS symbols.
module MT9M001_HDMI_Core #(
    parameter [1:0] BUILD_PROFILE_ID = 2'd0,
    parameter integer BUILD_PROFILE_FPS = 30,
    parameter integer PROFILE_RECONFIG_ENABLE = 0,
    parameter integer PIXEL_CLOCK_HZ = 74400000,
    parameter integer SENSOR_ROW_START = 164,
    parameter integer SENSOR_COLUMN_START = 20,
    parameter integer SENSOR_HBLANK = 19,
    parameter integer SENSOR_VBLANK = 25,
    parameter integer SENSOR_WIDTH = 1280,
    parameter integer SENSOR_HEIGHT = 720,
    parameter integer DISPLAY_WIDTH = 1280,
    parameter integer DISPLAY_HEIGHT = 720,
    parameter integer PIXEL_BITS = 16,
    parameter integer SYSTEM_CLOCK_HZ = 96000000,
    parameter integer MEDIA_PHOTO_FRAMES = 8,
    parameter integer MEDIA_VIDEO_FRAMES = 112,
    parameter integer MEDIA_PRE_FRAMES = 128,
    parameter SENSOR_ROW_MIRROR = 1,
    parameter SENSOR_COLUMN_MIRROR = 1,
    parameter [1:0] BAYER_PATTERN = 2'd1,
    parameter integer GAMMA_ENABLE = 1,
    // Calibrated from MT9M001 pre-WB black/gray/white ROI statistics.
    parameter integer R_GAIN_Q8 = 267,
    parameter integer G_GAIN_Q8 = 256,
    parameter integer B_GAIN_Q8 = 452,
    parameter integer BLACK_LEVEL = 0,
    parameter [2:0] IMAGE_PROCESS_MODE = 3,
    parameter integer OSD_ENABLE = 1,
    // 1: complete-frame Canny/Sobel with overlapping workers; 0: streaming path.
    parameter integer CANNY_ENABLE = 1,
    parameter integer CANNY_PROFILE_ENABLE = 1,
    parameter integer CANNY_OUT_PROFILE_ENABLE = 1,
    parameter integer CANNY_DIAGNOSTIC_ENABLE = 1,
    parameter integer CANNY_FRAME_PIPELINE = 1,
    // Conservative isolated-component experiment. A component containing an
    // untracked provisional seed is preserved. Set the minimum to 0 to bypass.
    parameter integer EDGE_MIN_COMPONENT_PIXELS = 8,
    parameter integer EDGE_AREA_TRACK_LABELS = 16384,
    parameter integer EDGE_DIAGNOSTIC_ENABLE = 1,
    parameter [11:0] SOBEL_THRESHOLD = 12'd96,
    parameter [11:0] SOBEL_THRESHOLD_MIN = 12'd16,
    parameter [11:0] SOBEL_THRESHOLD_MAX = 12'd1024,
    parameter [11:0] SOBEL_THRESHOLD_STEP = 12'd16,
    parameter EDGE_PROCESS_SELECT = 1'b1,
    parameter integer SHAPE_ENABLE = 1,
    parameter integer SHAPE_TARGETS = 6,
    parameter integer DISPLAY_FPS = 30,
    parameter [15:0] EXPOSURE_LINES = SENSOR_HEIGHT,
    parameter [15:0] ANALOG_GAIN = 16'h0008,
    parameter integer H_FRONT = 110,
    parameter integer H_SYNC = 40,
    parameter integer H_BACK = 220,
    parameter integer V_FRONT = 5,
    parameter integer V_SYNC = 5,
    parameter integer V_BACK = 20,
    // Image quality first by default. FPS remains measured/displayed but cannot
    // force edge thresholds upward. Guarded build is retained for comparison.
    parameter integer AUTO_RATE_GUARD_ENABLE = 0
)(
    input  wire        clk_sys,
    input  wire        clk_pixel,
    input  wire        k1_n,
    input  wire        k2_n,
    input  wire        status_uart_rx,
    input  wire        sys_pll_lock,
    input  wire        ddr_pll_lock,
    input  wire        core_clk,
    input  wire        tac_clk,
    input  wire        twd_clk,
    input  wire        tdqss_clk,

    input  wire        cfg_error,
    output wire [1:0]  cfg_cbsel,
    output wire        cfg_ena,cfg_config,

    input  wire        cmos_pclk,
    input  wire        cmos_vsync,
    input  wire        cmos_href,
    input  wire [7:0]  cmos_data,
    input  wire        cmos_sdat_i,
    output wire        cmos_sclk,
    output wire        cmos_sdat_o,
    output wire        cmos_sdat_oe,

    output wire [15:0] ddr_addr,
    output wire [2:0]  ddr_ba,
    output wire        ddr_we,
    output wire        ddr_reset,
    output wire        ddr_ras,
    output wire        ddr_cas,
    output wire        ddr_odt,
    output wire        ddr_cke,
    output wire        ddr_cs,
    input  wire [15:0] ddr_dq_hi_i,
    input  wire [15:0] ddr_dq_lo_i,
    output wire [15:0] ddr_dq_hi_o,
    output wire [15:0] ddr_dq_lo_o,
    output wire [15:0] ddr_dq_oe,
    output wire [1:0]  ddr_dm_hi_o,
    output wire [1:0]  ddr_dm_lo_o,
    input  wire [1:0]  ddr_dqs_hi_i,
    input  wire [1:0]  ddr_dqs_lo_i,
    input  wire [1:0]  ddr_dqs_n_hi_i,
    input  wire [1:0]  ddr_dqs_n_lo_i,
    output wire [1:0]  ddr_dqs_hi_o,
    output wire [1:0]  ddr_dqs_lo_o,
    output wire [1:0]  ddr_dqs_n_hi_o,
    output wire [1:0]  ddr_dqs_n_lo_o,
    output wire [1:0]  ddr_dqs_oe,
    output wire [1:0]  ddr_dqs_n_oe,
    output wire [2:0]  ddr_shift,
    output wire [4:0]  ddr_shift_sel,
    output wire        ddr_shift_ena,

    output wire [9:0]  hdmi_clock,
    output wire [9:0]  hdmi_data0,
    output wire [9:0]  hdmi_data1,
    output wire [9:0]  hdmi_data2,
    output wire [5:0]  status_led,
    output wire status_uart_tx
);
    localparam integer FRAME_BYTES = SENSOR_WIDTH * SENSOR_HEIGHT * (PIXEL_BITS/8);
    // MT9M001 mirror readout preserves the starting Bayer color. Keep the
    // hardware-validated GBRG phase even when row/column order is reversed.
    // The mirror parameters affect register 0x20 only, never the ISP phase.
    localparam [1:0] EFFECTIVE_BAYER_PATTERN = BAYER_PATTERN;

    wire reset_lock = sys_pll_lock & ddr_pll_lock;
    reg reset_n_sys = 1'b0;
    reg reset_n_pixel = 1'b0;
    always @(posedge clk_sys or negedge reset_lock)
        if (!reset_lock) reset_n_sys <= 1'b0; else reset_n_sys <= 1'b1;
    always @(posedge clk_pixel or negedge reset_lock)
        if (!reset_lock) reset_n_pixel <= 1'b0; else reset_n_pixel <= 1'b1;

    // Runtime controls originate in clk_sys and cross together by handshake.
    wire [2:0] mode_request_sys;
    wire edge_request_sys;
    wire gaussian_request_sys,gaussian_active;
    wire [11:0] threshold_request_sys;
    // The dedicated THRESHOLD control group keeps the user-selected value;
    // other groups use automatic thresholds only when the master switch is on.
    wire auto_threshold_enable_sys;
    wire [2:0] control_group;
    wire remote_auto_override_sys;
    wire adaptive_request_sys = auto_threshold_enable_sys && (control_group != 3'd3 || remote_auto_override_sys);
    wire osd_enable_sys, control_notice_sys;
    wire [2:0] osd_mask_sys;
    wire [1:0] control_item_sys;
    wire shape_enable_sys;
    wire [3:0] reserved_action_sys;
    wire osd_all_enable_sys,osd_quick_hidden_sys;
    // Media command interface; Capture/Playback owns retained media.
    wire [8:0] ui_operation_request_sys;
    wire uart_control_write,key_event_sys;
    wire [3:0] uart_control_address;
    wire [15:0] uart_control_value;
    wire profile_request;
    wire profile_switch_busy;
    wire [1:0] requested_profile;
    wire [3:0] profile_switch_state;
    wire [1:0] profile_target;
    wire profile_store,profile_quiesce,profile_reconfigure;
    wire profile_stored,profile_quiesced,profile_backend_failed;
    wire [1:0] profile_failure_reason;
    wire profile_traffic_idle,profile_traffic_error;
    wire profile_pause=PROFILE_RECONFIG_ENABLE && profile_quiesce;
    Profile_Switch_Controller profile_controller (
        .clk(clk_sys),.rst_n(reset_n_sys),.current_profile(BUILD_PROFILE_ID),
        .profile_switch_request(profile_request),.requested_profile(requested_profile),
        .backend_supported(PROFILE_RECONFIG_ENABLE!=0),.backend_busy(1'b0),.backend_failed(profile_backend_failed),
        .backend_failure_reason(profile_failure_reason),
        .boot_selection_stored(profile_stored),.video_quiesced(profile_quiesced),
        .store_boot_selection(profile_store),.quiesce_video(profile_quiesce),.request_reconfiguration(profile_reconfigure),
        .target_profile(profile_target),.state(profile_switch_state),.busy(profile_switch_busy),.failed());
    generate if(PROFILE_RECONFIG_ENABLE) begin : profile_boot
        Profile_Reconfig_Backend #(.NOTICE_CYCLES(SYSTEM_CLOCK_HZ/4)) backend (
            .clk(clk_sys),.rst_n(reset_n_sys),.target_profile(profile_target),
            .store_boot_selection(profile_store),.quiesce_video(profile_quiesce),
            .request_reconfiguration(profile_reconfigure),.traffic_idle(profile_traffic_idle),
            .traffic_error(profile_traffic_error),.cfg_error(cfg_error),
            .boot_selection_stored(profile_stored),.video_quiesced(profile_quiesced),
            .backend_failed(profile_backend_failed),.failure_reason(profile_failure_reason),
            .cfg_cbsel(cfg_cbsel),.cfg_ena(cfg_ena),.cfg_config(cfg_config));
    end else begin : no_profile_boot
        assign profile_stored=0;assign profile_quiesced=0;assign profile_backend_failed=0;assign profile_failure_reason=0;
        assign cfg_cbsel=0;assign cfg_ena=0;assign cfg_config=0;
    end endgenerate
    wire [5:0] profile_status_pixel;
    Video_Status_Sync #(.WIDTH(6),.DEFAULT_STATUS({BUILD_PROFILE_ID,4'd0})) profile_status_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),.source_status({profile_target,profile_switch_state}),
        .pixel_clk(clk_pixel),.pixel_status(profile_status_pixel));
    debug #(
        .DEBOUNCE_COUNT(SYSTEM_CLOCK_HZ/50),
        .COUNTER_WIDTH(21),
        .DEFAULT_MODE(IMAGE_PROCESS_MODE),
        .DEFAULT_EDGE_SELECT(EDGE_PROCESS_SELECT),
        .K1_LONG_PRESS_COUNT(SYSTEM_CLOCK_HZ/2),
        .K1_LONG_COUNTER_WIDTH(27),
        .NOTICE_COUNT(SYSTEM_CLOCK_HZ/2*3),
        .NOTICE_COUNTER_WIDTH(28),
        .LONG_PRESS_COUNT(SYSTEM_CLOCK_HZ/5*3),
        .DOUBLE_CLICK_COUNT(SYSTEM_CLOCK_HZ/10*3),
        .DEFAULT_THRESHOLD(SOBEL_THRESHOLD),
        .THRESHOLD_MIN(SOBEL_THRESHOLD_MIN),
        .THRESHOLD_MAX(SOBEL_THRESHOLD_MAX),
        .THRESHOLD_STEP(SOBEL_THRESHOLD_STEP),.REMOTE_ENABLE(1)
    ) debug_control (
        .clk(clk_sys), .rst_n(reset_n_sys), .k1_n(k1_n), .k2_n(k2_n),
        // RAW lease operations are transport-only: never wake the HDMI guide.
        .remote_write(uart_control_write && (uart_control_address<12 || uart_control_address==14)),.remote_address(uart_control_address),.remote_value(uart_control_value),
        .key_event(key_event_sys),
        .mode_select(mode_request_sys), .edge_select(edge_request_sys),
        .threshold_value(threshold_request_sys),
        .osd_enable(osd_enable_sys),
        .auto_threshold_enable(auto_threshold_enable_sys),
        .remote_auto_override(remote_auto_override_sys),
        .control_notice(control_notice_sys), .control_group(control_group),
        .osd_mask(osd_mask_sys),.control_item(control_item_sys),.reserved_action(reserved_action_sys),
        .shape_enable(shape_enable_sys),.gaussian_select(gaussian_request_sys),
        .osd_all_enable(osd_all_enable_sys),.osd_quick_hidden(osd_quick_hidden_sys),
        .operation_request(ui_operation_request_sys),
        .current_profile(BUILD_PROFILE_ID),.profile_switch_request(profile_request),
        .requested_profile(requested_profile)
    );

    wire [2:0] mode_active;
    wire edge_active;
    wire [11:0] threshold_active;
    wire adaptive_active;

    // Sensor register programming.
    wire [7:0] config_index;
    wire [23:0] config_data;
    wire [7:0] config_size;
    wire config_done;
    wire config_error;
    MT9M001_I2C_Master #(
        .CLK_FREQ(SYSTEM_CLOCK_HZ), .I2C_FREQ(50000)
    ) sensor_i2c (
        .clk(clk_sys), .rst_n(reset_n_sys),
        .i2c_sclk(cmos_sclk), .i2c_sdat_i(cmos_sdat_i),
        .i2c_sdat_o(cmos_sdat_o), .i2c_sdat_oe(cmos_sdat_oe),
        .config_index(config_index), .config_data(config_data),
        .config_size(config_size), .config_done(config_done),
        .config_error(config_error)
    );
    MT9M001_Register_Table #(
        .ACTIVE_WIDTH(SENSOR_WIDTH), .ACTIVE_HEIGHT(SENSOR_HEIGHT),
        .ROW_START(SENSOR_ROW_START),.COLUMN_START(SENSOR_COLUMN_START),
        .HORIZONTAL_BLANKING(SENSOR_HBLANK),.VERTICAL_BLANKING(SENSOR_VBLANK),
        .ROW_MIRROR(SENSOR_ROW_MIRROR), .COLUMN_MIRROR(SENSOR_COLUMN_MIRROR),
        .EXPOSURE_LINES(EXPOSURE_LINES), .ANALOG_GAIN(ANALOG_GAIN)
    ) sensor_registers (
        .LUT_INDEX(config_index), .LUT_DATA(config_data), .LUT_SIZE(config_size)
    );

    reg [2:0] config_done_sync = 3'b000;
    always @(posedge cmos_pclk or negedge reset_n_sys)
        if (!reset_n_sys) config_done_sync <= 3'b000;
        else config_done_sync <= {config_done_sync[1:0], config_done};

    // MT9M001 lightweight ISP.
    wire sensor_vsync;
    wire sensor_de;
    wire [15:0] sensor_rgb565;
    wire raw_tap_vsync,raw_tap_de;
    wire [7:0] raw_tap_bayer;
    MT9M001_ISP #(
        .IMAGE_WIDTH(SENSOR_WIDTH), .IMAGE_HEIGHT(SENSOR_HEIGHT),
        .BAYER_PATTERN(EFFECTIVE_BAYER_PATTERN), .GAMMA_ENABLE(GAMMA_ENABLE),
        .R_GAIN_Q8(R_GAIN_Q8), .G_GAIN_Q8(G_GAIN_Q8), .B_GAIN_Q8(B_GAIN_Q8),
        .BLACK_LEVEL(BLACK_LEVEL)
    ) sensor_isp (
        .clk(cmos_pclk), .rst_n(reset_n_sys & config_done_sync[2]),
        .in_vsync(cmos_vsync), .in_href(cmos_href), .in_bayer(cmos_data),
        .out_vsync(sensor_vsync), .out_href(sensor_de), .out_rgb565(sensor_rgb565),
        .raw_vsync(raw_tap_vsync),.raw_href(raw_tap_de),.raw_bayer(raw_tap_bayer)
    );

    // Reserved second source and frame-safe selection point.
    wire selected_vsync;
    wire selected_de;
    wire [15:0] selected_rgb565;
    Video_Source_Selector #(.DATA_WIDTH(16)) source_selector (
        .clk(cmos_pclk), .rst_n(reset_n_sys), .select_request(1'b0),
        .source0_vsync(sensor_vsync), .source0_de(sensor_de), .source0_data(sensor_rgb565),
        .source1_vsync(1'b0), .source1_de(1'b0), .source1_data(16'd0),
        .output_vsync(selected_vsync), .output_de(selected_de), .output_data(selected_rgb565)
    );

    wire processed_vsync;
    wire processed_de;
    wire [15:0] processed_rgb565;
    Video_Control_Sync #(
        .DEFAULT_MODE(IMAGE_PROCESS_MODE), .DEFAULT_EDGE_SELECT(EDGE_PROCESS_SELECT),
        .DEFAULT_THRESHOLD(SOBEL_THRESHOLD),.RUNTIME_FILTER(1)
    ) video_controls (
        .rst_n(reset_n_sys), .source_clk(clk_sys),
        .mode_request(mode_request_sys), .edge_request(edge_request_sys),
        .threshold_request(threshold_request_sys),
        .adaptive_request(adaptive_request_sys),
        .gaussian_request(gaussian_request_sys),.gaussian_active(gaussian_active),
        .pixel_clk(cmos_pclk), .frame_idle(!selected_vsync && !processed_vsync),
        .mode_active(mode_active), .edge_active(edge_active),
        .threshold_active(threshold_active), .adaptive_active(adaptive_active)
    );
    wire [1:0] canny_class,canny_direction;
    wire [7:0] canny_filtered;
    wire [11:0] canny_threshold_low;
    wire [11:0] canny_threshold_high;
    wire adaptive_decrease_sys;
    wire adaptive_decrease_camera;
    wire [11:0] adaptive_rate_floor_sys;
    wire [11:0] adaptive_rate_floor_camera;
    wire [19:0] canny_scene_samples;
    wire [11:0] canny_scene_high;
    wire [43:0] canny_feedback_sys;
    generate if(CANNY_ENABLE) begin: g_canny_frontend
        Canny_Frontend #(.ACTIVE_WIDTH(SENSOR_WIDTH),.ACTIVE_HEIGHT(SENSOR_HEIGHT),.RUNTIME_ALGORITHM(1),
            .AUTO_RATE_GUARD_ENABLE(AUTO_RATE_GUARD_ENABLE),.RUNTIME_FILTER(1)) frontend (
            .gaussian_select(gaussian_active),
            .clk(cmos_pclk),.rst_n(reset_n_sys),
            .input_vsync(selected_vsync),.input_de(selected_de),.input_rgb565(selected_rgb565),
            .threshold_low({1'b0,threshold_active[11:1]}),.threshold_high(threshold_active),
            .adaptive_enable(adaptive_active),.edge_select(edge_active),
            .adaptive_decrease_allowed(adaptive_decrease_camera),
            .adaptive_rate_floor_high(adaptive_rate_floor_camera),
            .output_vsync(processed_vsync),.output_de(processed_de),
            .output_x(),.output_y(),.output_magnitude(),.output_class(canny_class),.output_direction(canny_direction),
            .output_rgb565(processed_rgb565),.output_original_gray(),.output_filtered_gray(canny_filtered),
            .output_threshold_low(canny_threshold_low),.output_threshold_high(canny_threshold_high),
            .output_scene_samples(canny_scene_samples),.output_scene_high(canny_scene_high));
    end else begin: g_sobel_baseline
    Image_Process_Pipeline #(
        .DATA_WIDTH(16), .ACTIVE_WIDTH(SENSOR_WIDTH), .ACTIVE_HEIGHT(SENSOR_HEIGHT),.RUNTIME_FILTER(1)
    ) image_pipeline (
        .gaussian_select(gaussian_active),
        .clk(cmos_pclk), .rst_n(reset_n_sys),
        .input_vsync(selected_vsync), .input_de(selected_de), .input_data(selected_rgb565),
        .mode_select(mode_active), .edge_select(edge_active),
        .edge_threshold(threshold_active),
        .output_vsync(processed_vsync), .output_de(processed_de), .output_data(processed_rgb565)
    );

        assign canny_class=0;
        assign canny_direction=0;
        assign canny_filtered=0;
        assign canny_threshold_low=0;
        assign canny_threshold_high=threshold_active;
        assign canny_scene_samples=0;
        assign canny_scene_high=0;
    end endgenerate

    // DDR3 AXI connection.
    wire [3:0] axi_awid;
    wire [31:0] axi_awaddr;
    wire [7:0] axi_awlen;
    wire axi_awvalid, axi_awready;
    wire [127:0] axi_wdata;
    wire [15:0] axi_wstrb;
    wire axi_wlast, axi_wvalid, axi_wready;
    wire [3:0] axi_bid;
    wire [1:0] axi_bresp;
    wire axi_bvalid;
    wire [3:0] axi_arid;
    wire [31:0] axi_araddr;
    wire [7:0] axi_arlen;
    wire axi_arvalid, axi_arready;
    wire [3:0] axi_rid;
    wire [127:0] axi_rdata;
    wire [1:0] axi_rresp;
    wire axi_rlast, axi_rvalid, axi_rready;
    wire [3:0] axi_aid;
    wire [31:0] axi_aaddr;
    wire [7:0] axi_alen;
    wire axi_atype, axi_avalid, axi_aready;
    wire ddr_cal_done, ddr_cal_pass;
    reg ddr_unlocked = 1'b0;
    generate if(PROFILE_RECONFIG_ENABLE) begin : profile_drain
        Profile_DDR_Quiesce drain (
            .clk(clk_sys),.rst_n(reset_n_sys),.request(profile_pause),
            .tracking_enable(ddr_unlocked && ddr_cal_pass),
            .avalid(axi_avalid && ddr_unlocked),.aready(axi_aready),.atype(axi_atype),
            .bvalid(axi_bvalid),.rvalid(axi_rvalid),.rready(axi_rready),.rlast(axi_rlast),
            .awready(axi_awready),.arready(axi_arready),.idle(profile_traffic_idle),.error(profile_traffic_error));
    end else begin : no_profile_drain
        assign profile_traffic_idle=0;assign profile_traffic_error=0;
    end endgenerate
    always @(posedge clk_sys or negedge reset_n_sys)
        if (!reset_n_sys) ddr_unlocked <= 1'b0;
        else ddr_unlocked <= ddr_cal_done;

    DdrCtrl ddr_controller (
        .core_clk(core_clk), .tac_clk(tac_clk), .twd_clk(twd_clk), .tdqss_clk(tdqss_clk),
        .reset(ddr_reset), .cs(ddr_cs), .ras(ddr_ras), .cas(ddr_cas), .we(ddr_we),
        .cke(ddr_cke), .addr(ddr_addr), .ba(ddr_ba), .odt(ddr_odt),
        .o_dm_hi(ddr_dm_hi_o), .o_dm_lo(ddr_dm_lo_o),
        .i_dq_hi(ddr_dq_hi_i), .i_dq_lo(ddr_dq_lo_i),
        .o_dq_hi(ddr_dq_hi_o), .o_dq_lo(ddr_dq_lo_o), .o_dq_oe(ddr_dq_oe),
        .i_dqs_hi(ddr_dqs_hi_i), .i_dqs_lo(ddr_dqs_lo_i),
        .i_dqs_n_hi(ddr_dqs_n_hi_i), .i_dqs_n_lo(ddr_dqs_n_lo_i),
        .o_dqs_hi(ddr_dqs_hi_o), .o_dqs_lo(ddr_dqs_lo_o),
        .o_dqs_n_hi(ddr_dqs_n_hi_o), .o_dqs_n_lo(ddr_dqs_n_lo_o),
        .o_dqs_oe(ddr_dqs_oe), .o_dqs_n_oe(ddr_dqs_n_oe),
        .clk(clk_sys), .reset_n(reset_n_sys),
        .axi_avalid(axi_avalid & ddr_unlocked), .axi_aready(axi_aready),
        .axi_aaddr(axi_aaddr), .axi_aid(axi_aid), .axi_alen(axi_alen),
        .axi_asize(3'd4), .axi_aburst(2'd1), .axi_alock(2'd0), .axi_atype(axi_atype),
        .axi_wid(4'd0), .axi_wvalid(axi_wvalid), .axi_wready(axi_wready),
        .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb), .axi_wlast(axi_wlast),
        .axi_bvalid(axi_bvalid), .axi_bready(1'b1), .axi_bid(axi_bid), .axi_bresp(axi_bresp),
        .axi_rvalid(axi_rvalid), .axi_rready(axi_rready), .axi_rdata(axi_rdata),
        .axi_rid(axi_rid), .axi_rresp(axi_rresp), .axi_rlast(axi_rlast),
        .shift(ddr_shift), .shift_sel(), .shift_ena(ddr_shift_ena),
        .cal_ena(1'b1), .cal_done(ddr_cal_done), .cal_pass(ddr_cal_pass)
    );
    assign ddr_shift_sel = 5'b00100;

    DDR_AXI_Arbiter #(.AID_LEN(4), .AADDR_LEN(32)) ddr_axi_arbiter (
        .aclk_i(clk_sys), .arst_i(~reset_n_sys),
        .awid_i(axi_awid), .awaddr_i(axi_awaddr), .awlen_i(axi_awlen),
        .awvalid_i(axi_awvalid && !profile_pause), .awready_o(axi_awready),
        .arid_i(axi_arid), .araddr_i(axi_araddr), .arlen_i(axi_arlen),
        .arvalid_i(axi_arvalid && !profile_pause), .arready_o(axi_arready),
        .aid_o(axi_aid), .aaddr_o(axi_aaddr), .alen_o(axi_alen),
        .atype_o(axi_atype), .avalid_o(axi_avalid), .aready_i(axi_aready)
    );

    wire display_request;
    wire display_hsync, display_vsync, display_de;
    wire display_blanking,display_read_valid,display_metadata_valid;
    wire frame_metadata_valid,frame_metadata_external,frame_shape_enabled;
    wire [31:0] frame_metadata_tag;
    reg display_source_media;
    reg [31:0] display_source_base;
    wire photo_metadata_valid;
    wire [19:0] photo_word;
    wire [4:0] photo_word_index;
    wire photo_word_valid,photo_word_ready;
    wire [2:0] display_read_index;
    wire [13:0] completed_algorithm,display_algorithm;
    wire [SHAPE_TARGETS*51-1:0] completed_shapes,display_shapes;
    wire [SHAPE_TARGETS*4-1:0] completed_digits,display_digits;
    wire [SHAPE_TARGETS*2-1:0] completed_qualities,display_qualities;
    wire [SHAPE_TARGETS*8-1:0] completed_ids,display_ids,live_ids,live_ages;
    wire [SHAPE_TARGETS*8-1:0] live_lost;
    wire [SHAPE_TARGETS-1:0] live_trail_reset;
    wire [SHAPE_TARGETS*13-1:0] live_vx,live_vy;
    wire [SHAPE_TARGETS*51-1:0] live_boxes;
    wire [SHAPE_TARGETS*4-1:0] live_digits;
    wire [SHAPE_TARGETS-1:0] live_alive,live_observed;
    wire [31:0] canny_failed_frames;
    wire trail_ready,trail_dot,trail_outline,uart_source_safe;
    wire [SHAPE_TARGETS*2-1:0] live_qualities;
    wire [124:0] osd_playback;
    reg shape_visible;
    wire analysis_start,analysis_valid,analysis_edge,analysis_ready;
    wire [11:0] analysis_x,analysis_y;
    wire [15:0] display_rgb565;
    wire [11:0] display_x, display_y;
    wire [31:0] frame_debug;
    wire [3:0] fb_awid,fb_bid,fb_arid,fb_rid;
    wire [31:0] fb_awaddr,fb_araddr,ca_awaddr,ca_araddr;
    wire [7:0] fb_awlen,fb_arlen,ca_awlen,ca_arlen;
    wire [127:0] fb_wdata,fb_rdata,ca_wdata;
    wire [15:0] fb_wstrb;
    wire [1:0] fb_bresp,fb_rresp;
    wire fb_awvalid,fb_awready,fb_wvalid,fb_wready,fb_wlast,fb_bvalid;
    wire fb_arvalid,fb_arready,fb_rvalid,fb_rready,fb_rlast;
    wire ca_awvalid,ca_wvalid,ca_wlast,ca_arvalid,ca_arready,ca_rvalid,ca_rready;
    wire canny_complete;
    wire metadata_publish_ready;
    wire [143:0] canny_profile_bcd;
    wire canny_profile_valid;
    wire raw_locked,raw_busy,raw_request,raw_done,raw_error;
    wire [31:0] raw_locked_base;
    wire [15:0] raw_diagnostics;
    wire snapshot_request,snapshot_ready,snapshot_error;
    wire [2:0] snapshot_slot;
    localparam RAW_BYTES=((DISPLAY_WIDTH*DISPLAY_HEIGHT+2047)/2048)*2048;
    localparam PHOTO_BYTES=DISPLAY_WIDTH*DISPLAY_HEIGHT*2+RAW_BYTES+2048;
    localparam [31:0] RAW_CAPTURE_BASE=32'h04000000+MEDIA_PHOTO_FRAMES*PHOTO_BYTES+
        (MEDIA_VIDEO_FRAMES+MEDIA_PRE_FRAMES)*DISPLAY_WIDTH*DISPLAY_HEIGHT*2;
    wire [2:0] completed_mode;
    wire [1023:0] raw_header;
    reg [31:0] raw_frame_sequence;
    always @(posedge clk_sys or negedge reset_n_sys)
        if(!reset_n_sys) raw_frame_sequence<=0;
        else if(canny_complete) raw_frame_sequence<=raw_frame_sequence+1'b1;
    // Little-endian RAW1 v1 header, offsets documented in tools/uart/raw_photo.py.
    assign raw_header[31:0]=32'h31574152;
    assign raw_header[47:32]=16'd1;
    assign raw_header[63:48]=DISPLAY_WIDTH;
    assign raw_header[79:64]=DISPLAY_HEIGHT;
    assign raw_header[87:80]={6'd0,EFFECTIVE_BAYER_PATTERN};
    assign raw_header[95:88]=8'd8;
    assign raw_header[111:96]=R_GAIN_Q8;
    assign raw_header[127:112]=G_GAIN_Q8;
    assign raw_header[143:128]=B_GAIN_Q8;
    assign raw_header[159:144]=BLACK_LEVEL;
    assign raw_header[167:160]=GAMMA_ENABLE;
    assign raw_header[175:168]={5'd0,completed_mode};
    assign raw_header[183:176]={7'd0,completed_algorithm[12]};
    assign raw_header[191:184]={7'd0,completed_algorithm[13]};
    assign raw_header[207:192]={4'd0,completed_algorithm[11:0]};
    assign raw_header[223:208]={5'd0,completed_algorithm[11:1]};
    assign raw_header[239:224]=EDGE_MIN_COMPONENT_PIXELS;
    assign raw_header[255:240]=EDGE_AREA_TRACK_LABELS;
    assign raw_header[287:256]=raw_frame_sequence;
    assign raw_header[295:288]=SHAPE_TARGETS;
    assign raw_header[303:296]={7'd0,shape_enable_sys};
    assign raw_header[311:304]=SENSOR_ROW_MIRROR*2+SENSOR_COLUMN_MIRROR;
    assign raw_header[319:312]=0;
    assign raw_header[639:320]={{(320-SHAPE_TARGETS*51){1'b0}},completed_shapes};
    assign raw_header[671:640]={{(32-SHAPE_TARGETS*4){1'b0}},completed_digits};
    assign raw_header[703:672]={{(32-SHAPE_TARGETS*2){1'b0}},completed_qualities};
    assign raw_header[767:704]={{(64-SHAPE_TARGETS*8){1'b0}},completed_ids};
    assign raw_header[799:768]=DISPLAY_WIDTH*DISPLAY_HEIGHT;
    assign raw_header[831:800]=BUILD_PROFILE_ID;
    assign raw_header[863:832]=EXPOSURE_LINES;
    assign raw_header[895:864]=ANALOG_GAIN;
    assign raw_header[1023:896]=0;
    wire [143:0] canny_out_profile_bcd;
    wire canny_out_profile_valid;
    wire [143:0] canny_schedule_bcd;
    wire canny_schedule_valid;
    wire [191:0] canny_edge_stats_bcd;
    wire canny_edge_stats_valid;
    wire [7:0] canny_startup_events;
    wire [2:0] canny_slot;
    wire [31:0] core_awaddr,core_araddr,media_araddr,media_awaddr,playback_base;
    wire [7:0] core_awlen,core_arlen,media_arlen,media_awlen,media_pins;
    wire [127:0] core_wdata,media_wdata;
    wire core_awvalid,core_awready,core_wvalid,core_wready,core_wlast,core_bvalid;
    wire core_arvalid,core_arready,core_rvalid,core_rready;
    wire media_arvalid,media_arready,media_rvalid,media_rready;
    wire media_awvalid,media_awready,media_wvalid,media_wready,media_wlast,media_bvalid;
    wire playback_wanted,media_read_active;
    wire [31:0] media_active_base;
    assign axi_awid=0;
    assign axi_wstrb=16'hffff;
    assign core_awaddr=CANNY_ENABLE?ca_awaddr:fb_awaddr;
    assign core_awlen=CANNY_ENABLE?ca_awlen:fb_awlen;
    assign core_awvalid=CANNY_ENABLE?ca_awvalid:fb_awvalid;
    assign core_wdata=CANNY_ENABLE?ca_wdata:fb_wdata;
    assign core_wvalid=CANNY_ENABLE?ca_wvalid:fb_wvalid;
    assign core_wlast=CANNY_ENABLE?ca_wlast:fb_wlast;
    assign fb_awready=!CANNY_ENABLE && core_awready;
    assign fb_wready=!CANNY_ENABLE && core_wready;
    assign fb_bvalid=!CANNY_ENABLE && core_bvalid;
    AXI_Write_Arbiter_2 media_write_arbiter (
        .clk(clk_sys),.rst_n(reset_n_sys),
        .p0_addr(core_awaddr),.p0_len(core_awlen),.p0_valid(core_awvalid),.p0_ready(core_awready),
        .p0_data(core_wdata),.p0_wvalid(core_wvalid),.p0_wready(core_wready),.p0_wlast(core_wlast),.p0_bvalid(core_bvalid),
        .p1_addr(media_awaddr),.p1_len(media_awlen),.p1_valid(media_awvalid),.p1_ready(media_awready),
        .p1_data(media_wdata),.p1_wvalid(media_wvalid),.p1_wready(media_wready),.p1_wlast(media_wlast),.p1_bvalid(media_bvalid),
        .awaddr(axi_awaddr),.awlen(axi_awlen),.awvalid(axi_awvalid),.awready(axi_awready),
        .wdata(axi_wdata),.wvalid(axi_wvalid),.wready(axi_wready),.wlast(axi_wlast),.bvalid(axi_bvalid));
    AXI_Read_Arbiter_2 #(.ROUND_ROBIN(1)) media_read_arbiter (
        .clk(clk_sys),.rst_n(reset_n_sys),
        .p0_addr(core_araddr),.p0_len(core_arlen),.p0_valid(core_arvalid),.p0_ready(core_arready),
        .p0_rvalid(core_rvalid),.p0_rready(core_rready),
        .p1_addr(media_araddr),.p1_len(media_arlen),.p1_valid(media_arvalid),.p1_ready(media_arready),
        .p1_rvalid(media_rvalid),.p1_rready(media_rready),
        .araddr(axi_araddr),.arlen(axi_arlen),.arvalid(axi_arvalid),.arready(axi_arready),
        .rvalid(axi_rvalid),.rlast(axi_rlast),.rready(axi_rready));
    assign fb_bid=axi_bid;
    assign fb_bresp=axi_bresp;
    assign fb_rid=axi_rid;
    assign fb_rdata=axi_rdata;
    assign fb_rresp=axi_rresp;
    assign fb_rlast=axi_rlast;
    assign axi_arid=4'd0;
    generate if(CANNY_ENABLE) begin: g_canny_engine
        if(CANNY_FRAME_PIPELINE) begin: g_overlap
        Canny_Frame_Pipeline_Engine #(.WIDTH(SENSOR_WIDTH),.HEIGHT(SENSOR_HEIGHT),
            .RAW_EXPORT_ENABLE(1),.RAW_BASE(RAW_CAPTURE_BASE),
            .OUT_PROFILE_ENABLE(CANNY_OUT_PROFILE_ENABLE),
            .EDGE_DIAGNOSTIC_ENABLE(EDGE_DIAGNOSTIC_ENABLE),
            .MIN_COMPONENT_PIXELS(EDGE_MIN_COMPONENT_PIXELS),.AREA_TRACK_LABELS(EDGE_AREA_TRACK_LABELS),
            .PROFILE_ENABLE(CANNY_PROFILE_ENABLE),.CLOCK_HZ(SYSTEM_CLOCK_HZ),
            .DIAGNOSTIC_ENABLE(CANNY_DIAGNOSTIC_ENABLE),.METADATA_ENABLE(1),.METADATA_WIDTH(14),
            .RESULT_WAIT_ENABLE(1),.DEFAULT_METADATA({1'b0,EDGE_PROCESS_SELECT,SOBEL_THRESHOLD})) engine (
            .clk(clk_sys),.rst_n(reset_n_sys),.enable(ddr_unlocked && ddr_cal_pass && config_done && !profile_pause),
            .pclk(cmos_pclk),.vs(processed_vsync),.de(processed_de),
            .raw_vs(raw_tap_vsync),.raw_de(raw_tap_de),.raw_data(raw_tap_bayer),
            .export_header(raw_header),.completed_mode(completed_mode),
            .snapshot_request(snapshot_request),.snapshot_ready(snapshot_ready),
            .snapshot_error(snapshot_error),.snapshot_slot(snapshot_slot),
            .raw_diagnostics(raw_diagnostics),
            .rgb565(processed_rgb565),.median_gray(canny_filtered),.candidate_class(canny_class),.gradient_direction(canny_direction),
            .frame_metadata({gaussian_active,edge_active,canny_threshold_high}),.completed_metadata(completed_algorithm),
            .analysis_start(analysis_start),.analysis_valid(analysis_valid),.analysis_edge(analysis_edge),
            .analysis_x(analysis_x),.analysis_y(analysis_y),.analysis_ready(analysis_ready && metadata_publish_ready && trail_ready),
            .mode_request(mode_request_sys),.write_slot(canny_slot),.frame_complete(canny_complete),
            .completed_frames(),.failed_frames(canny_failed_frames),.last_cycles(),
            .profile_ms_bcd(canny_profile_bcd),.profile_valid(canny_profile_valid),
            .out_profile_bcd(canny_out_profile_bcd),.out_profile_valid(canny_out_profile_valid),
            .schedule_bcd(canny_schedule_bcd),.schedule_valid(canny_schedule_valid),
            .edge_stats_bcd(canny_edge_stats_bcd),.edge_stats_valid(canny_edge_stats_valid),
            .startup_events(canny_startup_events),
            .awaddr(ca_awaddr),.awlen(ca_awlen),.awvalid(ca_awvalid),.awready(core_awready),
            .wvalid(ca_wvalid),.wlast(ca_wlast),.wdata(ca_wdata),.wready(core_wready),
            .bvalid(core_bvalid),.bresp(axi_bresp),
            .araddr(ca_araddr),.arlen(ca_arlen),.arvalid(ca_arvalid),.arready(ca_arready),
            .rvalid(ca_rvalid),.rready(ca_rready),.rlast(axi_rlast),.rresp(axi_rresp),.rdata(axi_rdata));
        end else begin: g_serial
        assign raw_diagnostics=0;
        assign completed_mode=0;
        assign canny_out_profile_bcd=0;assign canny_out_profile_valid=0;
        assign canny_edge_stats_bcd=0;assign canny_edge_stats_valid=0;
        assign canny_startup_events=0;
        Canny_Frame_Engine #(.WIDTH(SENSOR_WIDTH),.HEIGHT(SENSOR_HEIGHT),
            .MIN_COMPONENT_PIXELS(EDGE_MIN_COMPONENT_PIXELS),.AREA_TRACK_LABELS(EDGE_AREA_TRACK_LABELS),
            .PROFILE_ENABLE(CANNY_PROFILE_ENABLE),.CLOCK_HZ(SYSTEM_CLOCK_HZ),
            .DIAGNOSTIC_ENABLE(CANNY_DIAGNOSTIC_ENABLE),.METADATA_ENABLE(1),.METADATA_WIDTH(14),
            .RESULT_WAIT_ENABLE(1),.DEFAULT_METADATA({1'b0,EDGE_PROCESS_SELECT,SOBEL_THRESHOLD})) engine (
            .clk(clk_sys),.rst_n(reset_n_sys),.enable(ddr_unlocked && ddr_cal_pass && config_done && !profile_pause),
            .pclk(cmos_pclk),.vs(processed_vsync),.de(processed_de),
            .rgb565(processed_rgb565),.median_gray(canny_filtered),.candidate_class(canny_class),.gradient_direction(canny_direction),
            .frame_metadata({gaussian_active,edge_active,canny_threshold_high}),.completed_metadata(completed_algorithm),
            .analysis_start(analysis_start),.analysis_valid(analysis_valid),.analysis_edge(analysis_edge),
            .analysis_x(analysis_x),.analysis_y(analysis_y),.analysis_ready(analysis_ready && metadata_publish_ready && trail_ready),
            .mode_request(mode_request_sys),.write_slot(canny_slot),.frame_complete(canny_complete),
            .completed_frames(),.failed_frames(canny_failed_frames),.last_cycles(),
            .profile_ms_bcd(canny_profile_bcd),.profile_valid(canny_profile_valid),
            .schedule_bcd(canny_schedule_bcd),.schedule_valid(canny_schedule_valid),
            .awaddr(ca_awaddr),.awlen(ca_awlen),.awvalid(ca_awvalid),.awready(core_awready),
            .wvalid(ca_wvalid),.wlast(ca_wlast),.wdata(ca_wdata),.wready(core_wready),
            .bvalid(core_bvalid),.bresp(axi_bresp),
            .araddr(ca_araddr),.arlen(ca_arlen),.arvalid(ca_arvalid),.arready(ca_arready),
            .rvalid(ca_rvalid),.rready(ca_rready),.rlast(axi_rlast),.rresp(axi_rresp),.rdata(axi_rdata));
        end
        AXI_Read_Arbiter_2 read_arbiter (
            .clk(clk_sys),.rst_n(reset_n_sys),
            .p0_addr(fb_araddr),.p0_len(fb_arlen),.p0_valid(fb_arvalid),.p0_ready(fb_arready),
            .p0_rvalid(fb_rvalid),.p0_rready(fb_rready),
            .p1_addr(ca_araddr),.p1_len(ca_arlen),.p1_valid(ca_arvalid),.p1_ready(ca_arready),
            .p1_rvalid(ca_rvalid),.p1_rready(ca_rready),
            .araddr(core_araddr),.arlen(core_arlen),.arvalid(core_arvalid),.arready(core_arready),
            .rvalid(core_rvalid),.rlast(axi_rlast),.rready(core_rready));
    end else begin: g_baseline_memory
        assign raw_diagnostics=0;
        assign completed_mode=0;
        assign canny_out_profile_bcd=0;assign canny_out_profile_valid=0;
        assign canny_complete=0;
        assign completed_algorithm={1'b0,EDGE_PROCESS_SELECT,SOBEL_THRESHOLD};
        assign analysis_start=0;assign analysis_valid=0;assign analysis_edge=0;
        assign analysis_x=0;assign analysis_y=0;
        assign canny_profile_bcd=0;assign canny_profile_valid=0;
        assign canny_schedule_bcd=0;assign canny_schedule_valid=0;
        assign canny_edge_stats_bcd=0;assign canny_edge_stats_valid=0;
        assign canny_startup_events=0;
        assign canny_failed_frames=0;
        assign ca_awaddr=0;assign ca_awlen=0;assign ca_awvalid=0;
        assign ca_wvalid=0;assign ca_wlast=0;assign ca_wdata=0;
        assign ca_araddr=0;assign ca_arlen=0;assign ca_arvalid=0;assign ca_rready=0;
        assign ca_arready=0;assign ca_rvalid=0;
        assign core_araddr=fb_araddr;assign core_arlen=fb_arlen;assign core_arvalid=fb_arvalid;
        assign fb_arready=core_arready;assign fb_rvalid=core_rvalid;assign core_rready=fb_rready;
    end endgenerate
    // One classifier consumes the exclusive OUT stream, while the other
    // worker can continue SCAN. Its results are committed with that same slot.
    generate if(CANNY_ENABLE && SHAPE_ENABLE) begin: g_shape
        wire [SHAPE_TARGETS*51-1:0] raw_shapes;
        wire [SHAPE_TARGETS*4-1:0] raw_digits;
        wire [SHAPE_TARGETS*2-1:0] raw_qualities;
        wire raw_ready;
        wire detector_reset_n,detector_start,detector_pixel_valid,tracked_ready;
        wire [SHAPE_TARGETS*51-1:0] tracked_shapes;
        wire [SHAPE_TARGETS*4-1:0] tracked_digits;
        wire [SHAPE_TARGETS*2-1:0] tracked_qualities;
        localparam TRACK_IB=SHAPE_TARGETS<2?1:$clog2(SHAPE_TARGETS);
        wire [SHAPE_TARGETS*TRACK_IB-1:0] trail_banks;
        wire [SHAPE_TARGETS*51-1:0] track_raw_boxes;
        Shape_Runtime_Control #(.TARGETS(SHAPE_TARGETS)) runtime_control (
            .clk(clk_sys),.rst_n(reset_n_sys),.enable(shape_enable_sys),
            .analysis_start(analysis_start),.analysis_valid(analysis_valid),
            .tracked_ready(tracked_ready),.tracked_shapes(tracked_shapes),.tracked_digits(tracked_digits),
            .tracked_qualities(tracked_qualities),
            .detector_reset_n(detector_reset_n),.detector_start(detector_start),
            .detector_pixel_valid(detector_pixel_valid),.analysis_ready(analysis_ready),
            .completed_shapes(completed_shapes),.completed_digits(completed_digits),
            .completed_qualities(completed_qualities));
        Shape_Detector #(.WIDTH(SENSOR_WIDTH),.HEIGHT(SENSOR_HEIGHT),
            .TARGETS(SHAPE_TARGETS),.TRIANGLE_ENABLE(1),.DIGIT_ENABLE(1),
            .DIGIT_DEBUG_ENABLE(0)) detector (
            .clk(clk_sys),.rst_n(detector_reset_n),.start(detector_start),.pixel_valid(detector_pixel_valid),
            .pixel_edge(analysis_edge),.pixel_x(analysis_x),.pixel_y(analysis_y),
            .result_valid(raw_ready),.results(raw_shapes),.qualities(raw_qualities),.digits(raw_digits),
            .debug_enable(1'b0),.debug_ready(1'b0),.debug_select(4'd0),.debug_threshold(12'd0));
        wire track_snapshot_ready;
        Shape_Target_Track #(.TARGETS(SHAPE_TARGETS),.PROFILE_FPS(DISPLAY_FPS)) confirmation (
            .clk(clk_sys),.rst_n(detector_reset_n),.clear(detector_start),.raw_valid(raw_ready),.publish(canny_complete),
            .raw_results(raw_shapes),.raw_digits(raw_digits),.qualities(raw_qualities),
            .ready(tracked_ready),.confirmed(tracked_shapes),.confirmed_digits(tracked_digits),
            .confirmed_qualities(tracked_qualities),
            .track_ids(live_ids),.track_ages(live_ages),.track_lost(live_lost),
            .track_vx(live_vx),.track_vy(live_vy),.track_banks(trail_banks),
            .track_boxes(live_boxes),.track_raw_boxes(track_raw_boxes),.track_digits(live_digits),
            .track_alive(live_alive),.track_observed(live_observed),.track_trail_reset(live_trail_reset),
            .track_snapshot_ready(track_snapshot_ready),.track_qualities(live_qualities));
        assign uart_source_safe=!shape_enable_sys || track_snapshot_ready;
        wire [SHAPE_TARGETS-1:0] trail_visible;
        genvar tv;
        for(tv=0;tv<SHAPE_TARGETS;tv=tv+1) begin: g_trail_visible
            assign trail_visible[tv]=tracked_shapes[tv*51+48 +: 3]!=0;
        end
        assign completed_ids=live_ids;
        Shape_Track_Trail #(.TARGETS(SHAPE_TARGETS),.WIDTH(DISPLAY_WIDTH),.HEIGHT(DISPLAY_HEIGHT)) trail (
            .clk(clk_sys),.rst_n(detector_reset_n),.publish(canny_complete),.frame_slot(canny_slot),
            .boxes(track_raw_boxes),.ids(live_ids),.reset_history(live_trail_reset),.banks(trail_banks),
            .observed(live_observed),.visible(trail_visible),.ready(trail_ready),
            .pixel_clk(clk_pixel),.pixel_rst_n(reset_n_pixel && shape_enable_sys),
            .frame_blanking(display_blanking),.x(display_x),.y(display_y),
            .display_slot(display_read_index),.display_ids(display_ids),
            .live_enable(shape_visible && !display_source_media),
            .dot(trail_dot),.outline(trail_outline));
    end else begin: g_no_shape
        assign analysis_ready=1;assign completed_shapes=0;assign completed_digits=0;
        assign completed_qualities=0;
        assign completed_ids=0;assign live_ids=0;assign live_ages=0;assign live_lost=0;
        assign live_vx=0;assign live_vy=0;assign live_boxes=0;assign live_digits=0;
        assign live_trail_reset=0;assign live_alive=0;assign live_observed=0;assign trail_ready=1;assign trail_dot=0;
        assign trail_outline=0;
        assign uart_source_safe=1;
        assign live_qualities=0;
    end endgenerate
    Video_Frame_Buffer #(
        .EXTERNAL_WRITE(CANNY_ENABLE),.MEDIA_ENABLE(1),
        .RAW_PHOTOS(CANNY_ENABLE && CANNY_FRAME_PIPELINE),
        .PHOTO_FRAMES(MEDIA_PHOTO_FRAMES),.SHAPE_TARGETS(SHAPE_TARGETS),
        .C_RD_END_ADDR(FRAME_BYTES), .C_W_WIDTH(16), .C_R_WIDTH(16), .C_ID_LEN(4)
    ) frame_buffer (
        .axi_clk(clk_sys), .axi_reset(~reset_n_sys),
        .external_write_complete(canny_complete), .external_write_index(canny_slot),
        .display_read_index(display_read_index),.display_read_valid(display_read_valid),
        .media_pin_mask(media_pins),.media_read_enable(playback_wanted),.media_read_base(playback_base),
        .rframe_blanking(display_blanking),.media_read_active(media_read_active),.media_active_base(media_active_base),
        .photo_metadata_valid(photo_metadata_valid),.photo_word(photo_word),
        .photo_word_index(photo_word_index),.photo_word_valid(photo_word_valid),.photo_word_ready(photo_word_ready),
        .axi_awid(fb_awid), .axi_awaddr(fb_awaddr), .axi_awlen(fb_awlen),
        .axi_awsize(), .axi_awburst(), .axi_awlock(), .axi_awcache(), .axi_awprot(), .axi_awqos(),
        .axi_awvalid(fb_awvalid), .axi_awready(fb_awready),
        .axi_wdata(fb_wdata), .axi_wstrb(fb_wstrb), .axi_wlast(fb_wlast),
        .axi_wvalid(fb_wvalid), .axi_wready(fb_wready),
        .axi_bid(fb_bid), .axi_bresp(fb_bresp), .axi_bvalid(fb_bvalid), .axi_bready(),
        .axi_arid(fb_arid), .axi_araddr(fb_araddr), .axi_arlen(fb_arlen),
        .axi_arsize(), .axi_arburst(), .axi_arlock(), .axi_arcache(), .axi_arprot(), .axi_arqos(),
        .axi_arvalid(fb_arvalid), .axi_arready(fb_arready),
        .axi_rid(fb_rid), .axi_rdata(fb_rdata), .axi_rresp(fb_rresp),
        .axi_rlast(fb_rlast), .axi_rvalid(fb_rvalid), .axi_rready(fb_rready),
        .wframe_pclk(cmos_pclk), .wframe_vsync(CANNY_ENABLE ? 1'b0 : processed_vsync),
        .wframe_data_en(CANNY_ENABLE ? 1'b0 : processed_de), .wframe_data(processed_rgb565),
        .rframe_pclk(clk_pixel), .rframe_vsync(~display_vsync),
        .rframe_data_en(display_request), .rframe_data(display_rgb565),
        .tp_o(frame_debug)
    );

    wire [23:0] display_rgb888;
    Video_Timing_Generator #(
        .ACTIVE_WIDTH(DISPLAY_WIDTH), .ACTIVE_HEIGHT(DISPLAY_HEIGHT),
        .H_FRONT(H_FRONT), .H_SYNC(H_SYNC), .H_BACK(H_BACK),
        .V_FRONT(V_FRONT), .V_SYNC(V_SYNC), .V_BACK(V_BACK)
    ) display_timing (
        .pixel_clk(clk_pixel), .rst_n(reset_n_pixel), .rgb565(display_rgb565),
        .pixel_request(display_request), .hsync(display_hsync), .vsync(display_vsync),
        .data_enable(display_de), .rgb888(display_rgb888),.frame_blanking(display_blanking),
        .active_x(display_x), .active_y(display_y)
    );

    // Keep camera-applied algorithm/threshold coherent, independently from
    // the actual DDR playback state and the live UI controls in clk_sys.
    wire [9:0] result_fps_sys,result_fps_pixel,camera_fps_sys,camera_fps_pixel;
    wire [31:0] camera_period_cycles,camera_frame_count;
    Video_Input_Frame_Rate #(.CLOCK_HZ(SYSTEM_CLOCK_HZ)) camera_rate (
        .clk(clk_sys),.rst_n(reset_n_sys),.camera_vsync(cmos_vsync),
        .fps_x10(camera_fps_sys),.period_cycles(camera_period_cycles),.frame_count(camera_frame_count));
    Video_Status_Sync #(.WIDTH(10),.DEFAULT_STATUS(10'd0)) camera_rate_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),.source_status(camera_fps_sys),
        .pixel_clk(clk_pixel),.pixel_status(camera_fps_pixel));
    wire [144:0] profile_pixel;
    wire [144:0] out_profile_pixel;
    wire [144:0] schedule_pixel;
    wire [192:0] edge_stats_pixel;
    // Independent of frame publication: diagnose a camera/DDR startup stall
    // even while all normal Canny timing reports still show "---".
    reg [2:0] camera_seen;
    always @(posedge cmos_pclk or negedge reset_n_sys)
        if(!reset_n_sys) camera_seen<=0;
        else camera_seen<=camera_seen | {processed_de,cmos_vsync,1'b1};
    (* async_reg = "true" *) reg [2:0] camera_seen_meta,camera_seen_sys;
    always @(posedge clk_sys or negedge reset_n_sys)
        if(!reset_n_sys) begin camera_seen_meta<=0;camera_seen_sys<=0;end
        else begin camera_seen_meta<=camera_seen;camera_seen_sys<=camera_seen_meta;end
    wire [14:0] startup_pixel;
    Video_Status_Sync #(.WIDTH(15),.DEFAULT_STATUS(15'd0)) startup_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),
        .source_status({canny_startup_events,camera_seen_sys,config_error,
                        config_done,ddr_cal_done,ddr_cal_pass}),
        .pixel_clk(clk_pixel),.pixel_status(startup_pixel));
    Video_Status_Sync #(.WIDTH(193),.DEFAULT_STATUS(193'd0)) edge_stats_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),
        .source_status({canny_edge_stats_valid,canny_edge_stats_bcd}),
        .pixel_clk(clk_pixel),.pixel_status(edge_stats_pixel));
    Video_Status_Sync #(.WIDTH(145),.DEFAULT_STATUS(145'd0)) schedule_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),
        .source_status({canny_schedule_valid,canny_schedule_bcd}),
        .pixel_clk(clk_pixel),.pixel_status(schedule_pixel));
    // Both converters publish on the same clock after the same 384-cycle
    // conversion. Cross PIPE and OUT DETAIL in a single coherent handshake.
    Video_Status_Sync #(.WIDTH(290),.DEFAULT_STATUS(290'd0)) profile_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),
        .source_status({canny_out_profile_valid,canny_out_profile_bcd,canny_profile_valid,canny_profile_bcd}),
        .pixel_clk(clk_pixel),.pixel_status({out_profile_pixel,profile_pixel}));
    Video_Frame_Rate #(.CLOCK_HZ(SYSTEM_CLOCK_HZ),.AVERAGE_LOG2(4)) result_rate (
        .clk(clk_sys),.rst_n(reset_n_sys),.frame_done(canny_complete),.fps_x10(result_fps_sys));
    generate if(AUTO_RATE_GUARD_ENABLE) begin: g_auto_rate_guard
    Canny_Threshold_Rate_Guard #(.CLOCK_HZ(SYSTEM_CLOCK_HZ),.TARGET_FPS(BUILD_PROFILE_FPS)) threshold_rate_guard (
        .clk(clk_sys),.rst_n(reset_n_sys),.frame_done(canny_complete),
        .adaptive_enable(adaptive_request_sys),
        .applied_high(canny_feedback_sys[43:32]),.scene_high(canny_feedback_sys[31:20]),
        .scene_samples(canny_feedback_sys[19:0]),
        .rate_floor_high(adaptive_rate_floor_sys),
        .decrease_allowed(adaptive_decrease_sys));
    Video_Status_Sync #(.WIDTH(44),.DEFAULT_STATUS({SOBEL_THRESHOLD,SOBEL_THRESHOLD,20'd0})) rate_feedback_sync (
        .rst_n(reset_n_sys),.source_clk(cmos_pclk),
        .source_status({canny_threshold_high,canny_scene_high,canny_scene_samples}),
        .pixel_clk(clk_sys),.pixel_status(canny_feedback_sys));
    Video_Status_Sync #(.WIDTH(13),.DEFAULT_STATUS({1'b0,SOBEL_THRESHOLD})) rate_control_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),
        .source_status({adaptive_decrease_sys,adaptive_rate_floor_sys}),
        .pixel_clk(cmos_pclk),.pixel_status({adaptive_decrease_camera,adaptive_rate_floor_camera}));
    end else begin: g_auto_image_only
        assign adaptive_decrease_sys=1'b1;
        assign adaptive_decrease_camera=1'b1;
        assign adaptive_rate_floor_sys=12'd16;
        assign adaptive_rate_floor_camera=12'd16;
        assign canny_feedback_sys=44'd0;
    end endgenerate
    Video_Status_Sync #(.WIDTH(10),.DEFAULT_STATUS(10'd0)) result_rate_sync (
        .rst_n(reset_n_sys),.source_clk(clk_sys),.source_status(result_fps_sys),
        .pixel_clk(clk_pixel),.pixel_status(result_fps_pixel));
    wire [13:0] osd_algorithm,live_algorithm;
    // One coherent bundle: mask, cursor, reserved acknowledgement, mode and
    // the original playback status. No independently sampled UI fields.
    wire [54:0] ui_media_status_sys;
    wire [2:0] ui_feedback_code_sys;
    wire ui_feedback_toggle_sys;
    reg ui_feedback_seen;
    wire [2:0] ui_feedback_pixel=(osd_playback[32]!=ui_feedback_seen)?osd_playback[35:33]:3'd0;
    wire dma_start,dma_done,dma_error;
    wire [31:0] dma_source,dma_destination;
    wire media_source_overrun,media_transfer_error,media_tick_overrun;
    wire media_photo_busy,media_pre_ready,media_record_ready;
    Media_Control #(.PIXELS(DISPLAY_WIDTH*DISPLAY_HEIGHT),.CLOCK_HZ(SYSTEM_CLOCK_HZ),
        .RAW_PHOTOS(CANNY_ENABLE && CANNY_FRAME_PIPELINE),
        .PHOTO_FRAMES(MEDIA_PHOTO_FRAMES),.VIDEO_FRAMES(MEDIA_VIDEO_FRAMES),.RING_FRAMES(MEDIA_PRE_FRAMES)) media_control (
        .clk(clk_sys),.rst_n(reset_n_sys),.enable(ddr_unlocked && ddr_cal_pass && config_done && !profile_pause),
        .control_group(control_group),.live_index(display_read_index),.live_valid(display_read_valid),
        .export_lock(raw_locked),.export_base(raw_locked_base),
        .photo_capture_request(snapshot_request),.photo_capture_ready(snapshot_ready),
        .photo_capture_error(snapshot_error),.photo_capture_slot(snapshot_slot),
        .display_media_active(media_read_active),.display_media_base(media_active_base),.operation_request(ui_operation_request_sys),
        .live_pin_mask(media_pins),.playback_enable(playback_wanted),.playback_address(playback_base),
        .media_status(ui_media_status_sys),.feedback_code(ui_feedback_code_sys),.feedback_toggle(ui_feedback_toggle_sys),
        .dma_start(dma_start),.dma_source(dma_source),.dma_destination(dma_destination),
        .dma_done(dma_done),.dma_error(dma_error),.source_overrun(media_source_overrun),
        .transfer_error(media_transfer_error),.tick_overrun(media_tick_overrun),
        .photo_busy(media_photo_busy),.pre_ready(media_pre_ready),.record_ready(media_record_ready));
    wire [31:0] dma_araddr,raw_araddr;
    wire [7:0] dma_arlen,raw_arlen;
    wire dma_arvalid,dma_arready,dma_rvalid,dma_rready;
    wire raw_arvalid,raw_arready,raw_rvalid,raw_rready;
    wire [15:0] raw_block_index;
    wire [127:0] raw_block_data;
    Raw_Photo_Export #(.PIXELS(DISPLAY_WIDTH*DISPLAY_HEIGHT),.CLOCK_HZ(SYSTEM_CLOCK_HZ)) photo_export (
        .clk(clk_sys),.rst_n(reset_n_sys),
        .open(uart_control_write && uart_control_address==12),
        .close(uart_control_write && uart_control_address==13),
        .selected_base(playback_base),.request(raw_request),.block_index(raw_block_index),
        .locked(raw_locked),.locked_base(raw_locked_base),.busy(raw_busy),
        .done(raw_done),.error(raw_error),.block_data(raw_block_data),
        .araddr(raw_araddr),.arlen(raw_arlen),.arvalid(raw_arvalid),.arready(raw_arready),
        .rvalid(raw_rvalid),.rready(raw_rready),.rlast(axi_rlast),.rdata(axi_rdata),.rresp(axi_rresp));
    AXI_Read_Arbiter_2 #(.ROUND_ROBIN(1)) export_read_arbiter (
        .clk(clk_sys),.rst_n(reset_n_sys),
        .p0_addr(dma_araddr),.p0_len(dma_arlen),.p0_valid(dma_arvalid),.p0_ready(dma_arready),.p0_rvalid(dma_rvalid),.p0_rready(dma_rready),
        .p1_addr(raw_araddr),.p1_len(raw_arlen),.p1_valid(raw_arvalid),.p1_ready(raw_arready),.p1_rvalid(raw_rvalid),.p1_rready(raw_rready),
        .araddr(media_araddr),.arlen(media_arlen),.arvalid(media_arvalid),.arready(media_arready),.rvalid(media_rvalid),.rlast(axi_rlast),.rready(media_rready));
    Media_Burst_DMA media_dma (
        .clk(clk_sys),.rst_n(reset_n_sys),.start(dma_start),
        .source_address(dma_source),.destination_address(dma_destination),.busy(),.done(dma_done),.error(dma_error),
        .araddr(dma_araddr),.arlen(dma_arlen),.arvalid(dma_arvalid),.arready(dma_arready),
        .rvalid(dma_rvalid),.rready(dma_rready),.rlast(axi_rlast),.rdata(axi_rdata),.rresp(axi_rresp),
        .awaddr(media_awaddr),.awlen(media_awlen),.awvalid(media_awvalid),.awready(media_awready),
        .wdata(media_wdata),.wvalid(media_wvalid),.wready(media_wready),.wlast(media_wlast),
        .bvalid(media_bvalid),.bresp(axi_bresp));
    // Held event code + toggle share the status handshake, including repeated
    // identical SNAP events. Generate exactly one pixel-clock pulse per event.
    always @(posedge clk_pixel or negedge reset_n_pixel)
        if(!reset_n_pixel) ui_feedback_seen<=0;
        else ui_feedback_seen<=osd_playback[32];
    Video_Status_Sync #(
        .WIDTH(14), .DEFAULT_STATUS({1'b0,EDGE_PROCESS_SELECT, SOBEL_THRESHOLD})
    ) osd_algorithm_sync (
        .rst_n(reset_n_sys), .source_clk(cmos_pclk),
        .source_status({gaussian_active,edge_active,
                        (CANNY_ENABLE != 0 ? canny_threshold_high : threshold_active)}),
        .pixel_clk(clk_pixel), .pixel_status(live_algorithm)
    );
    Video_Frame_Metadata #(.WIDTH(SHAPE_TARGETS*65+15),.EXTERNAL_READ(1),
        .DEFAULT_DATA({1'b0,{(SHAPE_TARGETS*65){1'b0}},1'b0,EDGE_PROCESS_SELECT,SOBEL_THRESHOLD})) frame_metadata (
        .clk(clk_sys),.rst_n(reset_n_sys),.publish(canny_complete),.write_index(canny_slot),
        .publish_ready(metadata_publish_ready),
        .published_data({1'b1,completed_ids,completed_qualities,completed_digits,completed_shapes,completed_algorithm}),
        .external_select(media_read_active),.external_valid(photo_metadata_valid),
        .external_word_valid(photo_word_valid),.external_word_index(photo_word_index),
        .external_word_data(photo_word),.external_word_ready(photo_word_ready),.external_tag(media_active_base),
        .read_index(display_read_index),.read_valid(display_read_valid),
        .pixel_clk(clk_pixel),.frame_blanking(display_blanking),
        .frame_data({frame_shape_enabled,display_ids,display_qualities,display_digits,display_shapes,display_algorithm}),
        .frame_valid(frame_metadata_valid),.frame_external(frame_metadata_external),.frame_tag(frame_metadata_tag));
    // The independent source tag suppresses a stale/in-flight CDC record if a
    // DDR header is delayed beyond blanking. No live labels may enter playback.
    assign display_metadata_valid=frame_metadata_valid &&
        (display_source_media ? frame_metadata_external && frame_shape_enabled &&
            frame_metadata_tag==display_source_base : !frame_metadata_external);
    assign osd_algorithm=CANNY_ENABLE?display_algorithm:live_algorithm;
    Video_Status_Sync #(
        .WIDTH(125), .DEFAULT_STATUS({32'd0,3'd0,1'b0,1'b1,56'd0,SOBEL_THRESHOLD,1'b0,1'b1,3'b011,2'd0,4'd0,IMAGE_PROCESS_MODE,
                                    1'b0,1'b1,1'b0,3'd0})
    ) osd_playback_sync (
        .rst_n(reset_n_sys), .source_clk(clk_sys),
        .source_status({media_active_base,ui_media_status_sys[54:52],media_read_active,shape_enable_sys,ui_media_status_sys[51:0],ui_feedback_code_sys,ui_feedback_toggle_sys,
                        threshold_request_sys,osd_quick_hidden_sys,osd_all_enable_sys,
                        osd_mask_sys,control_item_sys,reserved_action_sys,mode_request_sys,
                        auto_threshold_enable_sys,osd_enable_sys,control_notice_sys,control_group}),
        .pixel_clk(clk_pixel), .pixel_status(osd_playback)
    );
    wire [11:0] canny_low_pixel = CANNY_ENABLE != 0 ?
        {1'b0,osd_algorithm[11:1]} : osd_algorithm[11:0]/3;
    wire osd_hsync, osd_vsync, osd_de;
    wire [23:0] osd_rgb888;
    wire [23:0] shape_rgb888;
    reg [11:0] overlay_x,overlay_y;
    reg overlay_visible;
    // Match the OSD's one-pixel register and change visibility in blanking only.
    always @(posedge clk_pixel or negedge reset_n_pixel)
        if(!reset_n_pixel) begin
            overlay_x<=0;overlay_y<=0;overlay_visible<=0;
        end else begin
            overlay_x<=display_x;overlay_y<=display_y;overlay_visible<=shape_visible;
        end
    always @(posedge clk_pixel or negedge reset_n_pixel)
        if(!reset_n_pixel) begin shape_visible<=0;display_source_media<=0;display_source_base<=0;end
        else if(display_blanking) begin
            shape_visible<=OSD_ENABLE && osd_playback[4] && osd_playback[88];
            display_source_media<=osd_playback[89];display_source_base<=osd_playback[124:93];
        end
    Shape_Labeled_Overlay #(.ID_ENABLE(1),.WIDTH(DISPLAY_WIDTH),.HEIGHT(DISPLAY_HEIGHT),.TARGETS(SHAPE_TARGETS)) shape_overlay (
        .enable(CANNY_ENABLE && SHAPE_ENABLE && overlay_visible),.data_enable(osd_de),
        // Metadata is frame-held and changes only while display_de=0. It needs
        // no duplicate per-pixel register after the registered OSD stage.
        .metadata_valid(display_metadata_valid),.x(overlay_x),.y(overlay_y),
        .rgb(overlay_visible && trail_dot?24'h00ffff:
             overlay_visible && trail_outline?24'h101820:osd_rgb888),
        .track_ids(display_ids),.shapes(display_shapes),.digits(display_digits),.output_rgb(shape_rgb888));
    Video_OSD #(.FPS_VALUE(DISPLAY_FPS),.LIVE_FPS(CANNY_ENABLE != 0),.FRAME_BLANK_ENABLE(1),.MENU_ENABLE(1),.RUNTIME_FILTER(1),
        .SCREEN_WIDTH(DISPLAY_WIDTH),.SCREEN_HEIGHT(DISPLAY_HEIGHT),.BUILD_PROFILE_ID(BUILD_PROFILE_ID),
        .PROFILE_MENU_ENABLE(1),.PHOTO_CAPACITY(MEDIA_PHOTO_FRAMES),
        // Preserve the existing 720p feedback duration exactly. Other builds
        // derive it from their actual pixel clock.
        .FEEDBACK_COUNT(BUILD_PROFILE_ID==0?37125000:PIXEL_CLOCK_HZ/2),
        .BORDER_COUNT(BUILD_PROFILE_ID==0?11137500:PIXEL_CLOCK_HZ*3/20),
        .EDGE_DIAGNOSTIC_ENABLE(CANNY_ENABLE && CANNY_FRAME_PIPELINE && EDGE_DIAGNOSTIC_ENABLE),
        .STARTUP_DIAGNOSTIC_ENABLE(CANNY_ENABLE && CANNY_FRAME_PIPELINE),
        .PROFILE_ENABLE(CANNY_ENABLE && CANNY_PROFILE_ENABLE),.PROFILE_INTERVAL(1),
        .OUT_PROFILE_ENABLE(CANNY_ENABLE && CANNY_FRAME_PIPELINE && CANNY_PROFILE_ENABLE && CANNY_OUT_PROFILE_ENABLE),
        .DIAGNOSTIC_ENABLE(CANNY_ENABLE && CANNY_DIAGNOSTIC_ENABLE)) display_osd (
        .pixel_clk(clk_pixel), .rst_n(reset_n_pixel),
        .input_hsync(display_hsync), .input_vsync(display_vsync),.frame_blanking(display_blanking),
        .shape_type(2'd0),.shape_type_second(2'd0),
        .input_de(display_de), .input_x(display_x), .input_y(display_y),
        .input_rgb888(display_rgb888), .edge_select(osd_algorithm[12]),
        .osd_enable(OSD_ENABLE != 0 && osd_playback[4]),
        .osd_mask(osd_playback[17:15]),.control_item(osd_playback[14:13]),
        .osd_all_enable(osd_playback[18]),.manual_threshold(osd_playback[31:20]),
        .filter_active(1'b1),.gaussian_select(osd_algorithm[13]),
        .shape_enable(osd_playback[88]),
        .requested_profile(profile_status_pixel[5:4]),.profile_switch_state(profile_status_pixel[3:0]),
        // UI descriptors only; this does not select or write DDR media pixels.
        .media_status({osd_playback[92:90],osd_playback[87:36]}),.feedback_event(ui_feedback_pixel),
        .reserved_action(osd_playback[12:9]),.display_mode(osd_playback[8:6]),
        .control_notice(OSD_ENABLE != 0 && osd_playback[3]),
        .control_group(osd_playback[2:0]),
        .auto_threshold_enable(osd_playback[5]),
        .low_threshold(canny_low_pixel), .high_threshold(osd_algorithm[11:0]),
        .measured_fps_x10(result_fps_pixel),.camera_fps_x10(camera_fps_pixel),
        .profile_ms_bcd(profile_pixel[143:0]),.profile_valid(profile_pixel[144]),
        .out_profile_bcd(out_profile_pixel[143:0]),.out_profile_valid(out_profile_pixel[144]),
        .schedule_bcd(schedule_pixel[143:0]),.schedule_valid(schedule_pixel[144]),
        .edge_stats_bcd(edge_stats_pixel[191:0]),.edge_stats_valid(edge_stats_pixel[192]),
        .startup_status(startup_pixel),
        .output_hsync(osd_hsync), .output_vsync(osd_vsync),
        .output_de(osd_de), .output_rgb888(osd_rgb888)
    );

    UART_Control #(.BAUD(2000000),.FRAME_STREAM_ENABLE(1),.TARGETS(SHAPE_TARGETS),.CLOCK_HZ(SYSTEM_CLOCK_HZ),
        .WIDTH(DISPLAY_WIDTH),.HEIGHT(DISPLAY_HEIGHT),.FPS(DISPLAY_FPS),.PROFILE_ID(BUILD_PROFILE_ID),
        .RAW_EXPORT_ENABLE(CANNY_ENABLE && CANNY_FRAME_PIPELINE),
        .PROFILE_SWITCH_ENABLE(PROFILE_RECONFIG_ENABLE),
        .THRESHOLD_MIN(SOBEL_THRESHOLD_MIN),.THRESHOLD_MAX(SOBEL_THRESHOLD_MAX)) status_uart (
        .clk(clk_sys),.rst_n(reset_n_sys),.publish(canny_complete),
        .rx(status_uart_rx),
        .raw_available(playback_wanted && media_read_active && media_active_base==playback_base &&
            ui_media_status_sys[51] && !ui_media_status_sys[25] && !media_photo_busy),
        .raw_locked(raw_locked),.raw_busy(raw_busy),.raw_done(raw_done),.raw_error(raw_error),
        .raw_data(raw_block_data),.raw_request(raw_request),.raw_index(raw_block_index),
        .raw_diagnostics(raw_diagnostics),
        .boxes(live_boxes),.digits(live_digits),.qualities(live_qualities),
        .lost(live_lost),.ids(live_ids),.ages(live_ages),.vx(live_vx),.vy(live_vy),
        .alive(live_alive),.observed(live_observed),.fps_x10(result_fps_sys),
        .failed_frames(canny_failed_frames[15:0]),
        .control_state({control_group,control_item_sys,mode_request_sys,osd_mask_sys,
            osd_quick_hidden_sys,osd_all_enable_sys,shape_enable_sys,auto_threshold_enable_sys,edge_request_sys}),
        .manual_threshold(threshold_request_sys),.applied_algorithm(completed_algorithm),
        .device_flags({media_source_overrun,media_transfer_error,media_tick_overrun,
            camera_seen_sys[2],config_error,config_done,ddr_cal_done,ddr_cal_pass}),
        .media_status(ui_media_status_sys),.live_ready(ddr_unlocked && ddr_cal_pass && config_done && display_read_valid),
        .photo_busy(media_photo_busy),.pre_ready(media_pre_ready),.record_ready(media_record_ready),
        .source_safe(uart_source_safe),.source_tag(shape_enable_sys),
        .key_event(key_event_sys),
        .profile_switch_busy(profile_switch_busy),
        .scan_ms(canny_profile_bcd[35:24]),.out_ms(canny_profile_bcd[59:48]),
        .gap_ms(canny_schedule_bcd[71:60]),.profile_valid(canny_profile_valid && canny_schedule_valid),
        .control_write(uart_control_write),.control_address(uart_control_address),.control_value(uart_control_value),
        .tx(status_uart_tx));
    // synthesis translate_off
    initial if(RAW_CAPTURE_BASE+4*RAW_BYTES>32'h20000000 ||
        DISPLAY_WIDTH*DISPLAY_HEIGHT*2+RAW_BYTES+2048>(1<<22))
        $fatal(1,"RAW photo DDR layout exceeds storage");
    // synthesis translate_on
    HDMI_Transmitter #(.ENABLE_OSERDES(0)) hdmi_transmitter (
        .PixelClk(clk_pixel), .aRst(1'b0), .aRst_n(reset_n_pixel),
        .bitflip_i(4'b0000), .oe_i(1'b1),
        .vid_pVSync(osd_vsync), .vid_pHSync(osd_hsync),
        .vid_pVDE(osd_de), .vid_pData(shape_rgb888), .SerialClk(1'b0),
        .TMDS_Clk_p(), .TMDS_Clk_n(), .TMDS_Data_p(), .TMDS_Data_n(),
        .txc_o(hdmi_clock), .txd0_o(hdmi_data0),
        .txd1_o(hdmi_data1), .txd2_o(hdmi_data2)
    );

    // Low three LEDs identify the K2 operation group; upper health LEDs stay.
    wire [2:0] group_led = (control_group == 0) ? 3'b001 :
                           (control_group == 1) ? 3'b010 :
                           (control_group == 2) ? 3'b100 :
                           (control_group == 3) ? 3'b111 :
                           (control_group == 4) ? 3'b110 : 3'b101;
    assign status_led = {config_done, ~config_error, ddr_cal_pass, group_led};
endmodule
