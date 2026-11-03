
// Efinity Top-level template
// Version: 2025.1.110
// Date: 2026-09-19 01:13

// Copyright (C) 2013 - 2025 Efinix Inc. All rights reserved.

// This file may be used as a starting point for Efinity synthesis top-level target.
// The port list here matches what is expected by Efinity constraint files generated
// by the Efinity Interface Designer.

// To use this:
//     #1)  Save this file with a different name to a different directory, where source files are kept.
//              Example: you may wish to save as MT9M001_HDMI_Top.v
//     #2)  Add the newly saved file into Efinity project as design file
//     #3)  Edit the top level entity in Efinity project to:  MT9M001_HDMI_Top
//     #4)  Insert design content.


module MT9M001_HDMI_Top #(
    // BUILD_PROFILE_PARAMETERS_BEGIN (injected only in isolated builds)
    parameter integer BUILD_PROFILE_ID = 0,
    parameter integer BUILD_PROFILE_WIDTH = 1280,
    parameter integer BUILD_PROFILE_HEIGHT = 720,
    parameter integer BUILD_PROFILE_FPS = 30,
    parameter integer PROFILE_PIXEL_CLOCK_HZ = 74400000,
    parameter integer PROFILE_ROW_START = 164,
    parameter integer PROFILE_COLUMN_START = 20,
    parameter integer PROFILE_HBLANK = 19,
    parameter integer PROFILE_VBLANK = 25,
    parameter integer PROFILE_EXPOSURE_LINES = 720,
    parameter integer PROFILE_H_FRONT = 110,
    parameter integer PROFILE_H_SYNC = 40,
    parameter integer PROFILE_H_BACK = 220,
    parameter integer PROFILE_V_FRONT = 5,
    parameter integer PROFILE_V_SYNC = 5,
    parameter integer PROFILE_V_BACK = 20,
    parameter integer PROFILE_SYSTEM_CLOCK_HZ = 96000000,
    parameter integer PROFILE_PHOTO_FRAMES = 8,
    parameter integer PROFILE_VIDEO_FRAMES = 112,
    parameter integer PROFILE_PRE_FRAMES = 128,
    parameter integer PROFILE_PIXEL_BITS = 16,
    parameter integer PROFILE_SHAPE_TARGETS = 6,
    parameter integer PROFILE_ROW_MIRROR = 1,
    parameter integer PROFILE_COLUMN_MIRROR = 1,
    parameter integer PROFILE_BAYER_PATTERN = 1,
    parameter integer PROFILE_R_GAIN_Q8 = 267,
    parameter integer PROFILE_G_GAIN_Q8 = 256,
    parameter integer PROFILE_B_GAIN_Q8 = 452,
    parameter integer PROFILE_BLACK_LEVEL = 0,
    parameter integer PROFILE_GAMMA_ENABLE = 1
    // BUILD_PROFILE_PARAMETERS_END
)(
  (* syn_peri_port = 0 *) input clk_24m,
  (* syn_peri_port = 0 *) input clk_25m,
  (* syn_peri_port = 0 *) input cfg_ERROR,
  (* syn_peri_port = 0 *) output [1:0] cfg_CBSEL,
  (* syn_peri_port = 0 *) output cfg_ENA,
  (* syn_peri_port = 0 *) output cfg_CONFIG,
  (* syn_peri_port = 0 *) input k1,
  (* syn_peri_port = 0 *) input k2,
  (* syn_peri_port = 0 *) input cmos_ctl1,
  (* syn_peri_port = 0 *) input [7:0] cmos_data,
  (* syn_peri_port = 0 *) input cmos_href,
  (* syn_peri_port = 0 *) input cmos_sdat_IN,
  (* syn_peri_port = 0 *) input cmos_vsync,
  (* syn_peri_port = 0 *) input sys_pll_lock,
  (* syn_peri_port = 0 *) input ddr_pll_lock,
  (* syn_peri_port = 0 *) input dsi_pll_lock,
  (* syn_peri_port = 0 *) input lvds_pll_lock,
  (* syn_peri_port = 0 *) input dsi_serclk_i,
  (* syn_peri_port = 0 *) input dsi_txcclk_i,
  (* syn_peri_port = 0 *) input dsi_refclk_i,
  (* syn_peri_port = 0 *) input tdqss_clk,
  (* syn_peri_port = 0 *) input tac_clk,
  (* syn_peri_port = 0 *) input dsi_byteclk_i,
  (* syn_peri_port = 0 *) input clk_pixel,
  (* syn_peri_port = 0 *) input clk_sys,
  (* syn_peri_port = 0 *) input clk_pixel_2x,
  (* syn_peri_port = 0 *) input clk_pixel_10x,
  (* syn_peri_port = 0 *) input clk_lvds_7x,
  (* syn_peri_port = 0 *) input twd_clk,
  (* syn_peri_port = 0 *) input csi_rxc_i,
  (* syn_peri_port = 0 *) input clk_lvds_1x,
  (* syn_peri_port = 0 *) input clk_27m,
  (* syn_peri_port = 0 *) input core_clk,
  (* syn_peri_port = 0 *) input cmos_pclk,
  (* syn_peri_port = 0 *) input clk_54m,
  (* syn_peri_port = 0 *) input csi_ctl0_i,
  (* syn_peri_port = 0 *) input csi_ctl1_i,
  (* syn_peri_port = 0 *) input csi_scl_i,
  (* syn_peri_port = 0 *) input csi_sda_i,
  (* syn_peri_port = 0 *) input [15:0] i_dq_hi,
  (* syn_peri_port = 0 *) input [15:0] i_dq_lo,
  (* syn_peri_port = 0 *) input [1:0] i_dqs_hi,
  (* syn_peri_port = 0 *) input [1:0] i_dqs_lo,
  (* syn_peri_port = 0 *) input [1:0] i_dqs_n_hi,
  (* syn_peri_port = 0 *) input [1:0] i_dqs_n_lo,
  (* syn_peri_port = 0 *) input [7:0] lcd_b7_0_i,
  (* syn_peri_port = 0 *) input [7:0] lcd_g7_0_i,
  (* syn_peri_port = 0 *) input [7:0] lcd_r7_0_i,
  (* syn_peri_port = 0 *) input lcd_tp_int_i,
  (* syn_peri_port = 0 *) input lcd_tp_scl_i,
  (* syn_peri_port = 0 *) input lcd_tp_sda_i,
  (* syn_peri_port = 0 *) input uart_rx_i,
  (* syn_peri_port = 0 *) input csi_rxc_lp_n_i,
  (* syn_peri_port = 0 *) input csi_rxc_lp_p_i,
  (* syn_peri_port = 0 *) input [7:0] csi_rxd0_hs_i,
  (* syn_peri_port = 0 *) input csi_rxd0_lp_n_i,
  (* syn_peri_port = 0 *) input csi_rxd0_lp_p_i,
  (* syn_peri_port = 0 *) input [7:0] csi_rxd1_hs_i,
  (* syn_peri_port = 0 *) input csi_rxd1_lp_n_i,
  (* syn_peri_port = 0 *) input csi_rxd1_lp_p_i,
  (* syn_peri_port = 0 *) input [7:0] csi_rxd2_hs_i,
  (* syn_peri_port = 0 *) input csi_rxd2_lp_n_i,
  (* syn_peri_port = 0 *) input csi_rxd2_lp_p_i,
  (* syn_peri_port = 0 *) input [7:0] csi_rxd3_hs_i,
  (* syn_peri_port = 0 *) input csi_rxd3_lp_n_i,
  (* syn_peri_port = 0 *) input csi_rxd3_lp_p_i,
  (* syn_peri_port = 0 *) input dsi_txd0_lp_n_i,
  (* syn_peri_port = 0 *) input dsi_txd0_lp_p_i,
  (* syn_peri_port = 0 *) input dsi_txd1_lp_n_i,
  (* syn_peri_port = 0 *) input dsi_txd1_lp_p_i,
  (* syn_peri_port = 0 *) input dsi_txd2_lp_n_i,
  (* syn_peri_port = 0 *) input dsi_txd2_lp_p_i,
  (* syn_peri_port = 0 *) input dsi_txd3_lp_n_i,
  (* syn_peri_port = 0 *) input dsi_txd3_lp_p_i,
  (* syn_peri_port = 0 *) output cmos_ctl2,
  (* syn_peri_port = 0 *) output cmos_sclk,
  (* syn_peri_port = 0 *) output cmos_sdat_OUT,
  (* syn_peri_port = 0 *) output cmos_sdat_OE,
  (* syn_peri_port = 0 *) output [5:0] led_o,
  (* syn_peri_port = 0 *) output sys_pll_rstn_o,
  (* syn_peri_port = 0 *) output ddr_pll_rstn_o,
  (* syn_peri_port = 0 *) output [2:0] shift,
  (* syn_peri_port = 0 *) output shift_ena,
  (* syn_peri_port = 0 *) output [4:0] shift_sel,
  (* syn_peri_port = 0 *) output dsi_pll_rstn_o,
  (* syn_peri_port = 0 *) output lvds_pll_rstn_o,
  (* syn_peri_port = 0 *) output hdmi_txc_oe,
  (* syn_peri_port = 0 *) output [9:0] hdmi_txc_o,
  (* syn_peri_port = 0 *) output hdmi_txc_rst_o,
  (* syn_peri_port = 0 *) output hdmi_txd0_oe,
  (* syn_peri_port = 0 *) output [9:0] hdmi_txd0_o,
  (* syn_peri_port = 0 *) output hdmi_txd0_rst_o,
  (* syn_peri_port = 0 *) output hdmi_txd1_oe,
  (* syn_peri_port = 0 *) output [9:0] hdmi_txd1_o,
  (* syn_peri_port = 0 *) output hdmi_txd1_rst_o,
  (* syn_peri_port = 0 *) output hdmi_txd2_oe,
  (* syn_peri_port = 0 *) output [9:0] hdmi_txd2_o,
  (* syn_peri_port = 0 *) output hdmi_txd2_rst_o,
  (* syn_peri_port = 0 *) output lvds_txc_oe,
  (* syn_peri_port = 0 *) output [6:0] lvds_txc_o,
  (* syn_peri_port = 0 *) output lvds_txc_rst_o,
  (* syn_peri_port = 0 *) output lvds_txd0_oe,
  (* syn_peri_port = 0 *) output [6:0] lvds_txd0_o,
  (* syn_peri_port = 0 *) output lvds_txd0_rst_o,
  (* syn_peri_port = 0 *) output lvds_txd1_oe,
  (* syn_peri_port = 0 *) output [6:0] lvds_txd1_o,
  (* syn_peri_port = 0 *) output lvds_txd1_rst_o,
  (* syn_peri_port = 0 *) output lvds_txd2_oe,
  (* syn_peri_port = 0 *) output [6:0] lvds_txd2_o,
  (* syn_peri_port = 0 *) output lvds_txd2_rst_o,
  (* syn_peri_port = 0 *) output lvds_txd3_oe,
  (* syn_peri_port = 0 *) output [6:0] lvds_txd3_o,
  (* syn_peri_port = 0 *) output lvds_txd3_rst_o,
  (* syn_peri_port = 0 *) output [15:0] addr,
  (* syn_peri_port = 0 *) output [2:0] ba,
  (* syn_peri_port = 0 *) output cas,
  (* syn_peri_port = 0 *) output cke,
  (* syn_peri_port = 0 *) output clk_n_hi,
  (* syn_peri_port = 0 *) output clk_n_lo,
  (* syn_peri_port = 0 *) output clk_p_hi,
  (* syn_peri_port = 0 *) output clk_p_lo,
  (* syn_peri_port = 0 *) output cmos_ctl3,
  (* syn_peri_port = 0 *) output cs,
  (* syn_peri_port = 0 *) output csi_ctl0_o,
  (* syn_peri_port = 0 *) output csi_ctl0_oe,
  (* syn_peri_port = 0 *) output csi_ctl1_o,
  (* syn_peri_port = 0 *) output csi_ctl1_oe,
  (* syn_peri_port = 0 *) output csi_scl_o,
  (* syn_peri_port = 0 *) output csi_scl_oe,
  (* syn_peri_port = 0 *) output csi_sda_o,
  (* syn_peri_port = 0 *) output csi_sda_oe,
  (* syn_peri_port = 0 *) output [1:0] o_dm_hi,
  (* syn_peri_port = 0 *) output [1:0] o_dm_lo,
  (* syn_peri_port = 0 *) output [15:0] o_dq_hi,
  (* syn_peri_port = 0 *) output [15:0] o_dq_lo,
  (* syn_peri_port = 0 *) output [15:0] o_dq_oe,
  (* syn_peri_port = 0 *) output [1:0] o_dqs_hi,
  (* syn_peri_port = 0 *) output [1:0] o_dqs_lo,
  (* syn_peri_port = 0 *) output [1:0] o_dqs_oe,
  (* syn_peri_port = 0 *) output [1:0] o_dqs_n_hi,
  (* syn_peri_port = 0 *) output [1:0] o_dqs_n_lo,
  (* syn_peri_port = 0 *) output [1:0] o_dqs_n_oe,
  (* syn_peri_port = 0 *) output dsi_pwm_o,
  (* syn_peri_port = 0 *) output dsi_resetn_o,
  (* syn_peri_port = 0 *) output [7:0] lcd_b7_0_o,
  (* syn_peri_port = 0 *) output [7:0] lcd_b7_0_oe,
  (* syn_peri_port = 0 *) output lcd_blen_o,
  (* syn_peri_port = 0 *) output [7:0] lcd_g7_0_o,
  (* syn_peri_port = 0 *) output [7:0] lcd_g7_0_oe,
  (* syn_peri_port = 0 *) output lcd_hs_o,
  (* syn_peri_port = 0 *) output lcd_pwm_o,
  (* syn_peri_port = 0 *) output [7:0] lcd_r7_0_o,
  (* syn_peri_port = 0 *) output [7:0] lcd_r7_0_oe,
  (* syn_peri_port = 0 *) output lcd_tp_int_o,
  (* syn_peri_port = 0 *) output lcd_tp_int_oe,
  (* syn_peri_port = 0 *) output lcd_tp_rst_o,
  (* syn_peri_port = 0 *) output lcd_tp_scl_o,
  (* syn_peri_port = 0 *) output lcd_tp_scl_oe,
  (* syn_peri_port = 0 *) output lcd_tp_sda_o,
  (* syn_peri_port = 0 *) output lcd_tp_sda_oe,
  (* syn_peri_port = 0 *) output lcd_vs_o,
  (* syn_peri_port = 0 *) output odt,
  (* syn_peri_port = 0 *) output ras,
  (* syn_peri_port = 0 *) output reset,
  (* syn_peri_port = 0 *) output spi_sck_o,
  (* syn_peri_port = 0 *) output spi_ssn_o,
  (* syn_peri_port = 0 *) output uart_tx_o,
  (* syn_peri_port = 0 *) output we,
  (* syn_peri_port = 0 *) output csi_rxc_hs_en_o,
  (* syn_peri_port = 0 *) output csi_rxc_hs_term_en_o,
  (* syn_peri_port = 0 *) output csi_rxd0_hs_en_o,
  (* syn_peri_port = 0 *) output csi_rxd0_hs_term_en_o,
  (* syn_peri_port = 0 *) output csi_rxd0_rst_o,
  (* syn_peri_port = 0 *) output csi_rxd1_hs_en_o,
  (* syn_peri_port = 0 *) output csi_rxd1_hs_term_en_o,
  (* syn_peri_port = 0 *) output csi_rxd1_rst_o,
  (* syn_peri_port = 0 *) output csi_rxd2_hs_en_o,
  (* syn_peri_port = 0 *) output csi_rxd2_hs_term_en_o,
  (* syn_peri_port = 0 *) output csi_rxd2_rst_o,
  (* syn_peri_port = 0 *) output csi_rxd3_hs_en_o,
  (* syn_peri_port = 0 *) output csi_rxd3_hs_term_en_o,
  (* syn_peri_port = 0 *) output csi_rxd3_rst_o,
  (* syn_peri_port = 0 *) output dsi_txc_hs_oe,
  (* syn_peri_port = 0 *) output [7:0] dsi_txc_hs_o,
  (* syn_peri_port = 0 *) output dsi_txc_lp_n_oe,
  (* syn_peri_port = 0 *) output dsi_txc_lp_n_o,
  (* syn_peri_port = 0 *) output dsi_txc_lp_p_oe,
  (* syn_peri_port = 0 *) output dsi_txc_lp_p_o,
  (* syn_peri_port = 0 *) output dsi_txc_rst_o,
  (* syn_peri_port = 0 *) output dsi_txd0_hs_oe,
  (* syn_peri_port = 0 *) output [7:0] dsi_txd0_hs_o,
  (* syn_peri_port = 0 *) output dsi_txd0_lp_n_oe,
  (* syn_peri_port = 0 *) output dsi_txd0_lp_n_o,
  (* syn_peri_port = 0 *) output dsi_txd0_lp_p_oe,
  (* syn_peri_port = 0 *) output dsi_txd0_lp_p_o,
  (* syn_peri_port = 0 *) output dsi_txd0_rst_o,
  (* syn_peri_port = 0 *) output dsi_txd1_hs_oe,
  (* syn_peri_port = 0 *) output [7:0] dsi_txd1_hs_o,
  (* syn_peri_port = 0 *) output dsi_txd1_lp_n_oe,
  (* syn_peri_port = 0 *) output dsi_txd1_lp_n_o,
  (* syn_peri_port = 0 *) output dsi_txd1_lp_p_oe,
  (* syn_peri_port = 0 *) output dsi_txd1_lp_p_o,
  (* syn_peri_port = 0 *) output dsi_txd1_rst_o,
  (* syn_peri_port = 0 *) output dsi_txd2_hs_oe,
  (* syn_peri_port = 0 *) output [7:0] dsi_txd2_hs_o,
  (* syn_peri_port = 0 *) output dsi_txd2_lp_n_oe,
  (* syn_peri_port = 0 *) output dsi_txd2_lp_n_o,
  (* syn_peri_port = 0 *) output dsi_txd2_lp_p_oe,
  (* syn_peri_port = 0 *) output dsi_txd2_lp_p_o,
  (* syn_peri_port = 0 *) output dsi_txd2_rst_o,
  (* syn_peri_port = 0 *) output dsi_txd3_hs_oe,
  (* syn_peri_port = 0 *) output [7:0] dsi_txd3_hs_o,
  (* syn_peri_port = 0 *) output dsi_txd3_lp_n_oe,
  (* syn_peri_port = 0 *) output dsi_txd3_lp_n_o,
  (* syn_peri_port = 0 *) output dsi_txd3_lp_p_oe,
  (* syn_peri_port = 0 *) output dsi_txd3_lp_p_o,
  (* syn_peri_port = 0 *) output dsi_txd3_rst_o
);

  // Required PLL controls. Only system and DDR locks participate in reset.
  assign sys_pll_rstn_o  = 1'b1;
  assign ddr_pll_rstn_o  = sys_pll_lock;
  assign dsi_pll_rstn_o  = sys_pll_lock;
  assign lvds_pll_rstn_o = sys_pll_lock;

  // DDR differential clock pattern required by the board interface block.
  assign clk_p_hi = 1'b0;
  assign clk_p_lo = 1'b1;
  assign clk_n_hi = 1'b1;
  assign clk_n_lo = 1'b0;

  // HDMI lane enables and reset polarity match the original board design.
  assign hdmi_txc_oe = 1'b1;
  assign hdmi_txd0_oe = 1'b1;
  assign hdmi_txd1_oe = 1'b1;
  assign hdmi_txd2_oe = 1'b1;
  assign hdmi_txc_rst_o = ~(sys_pll_lock & ddr_pll_lock);
  assign hdmi_txd0_rst_o = ~(sys_pll_lock & ddr_pll_lock);
  assign hdmi_txd1_rst_o = ~(sys_pll_lock & ddr_pll_lock);
  assign hdmi_txd2_rst_o = ~(sys_pll_lock & ddr_pll_lock);

  MT9M001_HDMI_Core #(
    .BUILD_PROFILE_ID(BUILD_PROFILE_ID),.BUILD_PROFILE_FPS(BUILD_PROFILE_FPS),
    .PROFILE_RECONFIG_ENABLE(1),
    .SYSTEM_CLOCK_HZ(PROFILE_SYSTEM_CLOCK_HZ),.PIXEL_BITS(PROFILE_PIXEL_BITS),.SHAPE_TARGETS(PROFILE_SHAPE_TARGETS),
    .SENSOR_ROW_MIRROR(PROFILE_ROW_MIRROR),.SENSOR_COLUMN_MIRROR(PROFILE_COLUMN_MIRROR),.BAYER_PATTERN(PROFILE_BAYER_PATTERN),
    .R_GAIN_Q8(PROFILE_R_GAIN_Q8),.G_GAIN_Q8(PROFILE_G_GAIN_Q8),.B_GAIN_Q8(PROFILE_B_GAIN_Q8),
    .BLACK_LEVEL(PROFILE_BLACK_LEVEL),.GAMMA_ENABLE(PROFILE_GAMMA_ENABLE),
    .MEDIA_PHOTO_FRAMES(PROFILE_PHOTO_FRAMES),.MEDIA_VIDEO_FRAMES(PROFILE_VIDEO_FRAMES),.MEDIA_PRE_FRAMES(PROFILE_PRE_FRAMES),
    .SENSOR_WIDTH(BUILD_PROFILE_WIDTH),.SENSOR_HEIGHT(BUILD_PROFILE_HEIGHT),
    .DISPLAY_WIDTH(BUILD_PROFILE_WIDTH),.DISPLAY_HEIGHT(BUILD_PROFILE_HEIGHT),
    .DISPLAY_FPS(BUILD_PROFILE_FPS),.PIXEL_CLOCK_HZ(PROFILE_PIXEL_CLOCK_HZ),
    .SENSOR_ROW_START(PROFILE_ROW_START),.SENSOR_COLUMN_START(PROFILE_COLUMN_START),
    .SENSOR_HBLANK(PROFILE_HBLANK),.SENSOR_VBLANK(PROFILE_VBLANK),
    .EXPOSURE_LINES(PROFILE_EXPOSURE_LINES),
    .H_FRONT(PROFILE_H_FRONT),.H_SYNC(PROFILE_H_SYNC),.H_BACK(PROFILE_H_BACK),
    .V_FRONT(PROFILE_V_FRONT),.V_SYNC(PROFILE_V_SYNC),.V_BACK(PROFILE_V_BACK)
  ) video_core (
    .clk_sys(clk_sys), .clk_pixel(clk_pixel),
    .cfg_error(cfg_ERROR),.cfg_cbsel(cfg_CBSEL),.cfg_ena(cfg_ENA),.cfg_config(cfg_CONFIG),
    .k1_n(k1), .k2_n(k2),
    .sys_pll_lock(sys_pll_lock), .ddr_pll_lock(ddr_pll_lock),
    .core_clk(core_clk), .tac_clk(tac_clk), .twd_clk(twd_clk), .tdqss_clk(tdqss_clk),
    .cmos_pclk(cmos_pclk), .cmos_vsync(cmos_vsync), .cmos_href(cmos_href),
    .cmos_data(cmos_data), .cmos_sdat_i(cmos_sdat_IN),
    .cmos_sclk(cmos_sclk), .cmos_sdat_o(cmos_sdat_OUT), .cmos_sdat_oe(cmos_sdat_OE),
    .ddr_addr(addr), .ddr_ba(ba), .ddr_we(we), .ddr_reset(reset),
    .ddr_ras(ras), .ddr_cas(cas), .ddr_odt(odt), .ddr_cke(cke), .ddr_cs(cs),
    .ddr_dq_hi_i(i_dq_hi), .ddr_dq_lo_i(i_dq_lo),
    .ddr_dq_hi_o(o_dq_hi), .ddr_dq_lo_o(o_dq_lo), .ddr_dq_oe(o_dq_oe),
    .ddr_dm_hi_o(o_dm_hi), .ddr_dm_lo_o(o_dm_lo),
    .ddr_dqs_hi_i(i_dqs_hi), .ddr_dqs_lo_i(i_dqs_lo),
    .ddr_dqs_n_hi_i(i_dqs_n_hi), .ddr_dqs_n_lo_i(i_dqs_n_lo),
    .ddr_dqs_hi_o(o_dqs_hi), .ddr_dqs_lo_o(o_dqs_lo),
    .ddr_dqs_n_hi_o(o_dqs_n_hi), .ddr_dqs_n_lo_o(o_dqs_n_lo),
    .ddr_dqs_oe(o_dqs_oe), .ddr_dqs_n_oe(o_dqs_n_oe),
    .ddr_shift(shift), .ddr_shift_sel(shift_sel), .ddr_shift_ena(shift_ena),
    .hdmi_clock(hdmi_txc_o), .hdmi_data0(hdmi_txd0_o),
    .hdmi_data1(hdmi_txd1_o), .hdmi_data2(hdmi_txd2_o),
    .status_led(led_o),.status_uart_rx(uart_rx_i),.status_uart_tx(uart_tx_o)
  );

  // Unused demo interfaces are disabled at the board boundary.
  assign cmos_ctl2 = 1'b0;
  assign cmos_ctl3 = 1'b0;
  assign spi_sck_o = 1'b0;
  assign spi_ssn_o = 1'b1;

  assign csi_ctl0_o = 1'b0;
  assign csi_ctl0_oe = 1'b0;
  assign csi_ctl1_o = 1'b0;
  assign csi_ctl1_oe = 1'b0;
  assign csi_scl_o = 1'b0;
  assign csi_scl_oe = 1'b0;
  assign csi_sda_o = 1'b0;
  assign csi_sda_oe = 1'b0;
  assign csi_rxc_hs_en_o = 1'b0;
  assign csi_rxc_hs_term_en_o = 1'b0;
  assign csi_rxd0_hs_en_o = 1'b0;
  assign csi_rxd0_hs_term_en_o = 1'b0;
  assign csi_rxd0_rst_o = 1'b1;
  assign csi_rxd1_hs_en_o = 1'b0;
  assign csi_rxd1_hs_term_en_o = 1'b0;
  assign csi_rxd1_rst_o = 1'b1;
  assign csi_rxd2_hs_en_o = 1'b0;
  assign csi_rxd2_hs_term_en_o = 1'b0;
  assign csi_rxd2_rst_o = 1'b1;
  assign csi_rxd3_hs_en_o = 1'b0;
  assign csi_rxd3_hs_term_en_o = 1'b0;
  assign csi_rxd3_rst_o = 1'b1;

  assign dsi_pwm_o = 1'b0;
  assign dsi_resetn_o = 1'b0;
  assign dsi_txc_hs_oe = 1'b0;
  assign dsi_txc_hs_o = 8'd0;
  assign dsi_txc_lp_n_oe = 1'b0;
  assign dsi_txc_lp_n_o = 1'b0;
  assign dsi_txc_lp_p_oe = 1'b0;
  assign dsi_txc_lp_p_o = 1'b0;
  assign dsi_txc_rst_o = 1'b1;
  assign dsi_txd0_hs_oe = 1'b0;
  assign dsi_txd0_hs_o = 8'd0;
  assign dsi_txd0_lp_n_oe = 1'b0;
  assign dsi_txd0_lp_n_o = 1'b0;
  assign dsi_txd0_lp_p_oe = 1'b0;
  assign dsi_txd0_lp_p_o = 1'b0;
  assign dsi_txd0_rst_o = 1'b1;
  assign dsi_txd1_hs_oe = 1'b0;
  assign dsi_txd1_hs_o = 8'd0;
  assign dsi_txd1_lp_n_oe = 1'b0;
  assign dsi_txd1_lp_n_o = 1'b0;
  assign dsi_txd1_lp_p_oe = 1'b0;
  assign dsi_txd1_lp_p_o = 1'b0;
  assign dsi_txd1_rst_o = 1'b1;
  assign dsi_txd2_hs_oe = 1'b0;
  assign dsi_txd2_hs_o = 8'd0;
  assign dsi_txd2_lp_n_oe = 1'b0;
  assign dsi_txd2_lp_n_o = 1'b0;
  assign dsi_txd2_lp_p_oe = 1'b0;
  assign dsi_txd2_lp_p_o = 1'b0;
  assign dsi_txd2_rst_o = 1'b1;
  assign dsi_txd3_hs_oe = 1'b0;
  assign dsi_txd3_hs_o = 8'd0;
  assign dsi_txd3_lp_n_oe = 1'b0;
  assign dsi_txd3_lp_n_o = 1'b0;
  assign dsi_txd3_lp_p_oe = 1'b0;
  assign dsi_txd3_lp_p_o = 1'b0;
  assign dsi_txd3_rst_o = 1'b1;

  assign lvds_txc_oe = 1'b0;
  assign lvds_txc_o = 7'd0;
  assign lvds_txc_rst_o = 1'b1;
  assign lvds_txd0_oe = 1'b0;
  assign lvds_txd0_o = 7'd0;
  assign lvds_txd0_rst_o = 1'b1;
  assign lvds_txd1_oe = 1'b0;
  assign lvds_txd1_o = 7'd0;
  assign lvds_txd1_rst_o = 1'b1;
  assign lvds_txd2_oe = 1'b0;
  assign lvds_txd2_o = 7'd0;
  assign lvds_txd2_rst_o = 1'b1;
  assign lvds_txd3_oe = 1'b0;
  assign lvds_txd3_o = 7'd0;
  assign lvds_txd3_rst_o = 1'b1;

  assign lcd_b7_0_o = 8'd0;
  assign lcd_b7_0_oe = 8'd0;
  assign lcd_g7_0_o = 8'd0;
  assign lcd_g7_0_oe = 8'd0;
  assign lcd_r7_0_o = 8'd0;
  assign lcd_r7_0_oe = 8'd0;
  assign lcd_blen_o = 1'b0;
  assign lcd_hs_o = 1'b0;
  assign lcd_vs_o = 1'b0;
  assign lcd_pwm_o = 1'b0;
  assign lcd_tp_int_o = 1'b0;
  assign lcd_tp_int_oe = 1'b0;
  assign lcd_tp_rst_o = 1'b0;
  assign lcd_tp_scl_o = 1'b0;
  assign lcd_tp_scl_oe = 1'b0;
  assign lcd_tp_sda_o = 1'b0;
  assign lcd_tp_sda_oe = 1'b0;

endmodule

