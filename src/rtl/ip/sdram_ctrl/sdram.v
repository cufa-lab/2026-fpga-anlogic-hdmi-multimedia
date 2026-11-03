`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: anlgoic
// Author: 	xg 
// description:  top module
//////////////////////////////////////////////////////////////////////////////////

`define DEBUG

`include "../src/rtl/ip/sdram_ctrl/global_def.v"

module sdram(
    `ifdef SIMULATION
	output					SDRAM_CLK,
	output					SDR_CKE,
	output					SDR_RAS,
	output					SDR_CAS,
	output					SDR_WE,
	output	[`BA_WIDTH-1:0]	SDR_BA,
	output	[`ROW_WIDTH-1:0]SDR_ADDR,
	output	[`DM_WIDTH-1:0]	SDR_DM,
	inout	[`DATA_WIDTH-1:0]	SDR_DQ,	
	`endif
    input   					Clk,
    input						Clk_sft,
    input						Rst_n,          
    
    output 						Sdr_init_done,
    output						Sdr_init_ref_vld,
    output						Sdr_busy,
    input						App_ref_req,   

    input						App_wr_en,
	input [`ADDR_WIDTH-1:0]		App_wr_addr,  	
	input [`DM_WIDTH-1:0]		App_wr_dm,
	input [`DATA_WIDTH-1:0]		App_wr_din,	//synthesis keep
    
    input 						App_rd_en,
	input [`ADDR_WIDTH-1:0]		App_rd_addr,
	output 						Sdr_rd_en,
	output [`DATA_WIDTH-1:0]	Sdr_rd_dout	//synthesis keep
    
    
    
);
`ifndef SIMULATION
wire  			SDRAM_CLK;
wire 			SDR_RAS;
wire  			SDR_CAS;
wire  			SDR_WE;
wire  [`BA_WIDTH-1:0]	SDR_BA; 
wire  [`ROW_WIDTH-1:0]	SDR_ADDR;
wire  [`DATA_WIDTH-1:0]	SDR_DQ ;
wire  [`DM_WIDTH-1:0]	SDR_DM; 
`endif

wire SDR_RAS_raw;
wire SDR_CAS_raw;
wire SDR_WE_raw;

// 官方控制器在少数仲裁边界会把 AUTO REFRESH 命令保持两个 40MHz 周期，
// 使物理 SDRAM 看到间隔仅 25ns 的两个 REF，短于典型 tRFC=60ns。控制器内部
// 的自动刷新定时仍是唯一刷新源；这里只保留首个 REF，并在随后 3 个 Clk 周期
// 内把重复 REF 改成 NOP。地址、数据和其它命令不增加流水级。
reg [1:0] ref_guard;
wire raw_auto_refresh = !SDR_RAS_raw && !SDR_CAS_raw && SDR_WE_raw;
wire suppress_repeat_refresh = raw_auto_refresh && (ref_guard != 2'd0);

always @(posedge Clk or negedge Rst_n) begin
	if (!Rst_n)
		ref_guard <= 2'd0;
	else if (raw_auto_refresh && ref_guard == 2'd0)
		ref_guard <= 2'd3;
	else if (ref_guard != 2'd0)
		ref_guard <= ref_guard - 1'b1;
end

assign SDR_RAS = suppress_repeat_refresh ? 1'b1 : SDR_RAS_raw;
assign SDR_CAS = suppress_repeat_refresh ? 1'b1 : SDR_CAS_raw;
assign SDR_WE  = suppress_repeat_refresh ? 1'b1 : SDR_WE_raw;


sdr_as_ram  #( .self_refresh_open(1'b0))
	u2_ram( 
		.Sdr_clk(Clk),
		.Sdr_clk_sft(Clk_sft),
		.Rst(~Rst_n),
			  			  
		.Sdr_init_done(Sdr_init_done),
		.Sdr_init_ref_vld(Sdr_init_ref_vld),
		.Sdr_busy(Sdr_busy),
		
		.App_ref_req(App_ref_req),
		
        .App_wr_en(App_wr_en), 
        .App_wr_addr(App_wr_addr),  	
		.App_wr_dm(App_wr_dm),
		.App_wr_din(App_wr_din),

		.App_rd_en(App_rd_en),
		.App_rd_addr(App_rd_addr),
		.Sdr_rd_en	(Sdr_rd_en),
		.Sdr_rd_dout(Sdr_rd_dout),
	
		.SDRAM_CLK(SDRAM_CLK),
		.SDR_RAS(SDR_RAS_raw),
		.SDR_CAS(SDR_CAS_raw),
		.SDR_WE(SDR_WE_raw),
		.SDR_BA(SDR_BA),
		.SDR_ADDR(SDR_ADDR),
		.SDR_DM(SDR_DM),
		.SDR_DQ(SDR_DQ)	
	);


assign SDR_CKE=1'b1;

//`ifndef SIMULATION
	EG_PHY_SDRAM_2M_32 sdram(
		.clk(SDRAM_CLK),
		.ras_n(SDR_RAS),
		.cas_n(SDR_CAS),
		.we_n(SDR_WE),
		.addr(SDR_ADDR[10:0]),
		.ba(SDR_BA),
		.dq(SDR_DQ),
		.cs_n(1'b0),
		.dm0(SDR_DM[0]),
		.dm1(SDR_DM[1]),
		.dm2(SDR_DM[2]),
		.dm3(SDR_DM[3]),
		.cke(1'b1)
		);
//`endif

endmodule 
