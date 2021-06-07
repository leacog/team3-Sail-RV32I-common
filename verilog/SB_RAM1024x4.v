

module SB_RAM1024x4( RDATA, RADDR, RCLK, RCLKE, RE,WADDR, WCLK, WCLKE, WDATA, WE);
  output [3:0] RDATA;
	input [9:0] RADDR;
	input RCLK;
	input RCLKE;
	input RE;
	input [9:0] WADDR;
	input WCLK;
	input WCLKE;
	output [3:0] WDATA;
	input WE;
  wire _0_ [15:0];
  wire _W_ [15:0];

  SB_RAM40_4K ram40_4kinst_physical(
  .RDATA({ _0_[15:14], RDATA[3], _0_[12:10], RDATA[2], _0_[8:6], RDATA[1], _0_[4:2], RDATA[0], _0_[0]}),
  .RADDR({1b'0, RADDR}),
  .WADDR({1b'0, WADDR}),
  .MASK(),
  .WDATA({ _W_[15:14], WDATA[3], _W_[12:10], WDATA[2], _W_[8:6], WDATA[1], _W_[4:2], WDATA[0], _W_[0]})
  .RCLKE(RCLKE),
  .RCLK(RCLK),
  .RE(RE),
  .WCLKE(WCLKE),
  .WCLK(WCLK),
  .WE(WE)
  );
  defparam ram40_4kinst_physical.READ_MODE=2;
  defparam ram40_4kinst_physical.WRITE_MODE=2;

  parameter INIT_0 = 256'h0;
  parameter INIT_1 = 256'h0;
  parameter INIT_2 = 256'h0;
  parameter INIT_3 = 256'h0;
  parameter INIT_4 = 256'h0;
  parameter INIT_5 = 256'h0;
  parameter INIT_6 = 256'h0;
  parameter INIT_7 = 256'h0;
  parameter INIT_8 = 256'h0;
  parameter INIT_9 = 256'h0;
  parameter INIT_A = 256'h0;
  parameter INIT_B = 256'h0;
  parameter INIT_C = 256'h0;
  parameter INIT_D = 256'h0;
  parameter INIT_E = 256'h0;
  parameter INIT_F = 256'h0;

  defparam ram40_4kinst_physical.INIT_0 = INIT_0;
  defparam ram40_4kinst_physical.INIT_1 = INIT_1;
  defparam ram40_4kinst_physical.INIT_2 = INIT_2;
  defparam ram40_4kinst_physical.INIT_3 = INIT_3;
  defparam ram40_4kinst_physical.INIT_4 = INIT_4;
  defparam ram40_4kinst_physical.INIT_5 = INIT_5;
  defparam ram40_4kinst_physical.INIT_6 = INIT_6;
  defparam ram40_4kinst_physical.INIT_7 = INIT_7;
  defparam ram40_4kinst_physical.INIT_8 = INIT_8;
  defparam ram40_4kinst_physical.INIT_9 = INIT_9;
  defparam ram40_4kinst_physical.INIT_A = INIT_A;
  defparam ram40_4kinst_physical.INIT_B = INIT_B;
  defparam ram40_4kinst_physical.INIT_C = INIT_C;
  defparam ram40_4kinst_physical.INIT_D = INIT_D;
  defparam ram40_4kinst_physical.INIT_E = INIT_E;
  defparam ram40_4kinst_physical.INIT_F = INIT_F;

);