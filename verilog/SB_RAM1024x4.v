

module SB_RAM1024x4( RDATA, RADDR, RCLK, RCLKE, RE,WADDR, WCLK, WCLKE, WE);
  	output [3:0] RDATA;
	input [9:0] RADDR;
	input RCLK;
	input RCLKE;
	input RE;
	input [9:0] WADDR;
	input WCLK;
	input WCLKE;
	input WE;
  	wire [11:0] _0_;
  	wire [11:0] _1_;

  SB_RAM40_4K ram40_4kinst_physical(
  .RDATA({_0_[11:10], RDATA[3], _0_[9:7], RDATA[2], _0_[6:4], RDATA[1], _0_[3:1], RDATA[0], _0_[0]}),
  .RADDR({1'b0, RADDR}),
  .WADDR({1'b0, WADDR}),
  .MASK(),
  .WDATA(16'b0010001000100010),
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
  parameter INIT_FILE = "";

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
  defparam ram40_4kinst_physical.INIT_FILE = INIT_FILE;
endmodule
