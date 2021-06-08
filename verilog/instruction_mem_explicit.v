/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



/*
 *	RISC-V instruction memory
 */

`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mods_to_use.v"

`ifdef USE_INSTRUCTION_MEM_BRAM

	module instruction_memory_bram(addr, out, clk);
		input clk;
		input [31:0] addr;
		output [31:0] out;
		
		
		SB_RAM1024x4 mem_cell_1 (
			.RDATA(out[3:0]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);
		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_1_params.v"
		defparam mem_cell_1.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program1.hex";
	
		SB_RAM1024x4 mem_cell_2 (
			.RDATA(out[7:4]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_2_params.v"
		defparam mem_cell_2.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program2.hex";

		SB_RAM1024x4 mem_cell_3 (
			.RDATA(out[11:8]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_3_params.v"
		defparam mem_cell_3.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program3.hex";

		SB_RAM1024x4 mem_cell_4 (
			.RDATA(out[15:12]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_4_params.v"
		defparam mem_cell_4.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program4.hex";

		SB_RAM1024x4 mem_cell_5 (
			.RDATA(out[19:16]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_5_params.v"
		defparam mem_cell_5.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program5.hex";

		SB_RAM1024x4 mem_cell_6 (
			.RDATA(out[23:20]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_6_params.v"
		defparam mem_cell_6.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program6.hex";

		SB_RAM1024x4 mem_cell_7 (
			.RDATA(out[27:24]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_7_params.v"
		defparam mem_cell_7.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program7.hex";

		SB_RAM1024x4 mem_cell_8 (
			.RDATA(out[31:28]),
			.RADDR(addr[11:2]),
			.RCLK(clk),
			.RCLKE(1'b1),
			.RE(1'b1),
			.WADDR(10'b0),
			.WCLK(1'b0),
			.WCLKE(1'b0),
			.WE(1'b0)
		);

		`include "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_8_params.v"
		defparam mem_cell_8.INIT_FILE = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program8.hex";

	endmodule

`else

	module instruction_memory(addr, out);
		input [31:0]		addr;
		output [31:0]		out;

		reg [31:0]		instruction_memory[0:1023];

		initial begin
			$readmemh("/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/programs/program.hex",instruction_memory);
		end

		assign out = instruction_memory[addr >> 2];
	endmodule

`endif
