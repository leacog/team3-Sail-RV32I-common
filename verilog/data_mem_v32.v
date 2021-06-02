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



//Data cache

module data_mem (clk, addr, write_data, memwrite, memread, sign_mask, read_data, led, clk_stall);
	input			clk;
	input [31:0]		addr;
	input [31:0]		write_data;
	input			memwrite;
	input			memread;
	input [3:0]		sign_mask;
	output [31:0]	read_data;
	output [7:0]		led;
	output reg		clk_stall;	//Sets the clock high

	/*
	 *	led register
	 */
	reg [31:0]		led_reg;

	/*
	 *	Current state
	 */
	integer			state = 0;

	/*
	 *	Possible states
	 */
	parameter		IDLE = 0;
	parameter		READ_BUFFER = 1;
	parameter		READ_WRITE_BUFFER = 2;
	parameter		READ = 3;
	parameter		WRITE = 4;

	/*
	 *	Line buffer
	 */
	reg [31:0]		word_buf_output;

	/*
	 *	Read buffer
	 */
	wire [31:0]		read_buf;

	/*
	 *	Buffer to identify read or write operation
	 */
	wire			memread_buf;
	wire		    memwrite_buf;

	/*
	 *	Buffers to store write data
	 */
	wire [31:0]		write_data_buffer;

	/*
	 *	Buffer to store address
	 */
	wire [31:0]		addr_buf;
	reg [1:0]		addr_byte_offset_buf_read; 

	/*
	 *	Sign_mask buffer
	 */
	wire [3:0]		sign_mask_buf_input;
	reg [3:0]		sign_mask_buf;

	/*
	 *	Block memory registers
	 *
	 *	(Bad practice: The constant for the size should be a `define).
	 */
	reg [31:0]		data_block[0:1023];

	/*
	 *	wire assignments
	 */
	wire [9:0]		addr_buf_block_addr;
	wire [1:0]		addr_buf_byte_offset;
	
	wire [31:0]		replacement_word;

	assign			addr_buf_block_addr	= addr_buf[11:2];
	assign			addr_buf_byte_offset = addr_buf[1:0]; 

	/*
	 *	Byte select decoder
	 */
	wire bdec_sig0;
	wire bdec_sig1;
	wire bdec_sig2;
	wire bdec_sig3;

	assign bdec_sig0 = (~addr_buf_byte_offset[1]) & (~addr_buf_byte_offset[0]);
	assign bdec_sig1 = (~addr_buf_byte_offset[1]) & (addr_buf_byte_offset[0]);
	assign bdec_sig2 = (addr_buf_byte_offset[1]) & (~addr_buf_byte_offset[0]);
	assign bdec_sig3 = (addr_buf_byte_offset[1]) & (addr_buf_byte_offset[0]);

	/*
	 *	Constructing the word to be replaced for write byte   
	 */

	wire[7:0] byte_r0;
	wire[7:0] byte_r1;
	wire[7:0] byte_r2;
	wire[7:0] byte_r3;
	wire[3:0] mask_byte;
	assign byte_r0 = (bdec_sig0==1'b1)? write_data[7:0]:8'b00;
	assign byte_r1 = (bdec_sig1==1'b1)? write_data[7:0]:8'b00;
	assign byte_r2 = (bdec_sig2==1'b1)? write_data[7:0]:8'b00;
	assign byte_r3 = (bdec_sig3==1'b1)? write_data[7:0]:8'b00;

	assign mask_byte[0] = (bdec_sig0==1'b1)? 1'b1 :1'b0;
	assign mask_byte[1] = (bdec_sig1==1'b1)? 1'b1 :1'b0;
	assign mask_byte[2] = (bdec_sig2==1'b1)? 1'b1 :1'b0;
	assign mask_byte[3] = (bdec_sig3==1'b1)? 1'b1 :1'b0;


	//For write halfword
	wire[15:0] halfword_r0;
	wire[15:0] halfword_r1;
	wire[3:0] mask_half;
	assign halfword_r0 = (addr_buf_byte_offset[1]==1'b1)?{8'b00, 8'b00}:write_data[15:0];
	assign halfword_r1 = (addr_buf_byte_offset[1]==1'b1)? write_data[15:0]:{8'b00, 8'b00}; 
	assign mask_half[1:0]= (addr_buf_byte_offset[1]==1'b1)?{2'b00}:{2'b11};
	assign mask_half[3:2]= (addr_buf_byte_offset[1]==1'b1)?{2'b11}:{2'b00}; 
	/*
	 *	Combinational logic for generating 32-bit read data
	 */

	wire [7:0]		rbuf0;
	wire [7:0]		rbuf1;
	wire [7:0]		rbuf2;
	wire [7:0]		rbuf3;

	assign 			rbuf0	= word_buf_output[7:0];
	assign 			rbuf1	= word_buf_output[15:8];
	assign 			rbuf2	= word_buf_output[23:16];
	assign 			rbuf3	= word_buf_output[31:24];
	
	wire select0;
	wire select1;
	wire select2;
	
	wire[31:0] out1;
	wire[31:0] out2;
	wire[31:0] out3;
	wire[31:0] out4;
	wire[31:0] out5; 
	wire[31:0] out6;
	/* a is sign_mask_buf[2], b is sign_mask_buf[1], c is sign_mask_buf[0]
	 * d is addr_buf_byte_offset[1], e is addr_buf_byte_offset[0]
	 */
	
	assign select0 = (~sign_mask_buf[2] & ~sign_mask_buf[1] & ~addr_byte_offset_buf_read[1] & addr_byte_offset_buf_read[0]) | (~sign_mask_buf[2] & addr_byte_offset_buf_read[1] & addr_byte_offset_buf_read[0]) | (~sign_mask_buf[2] & sign_mask_buf[1] & addr_byte_offset_buf_read[1]); //~a~b~de + ~ade + ~abd
	assign select1 = (~sign_mask_buf[2] & ~sign_mask_buf[1] & addr_byte_offset_buf_read[1]) | (sign_mask_buf[2] & sign_mask_buf[1]); // ~a~bd + ab
	assign select2 = sign_mask_buf[1]; //b
	
	assign out1 = (select0) ? ((sign_mask_buf[3]==1'b1) ? {{24{rbuf1[7]}}, rbuf1} : {24'b0, rbuf1}) : ((sign_mask_buf[3]==1'b1) ? {{24{rbuf0[7]}}, rbuf0} : {24'b0, rbuf0});
	assign out2 = (select0) ? ((sign_mask_buf[3]==1'b1) ? {{24{rbuf3[7]}}, rbuf3} : {24'b0, rbuf3}) : ((sign_mask_buf[3]==1'b1) ? {{24{rbuf2[7]}}, rbuf2} : {24'b0, rbuf2}); 
	assign out3 = (select0) ? ((sign_mask_buf[3]==1'b1) ? {{16{rbuf3[7]}}, rbuf3, rbuf2} : {16'b0, rbuf3, rbuf2}) : ((sign_mask_buf[3]==1'b1) ? {{16{rbuf1[7]}}, rbuf1, rbuf0} : {16'b0, rbuf1, rbuf0});
	assign out4 = (select0) ? 32'b0 : {rbuf3, rbuf2, rbuf1, rbuf0};
	
	assign out5 = (select1) ? out2 : out1;
	assign out6 = (select1) ? out4 : out3;
	
	assign read_data = (select2) ? out6 : out5; 
	
	/*
	 *	This uses Yosys's support for nonzero initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design.
	 */
	initial begin
		$readmemh("verilog/data.hex", data_block);
		clk_stall = 0;
	end

	/*
	 *	LED register interfacing with I/O
	 */
	always @(posedge clk) begin
		if(memwrite == 1'b1 && addr == 32'h2000) begin
			led_reg <= write_data;
		end
	end

	/*
	 *	State machine
	 */

	reg[31:0]datain;
	reg[3:0] br_mask;
	

	always @(*)begin
		case (sign_mask[2:0])
					3'b001: begin //byte
						datain <= {byte_r3, byte_r2, byte_r1, byte_r0};
						br_mask <= mask_byte;
					end
					3'b011 : begin //halfword
						datain <= {halfword_r1, halfword_r0};
						br_mask <= mask_half;
					end
					3'b111: begin //word
						datain <= write_data;
						br_mask <= 4'b1111;
					end
					default: begin
						//do nothing
					end
				endcase
	end

	assign sign_mask_buf_input = sign_mask;
	assign write_data_buffer = write_data;
	assign memread_buf = memread;
	assign memwrite_buf = memwrite;
	assign addr_buf = addr;



	
	always @(posedge clk) begin
		if(memread_buf==1'b1)begin
			addr_byte_offset_buf_read <= addr_buf[1:0];
			sign_mask_buf <= sign_mask;
			word_buf_output <= data_block[addr_buf_block_addr - 32'h1000 ];
		end
		else if(memwrite_buf==1'b1) begin
			if (br_mask[3]) begin data_block[addr_buf_block_addr - 32'h1000][31:24] <= datain[31:24];end
			if (br_mask[2]) begin data_block[addr_buf_block_addr - 32'h1000][23:16] <= datain[23:16];end
			if (br_mask[1]) begin data_block[addr_buf_block_addr - 32'h1000][15:8] <= datain[15:8];end
			if (br_mask[0]) begin data_block[addr_buf_block_addr - 32'h1000][7:0] <= datain[7:0];end
		end	
	end

	/*
	 *	Test led
	 */
	assign led = led_reg[7:0];
endmodule
