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
 *	Description:
 *
 *		This module implements an adder for use by the branch unit
 *		and program counter increment among other things.
 */



module DSPadder(input1, input2, out); // out = input1 - input2
	//Input and output should remain the same to perserve interface, need to change the add operation.
	input [31:0]	input1;
	input [31:0]	input2;
	output [31:0]	out;

	SB_MAC16 i_sbmac16 ( 		
		//Taken directly from SBTICE datasheet - defines connection to, from and inside DSP block
		.A(input1[31:16]),
		.B(input1[15:0]),
		.C(input2[31:16]),
		.D(input2[15:0]),
		.O(out),
		.CLK(), 							// Clock not connected - asynchronus operation for now
		.CE(0), 							// Clock disabled
		.IRSTTOP(0),					// Reset for input and output accumulators - not in use
		.IRSTBOT(0),	
		.ORSTTOP(0),
		.ORSTBOT(0),
		.AHOLD(0),						//Hold register values - don't care since registers are not in use for asyncronys operation
		.BHOLD(0),
		.CHOLD(0),
		.DHOLD(0),
		.OHOLDTOP(0),
		.OHOLDBOT(0),
		.OLOADTOP(0),
		.OLOADBOT(0),
		.ADDSUBTOP(1), 				// 0 for add, 1 for sub
		.ADDSUBBOT(1),				// --||--
		.CO(),         				// Carry output, could be used to indicate overflow in the future
		.CI(0),								// Carry input to lower adder, set to zero
		.ACCUMCI(0),					// Could be used for feedback? implementing other logic functions?
		.ACCUMCO(),						// Top accumulator output - not connected
		.SIGNEXTIN(0), 				// Sign extension - not used 
		.SIGNEXTOUT()					// Sign extension outut - not connected
		);
		
		// Set the instance parameters to define operation of DSP
		defparam i_sbmac16.B_SIGNED = 1'b0 ; 									// Don't care - only applies for mutliplication operation
		defparam i_sbmac16.A_SIGNED = 1'b0 ;									// --||--
		defparam i_sbmac16.MODE_8x8 = 1'b1 ; 									// Apparently reduced power, shouldn't affect operations since multipliers are bypassed in addition mode
		defparam i_sbmac16.BOTADDSUB_CARRYSELECT = 2'b00 ; 		// Constant zero on the carry input for lower adder
		defparam i_sbmac16.BOTADDSUB_UPPERINPUT = 1'b1 ;			// Connects D to Y (upper input)
		defparam i_sbmac16.BOTADDSUB_LOWERINPUT = 2'b00 ;  		// Connects B to Z (lower input)
		defparam i_sbmac16.BOTOUTPUT_SELECT = 2'b00 ;					// Connects output of adder directly to Lo output
		defparam i_sbmac16.TOPADDSUB_CARRYSELECT = 2'b11 ; 		// Connects carry out from lower adder to upper adder, (11 should work aswell but higher prop delay?)
		defparam i_sbmac16.TOPADDSUB_UPPERINPUT = 1'b1 ;  		// Connects C to W (upper input)
		defparam i_sbmac16.TOPADDSUB_LOWERINPUT = 2'b10 ; 		// Connects A to X (lower input)
		defparam i_sbmac16.TOPOUTPUT_SELECT = 2'b00 ;					// Connects output of adder directly to Hi output
		defparam i_sbmac16.PIPELINE_16x16_MULT_REG2 = 1'b0 ; 	// Dont care since multiplier not connected
		defparam i_sbmac16.PIPELINE_16x16_MULT_REG1 = 1'b1 ; 	// --||--
		defparam i_sbmac16.BOT_8x8_MULT_REG = 1'b1 ;					// --||--
		defparam i_sbmac16.TOP_8x8_MULT_REG = 1'b1 ;					// --||--
		defparam i_sbmac16.D_REG = 1'b0 ;											// Bypasses all input registers for asynchronus operation
		defparam i_sbmac16.B_REG = 1'b0 ;											// --||--
		defparam i_sbmac16.A_REG = 1'b0 ;											// --||--
		defparam i_sbmac16.C_REG = 1'b0 ;											// --||--

	//assign		out = input1 + input2; - Not needed anymore, handled by the DSP
endmodule
