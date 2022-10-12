`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:51:00 11/06/2010
// Design Name:   mult
// Module Name:   A:/IIIT Hyderabad/Sem3/Modern Computer Architecture/PROJECT/simplemult.v/tb_mult.v
// Project Name:  simplemult.v
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: mult
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_mult;

	// Inputs
	reg [31:0] multiplier;
	reg [31:0] multiplicand;

	// Outputs
	wire [31:0] product1;
	wire [31:0] product2;

	// Instantiate the Unit Under Test (UUT)
	mult uut (
		.multiplier(multiplier), 
		.multiplicand(multiplicand), 
		.product1(product1), 
		.product2(product2)
	);

	initial begin
		// Initialize Inputs
		multiplier = 0;
		multiplicand = 0;

		// Wait 100 ns for global reset to finish
		#100;
        multiplier = 2147483648;
		multiplicand = 2;
		// Add stimulus here

	end
      initial $monitor("Time = %g , multiplier = %d , multiplicand = %d product =%d,product1=%d",$time,multiplier,multiplicand,{product2,product1},product2);
endmodule

