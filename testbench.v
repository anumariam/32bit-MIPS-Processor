`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   00:26:44 04/19/2010
// Design Name:   control_logic
// Module Name:   C:/Documents and Settings/shilpi/Desktop/embedded project/processor/testbench.v
// Project Name:  processor
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: control_logic
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module testbench;

	// Inputs
	reg clk;
	reg reset;

	// Outputs
	wire [31:0] aluout;
	wire [31:0] dataout;
	wire [31:0] signvalue;

	// Instantiate the Unit Under Test (UUT)
	control_logic uut (
		.clk(clk), 
		.reset(reset), 
		.aluout(aluout), 
		.dataout(dataout), 
		.signvalue(signvalue)
	);

initial
begin

clk = 0; reset = 1;
#40 reset = 0;

end

initial #100000000 $finish;


always #20 clk = ~clk ;

initial $monitor("Time = %g , dataout = %d , signvalue = %d Aluout =%d",$time,dataout,signvalue,aluout);
      
endmodule

