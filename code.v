`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    00:21:54 04/19/2010 
// Design Name: 
// Module Name:    code 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define Reg0 5'd0
`define Reg1 5'd1
`define Reg2 5'd2
`define Reg3 5'd3
`define Reg4 5'd4
`define Reg5 5'd5
`define Reg6 5'd6
`define Reg7 5'd7
`define Reg8 5'd8
`define Reg9 5'd9
`define Reg10 5'd10
`define Reg11 5'd11
`define Reg12 5'd12
`define Reg13 5'd13
`define Reg14 5'd14
`define Reg15 5'd15
`define Reg16 5'd16
`define Reg17 5'd17
`define Reg18 5'd18
`define Reg19 5'd19
`define Reg20 5'd20
`define Reg21 5'd21
`define Reg22 5'd22
`define Reg23 5'd23
`define Reg24 5'd24
`define Reg25 5'd25
`define Reg26 5'd26
`define Reg27 5'd27
`define Reg28 5'd28
`define Reg29 5'd29


module mux2_1(in1,in2,out,sel);

input [4:0]in1,in2;
input sel;
output  [4:0]out;

assign out = (sel == 1 ) ? in2 : in1 ;

endmodule

module mux2_1_alu(in1,in2,out,sel);

input [31:0]in1,in2;
input sel;
output  [31:0]out;

assign out = (sel == 1 ) ? in2 : in1 ;

endmodule


module mux4_2(ina1,ina2,ina3,ina4,outa,sela);

input [31:0]ina1,ina2,ina3,ina4;

input [1:0]sela;

output reg [31:0]outa;

always @ ( ina1 or ina2 or ina3 or ina4 or sela )
begin

case(sela)

 0: outa = ina1;

 1: outa = ina2;

 2: outa = ina3;

 3: outa = ina4;

endcase

end

endmodule

module mux4_2_s(ina1,ina2,ina3,ina4,outa,sela);

input [4:0]ina1,ina2,ina3,ina4;

input [1:0]sela;

output reg [4:0]outa;

always @ ( ina1 or ina2 or ina3 or ina4 or sela )
begin

case(sela)

0: outa = ina1;

1: outa = ina2;

2: outa = ina3;

3: outa = ina4;

endcase

end

endmodule


module program_counter(pc_out,pc_write,pc_in);

input [31:0]pc_in;
output reg [31:0]pc_out;

input pc_write;


always @ ( pc_write or pc_in )
begin

 if(pc_write==1)
 begin
 pc_out <= pc_in;
 end

end

endmodule

module instruction_memory (in_read,address,data);

input in_read;

input [4:0] address;

output reg [31:0] data;

always @ ( address or in_read )
	begin

	if ( in_read == 1)
	begin

	case (address)

	0: data = 32'd1;

4: data = { 6'd0, `Reg1, `Reg0, `Reg3, 5'd0, 6'd4}; 	//AND

8: data = { 6'd0, `Reg3, `Reg0,`Reg2,5'd0,6'd5}; 		//OR

12: data = { 6'd0, `Reg3, `Reg0, `Reg5, 5'd0, 6'd1}; //ADD

16: data = { 6'd13, `Reg5, `Reg5,16'd1 };		//reg5=reg5 then branch to 1(BEQ)

20: data = { 6'd0, `Reg5, `Reg2,`Reg4 ,5'd0,6'd4};


24: data = { 6'd8, `Reg4, `Reg7, 16'd5};

28: data = { 6'd1, `Reg3, `Reg1, `Reg5, 5'd0, 6'd1 };

default : data = 0;

endcase
//$display ("address:%d",address");
end

end

endmodule


module DATA_MEM(memwrite,memread,in_data,out_data,addr_in,enable,clk);

input memwrite,memread, enable , clk ;
input [31:0] in_data;

output [31:0] out_data;
reg [31:0] out_data;
input [31:0] addr_in;
reg [7:0] memory[64:0];

always@ ( memread or enable or in_data)
begin

if( memread==1 && enable ==1)
begin
out_data[7:0]   = memory[addr_in + 0];
out_data[15:8]  = memory[addr_in + 1];
out_data[23:16] = memory[addr_in + 2];
out_data[31:24] = memory[addr_in + 3];

end
end

always @ ( posedge clk)
begin

if( memwrite==1 && enable==1)
begin
        memory[addr_in + 0] <= in_data[7:0];
        memory[addr_in + 1] <= in_data[15:8];
        memory[addr_in + 2] <= in_data[23:16];
        memory[addr_in + 3] <= in_data[31:24];

end

end

endmodule


module sign_extend(IN,OUT);
input [15:0] IN;
output [31:0] OUT;

reg [31:0] Temp_out;

always @(IN)
	begin

if(IN[15]==0)
	begin
	Temp_out[15:0]=IN[15:0];
	Temp_out[31:16]=16'b0;
	end

else if(IN[15]==1)
	begin
	Temp_out[15:0]=IN[15:0];
	Temp_out[31:16]=16'b0;

	Temp_out=~Temp_out;
	Temp_out=Temp_out+32'd1;
	end
	end

	assign OUT=Temp_out;

endmodule


module register(reg1,reg2,wreg,wd,clk,read1,read2,regwrite);

input [4:0]reg1,reg2,wreg;

input [31:0]wd;

input regwrite,clk;

output reg [31:0]read1,read2;

reg [31:0]registers[31:0];

always @ ( reg1 or reg2 or regwrite )
        begin


if(regwrite==0)
        begin

        read1 = registers[reg1];
        read2 = registers[reg2];

        end



        end

always @ ( posedge clk )
        begin

        registers[0] <= 5; registers[1] <= 4;registers[9]<=0;
		   //  registers[3] <= 32'd4; registers[4] <= 32'd5;

        if(regwrite==1)
        begin

        registers[wreg] <= wd;

        end

        end

endmodule



module ALU(In_1,In_2,Out,Cy,zero,INST,Reset,alu_signal,clk);

input [31:0] In_1,In_2,INST;
input Reset,alu_signal,clk;

output reg [31:0] Out;
output reg Cy,zero;

reg [31:0] Y1,Y2;

always @( posedge clk )
	begin	
	
   if(Reset==0)
	begin
	Out <= 32'd0;
	Cy <= 1'b0;
	zero <= 1'b0;
	end

 if (alu_signal == 1)
  begin

   if(INST[31:26]==6'b0)
	begin
   case(INST[5:0])


	6'd1 : begin
   {Cy,Out} <= In_1 + In_2 ;
end

	6'd2 :begin
if(In_1 >= In_2)
	begin
	Y2 <= ~In_2;
	Y2 <= Y2+1;
{Cy,Out} <= In_1 + Y2 ;
end

else 
begin
Y2 <= ~In_2;
{Cy,Out} <= In_1 + Y2 + 32'b1;
Out <= ~Out;
Out <= Out + 32'b1;
end
end

6'd4 :begin
Out <= In_1 & In_2;
end

6'd5 :begin
Out <= In_1 | In_2;
end

6'd6 :begin
Out <= In_1 | In_2;
Out <= ~Out;
end

6'd7 :begin
Out <= In_1 & In_2;
Out <= ~Out;
end

6'd8 :begin
Out <= In_1 ^ In_2;
end

	6'd10 :begin
if(In_1<In_2)
	Out <= 32'b1;
	else
	Out <= 32'b0;
	end
	endcase	

	end

	else 
	begin
case(INST[31:26])


	6'd1 : begin
{Cy,Out} <= In_1 + In_2 ;
end

        6'd11 : begin

{Cy, Out } <= In_1 + In_2 ;

end

   6'd12 : begin

{Cy, Out } <= In_1 + In_2 ;

end

   6'd13 : begin
Out <=In_1-In_2;

if (Out == 0 )
begin
zero <= 1'b1;
//$display("ze=%d",zero);
end
else
zero <= 0;
//$display("zero_alu=%d , In_1=%d,In_2=%d,Out=%d",zero,In_1,In_2,Out);
end

6'd14 : begin
Out <= In_1-In_2;


if (Out == 0 )
zero <= 1;

else
zero <= 0;

end

6'd15 : begin
Out<=In_1-In_2;



if (Out == 0 )
zero <= 1;

else 
zero <= 0;

end

6'd16 : begin
if(In_1 >= In_2)
	begin   
	Y2 <= ~In_2;
{Cy,Out} <= In_1 + Y2 +32'b1;
end

else
begin
Y2 <= ~In_2;
{Cy,Out} <= In_1 + Y2 + 32'b1;
Out <= ~Out;
Out <=  Out + 32'b1;
end


if (Out == 0 )
zero <= 1;

else zero <= 0;

end




	6'd2 :begin
if(In_1 >= In_2)
	begin
	Y2 <= ~In_2;
{Cy,Out} <= In_1 + Y2 +32'b1;
end

else 
begin
Y2 <= ~In_2;
{Cy,Out} <=In_1 + Y2 + 32'b1;
Out <= ~Out;
Out <= Out + 32'b1;
end
end

6'd4 :begin
Out <= In_1 & In_2;
end

6'd5 :begin
Out <= In_1 | In_2;
end

6'd6 :begin
Out <= In_1 | In_2;
Out <= ~Out;
end

6'd7 :begin
Out <= In_1 & In_2;
Out <= ~Out;
end

6'd8 :begin
Out <= In_1 ^ In_2;
end

	6'd10 :begin
if(In_1<In_2)
	Out <= 32'b1;
	else
	Out <= 32'b0;
	end

	6'd17 :begin
	Out <= In_1<<In_2;
	end

	6'd18 :begin
	Out <= In_1>>In_2;
	end
endcase
	
	
end



end



	
end

endmodule

module proccessor_datapath(dataouti,signvalue,Aluout,ze,clk,pc_write,in_read,pc_in,regwrite,alu_reset,regdst,alusrc,alu_signal,mem_enable,memwrite,memread,mem2reg);

input pc_write,in_read,regwrite,alu_reset,alusrc,alu_signal,mem_enable,memwrite,memread,clk;

input [1:0]mem2reg,regdst;

input [31:0]pc_in;

output reg [31:0]dataouti,signvalue;

output reg ze;

wire [31:0]pc_out,data,wd,read1,read2,imm_value,readval2,Out,out_data,regw;

output reg [31:0]Aluout;

wire [4:0]wriadd;

wire Cy;

reg [4:0]next;

program_counter ins1(pc_out,pc_write,pc_in);

instruction_memory ins2(in_read,pc_out,data);

mux4_2_s ins22(data[20:16],data[15:11],next,,wriadd,regdst);

sign_extend ins23(data[15:0],imm_value);

register ins3(data[25:21],data[20:16],wriadd,regw,clk,read1,read2,regwrite);

mux2_1_alu ins32(read2,imm_value,readval2,alusrc);

ALU ins4(read1,readval2,Out,Cy,zero,data,alu_reset,alu_signal,clk);

//booth_multiplier ins44(read1,read2,mult_reset,mult_cs,clk,Mulout1,Mulout2);

DATA_MEM ins5(memwrite,memread,read2,out_data,Out,mem_enable,clk);

mux4_2 ins6(Out,out_data,,,regw,mem2reg);

always @ ( data or imm_value or zero or Out)
begin

dataouti = data ;
signvalue = imm_value;
ze = zero;
//$display("ze*=%d",ze);
Aluout = Out;
next = data[20:16] + 1 ;

end
	
initial $monitor("Time = %g, pc_in = %d , pc_out = %d , data = %d , pc_write = %b , in_read = %b ,read1 = %d , read2 = %d , actual read2 = %d , immediatevalue = %d, reg_write = %b , Alu_out = %d , Cy = %d funct = %d , aluop = %d , write reg = %d write data = %d ze= %d, Registerchoose = %d zero=%d\n",$time,pc_in,pc_out,data,pc_write,in_read,read1,read2,readval2, imm_value,regwrite,Out,Cy,data[5:0],data[31:26],wriadd,regw,ze,mem2reg,zero);

endmodule

module control_logic(clk,reset,aluout,dataout,signvalue);

input clk,reset;

output [31:0]dataout,signvalue;

wire zero;

output [31:0]aluout;

reg pc_write,in_read,reg_write,alu_reset,alusrc,alu_signal,mem_enable,memwrite,memread,mult_reset,mult_cs;
reg [1:0]mem2reg,regdst;

reg [31:0]pc_in,dataouti;

reg [3:0]state;

reg branch;

reg [31:0]pc_new;

parameter instruction_fetch=0 , instruction_decode = 1 , execute = 2 , datamem = 3 ,writeback = 4 , null_state = 6 , multiplystal1 = 7 , multiplystal2 = 8 , multiplystal3 = 9 , writebackmul1 = 10 , writebackmul2 = 11, finalwritemul = 12 ;

always @ ( posedge clk )
begin


   if ( reset == 1 )
    begin

    pc_in <= 0;

    dataouti <= 0 ;

    pc_write <= 1;

    in_read <= 1;

    reg_write <= 0;

    regdst <= 1;

    alu_reset <= 1 ;

    alusrc <= 0;

    alu_signal <= 0;

    memread <= 0;

    memwrite <= 0;

    mem_enable <= 0;

    mem2reg <= 0 ; 

    branch <= 0;

    mult_reset <= 1;

    state <= instruction_fetch;

    end

   else
   begin

   case(state)

   instruction_fetch:
   begin

   if(branch == 1)
	pc_in <= pc_new;
   
   else	
   pc_in <= pc_in + 4;

    pc_write <= 1;

    in_read <= 0;

    alu_signal <= 0;

    reg_write <= 0;

    memwrite <= 0;

  //  mult_reset <= 1;

    state <= instruction_decode;
  //$display("pc_in=%d",pc_in);
   end


   instruction_decode:
   begin

   in_read <=1;

   reg_write <=0;

   alu_reset <= 1;

   branch <= 0;
   state <= execute;


   end

   execute:
   begin


   if(dataout[31:26]>0 && dataout[31:26] <= 12)
	begin
	regdst <= 0;
	alusrc <= 1;
	end
   
   else if(dataout[31:26]>=13 && dataout[31:26] <= 16 )
	
      begin
    regdst <= 0;
    alusrc <= 0;
	  alu_signal <= 1;
	 // state <= null;
      end

    else
	begin
	regdst <= 1;
	alusrc <= 0;
	 alu_signal <= 1;
	end

     
       
      
    if(dataout[31:26] == 11 || dataout[31:26] == 12)
	begin
	mem2reg <= 1;
	state <= datamem; 

	end


       else if(dataout[31:26]==19)     //jump
	begin
	
	pc_new <= { pc_in[31:28] , 28'b0 } + (signvalue << 2) ;

	branch <= 1;

	state <= instruction_fetch;

	end
	


	

else if ( dataout[31:26] >= 13 && dataout[31:26] <= 16 )   //branch equal BNE BLT BGT

	begin
	

	state <= null_state;

	end


   else	
   state <= writeback;
   
   end

null_state:
state<=datamem;

   datamem:
   begin
	
if(dataout[31:26]>=13 && dataout[31:26] <= 16)
	begin


if(dataout[31:26] == 13 && zero == 1)
	begin
//$display ("zero=%d",zero);
	pc_new <= { pc_in[31:28] , 28'b0 } +(signvalue<<2) ;

	branch <= 1;
//$display("pc_out=%d",pc_new);

	end


if(dataout[31:26] == 14 && zero == 0 )
	begin

	pc_new <= { pc_in[31:28] , 28'b0 } +(signvalue<<2);


	branch <= 1;

	end

	state <= instruction_fetch;

end


   mem_enable <= 1;
   alu_signal <= 0;

   if(dataout[31:26] == 11)
	begin
	memread <= 1;
	mem2reg <= 1;
	state <= writeback;
	end

    if(dataout[31:26] == 12)
	begin
	memwrite <= 1;
	mem2reg <= 0;
	state <= instruction_fetch;
	end

end




   writeback:
   begin

   mem_enable <= 0;

   in_read <= 0;

   reg_write <= 1;

   state <= instruction_fetch;

   end

  
  endcase

//$display("pc_out=%d",pc_new);
   end


end



proccessor_datapath il1(dataout,signvalue,aluout,zero,clk,pc_write,in_read,pc_in,reg_write,alu_reset,regdst,alusrc,alu_signal,mem_enable,memwrite,memread,mem2reg);



endmodule

