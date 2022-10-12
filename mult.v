module mult(multiplier,multiplicand,product1,product2);
   input [31:0]  multiplier, multiplicand;
   output        product1,product2;
	reg Cy;
   reg [31:0]    product1,product2;

   integer       i,j;

   always @( multiplier or multiplicand )
     begin
        
        product1 = 0;
	product2=0;
            
        for(i=0; i<32; i=i+1)
          if( multiplier[i] == 1'b1 ) 
		
{product2,product1} = {product2,product1} + ( multiplicand << i );


  
     end
     
endmodule
