`timescale 1ns/1ns

`define CLK_PERIOD 83.3333333
`define BITIME  8000 
`define FETCH_FROM_FLASH
`ifdef ICARUS_VERILOG
    `define   SIM_TIME    500_000_000
    `define   SIM_LEVEL   0
    `define   TEST_FILE   "../mem/test_flash.mem" 

    `include "rtl_fpga/n5_netlists.v"
    `include "sst26wf080b.v"
    `include "23LC512.v" 
    `include "M24LC16B.v"
`else
    `define   TEST_FILE   "test.mem" 
`endif

`ifdef USE_RESET_BTN
    `define RESET_DELAY  80000000
    `define RESET_VAL    1     // when button is pressed
`else
    `define RESET_DELAY 800
    `define RESET_VAL   0
`endif

module N5_FPGA_TB;

localparam  WE_OFF = 32'h4C000000,
                SS_OFF = 32'h4C000004,
                SCK_OFF= 32'h4C000008,
                OE_OFF = 32'h4C00000C,
                SO_OFF = 32'h4C000010,
                SI_OFF = 32'h4C000014,
                ID_OFF = 32'h4C000018; 

    reg HCLK, HRESETn;
    reg  TX;
    wire RX;
    reg M_clk;
    
    wire [3:0]		fdi;
    wire [3:0]    	fdo;
    wire [3:0]      fdio;
    wire     	    fdoe;
    wire          	fsclk;
    wire          	fcen;

    wire [15: 0] GPIO_Sys0_S2;

    wire [0: 0] RsRx_Sys0_SS0_S0;
    wire [0: 0] RsTx_Sys0_SS0_S0;
    
    wire [0: 0] RsRx_Sys0_SS0_S1;
    wire [0: 0] RsTx_Sys0_SS0_S1;

    wire [0: 0] MSI_Sys0_SS0_S2;
    wire [0: 0] MSO_Sys0_SS0_S2;
    wire [0: 0] SSn_Sys0_SS0_S2;
    wire [0: 0] SCLK_Sys0_SS0_S2;

    wire [0: 0] MSI_Sys0_SS0_S3;
    wire [0: 0] MSO_Sys0_SS0_S3;
    wire [0: 0] SSn_Sys0_SS0_S3;
    wire [0: 0] SCLK_Sys0_SS0_S3;

    wire [0: 0] scl_Sys0_SS0_S4;
    wire [0: 0] sda_Sys0_SS0_S4;

    wire [0: 0] scl_Sys0_SS0_S5;
    wire [0: 0] sda_Sys0_SS0_S5;

    wire [0: 0] pwm_Sys0_SS0_S6;
	wire [0: 0] pwm_Sys0_SS0_S7;

    // I2C E2PROM connected to I2C0
    wire    scl, sda;
    
    /*wire        fm_ce_n,fm_sck;
            wire [3:0]  SIO; 
            wire [3:0]  fm_din, fm_dout,fm_douten;*/
           
    `ifdef FETCH_FROM_FLASH
        // Program Flash 
        sst26wf080b FLASH(
            .SCK(fsclk),
            .SIO(fdio),
            .CEb(fcen)
        );
         defparam FLASH.I0.Tsce = 25_000;
         defparam FLASH.I0.Tpp = 1_000;
    `endif

    /* N5_SoC Core */
    soc_core MUV (
    `ifdef USE_POWER_PINS
        .VPWR(1'b1),
        .VGND(1'b0),
    `endif
        .HCLK(HCLK),
        .HRESETn(HRESETn),

        // .SYSTICKCLKDIV(8'd100),
        // .NMI(1'b0),
        .UART_MASTER_RX(TX),
        .UART_MASTER_TX(RX),
        
        .fd_Sys0_S0(fdio),
        .fsclk_Sys0_S0(fsclk),
        .fcen_Sys0_S0(fcen),
        
        /*.fdi_Sys0_S0(fm_din),
        .fdo_Sys0_S0(fm_dout),
        .fdoe_Sys0_S0(fm_douten),
        .fsclk_Sys0_S0(fm_sck),
        .fcen_Sys0_S0(fm_ce_n),*/


        .GPIO_Sys0_S2(GPIO_Sys0_S2),
    	
        .RsRx_Sys0_SS0_S0(RsRx_Sys0_SS0_S0),
        .RsTx_Sys0_SS0_S0(RsTx_Sys0_SS0_S0),

        .RsRx_Sys0_SS0_S1(RsRx_Sys0_SS0_S1),
        .RsTx_Sys0_SS0_S1(RsTx_Sys0_SS0_S1),
       
        .MSI_Sys0_SS0_S2(MSI_Sys0_SS0_S2),
        .MSO_Sys0_SS0_S2(MSO_Sys0_SS0_S2),
        .SSn_Sys0_SS0_S2(SSn_Sys0_SS0_S2),
        .SCLK_Sys0_SS0_S2(SCLK_Sys0_SS0_S2),

        .MSI_Sys0_SS0_S3(MSI_Sys0_SS0_S3),
        .MSO_Sys0_SS0_S3(MSO_Sys0_SS0_S3),
        .SSn_Sys0_SS0_S3(SSn_Sys0_SS0_S3),
        .SCLK_Sys0_SS0_S3(SCLK_Sys0_SS0_S3),

        .scl_Sys0_SS0_S4(scl),
        .sda_Sys0_SS0_S4(sda),
    
        .scl_Sys0_SS0_S5(scl_Sys0_SS0_S5),
        .sda_Sys0_SS0_S5(sda_Sys0_SS0_S5),
    
        .pwm_Sys0_SS0_S6(pwm_Sys0_SS0_S6),
	    .pwm_Sys0_SS0_S7(pwm_Sys0_SS0_S7)

    );
    
        /* Program Flash */

        
           
         /*    assign SIO[0] = fm_douten[0] ? fm_dout[0] : 1'bz;
               assign SIO[1] = fm_douten[1] ? fm_dout[1] : 1'bz;
               assign SIO[2] = fm_douten[2] ? fm_dout[2] : 1'bz;
               assign SIO[3] = fm_douten[3] ? fm_dout[3] : 1'bz;
               
        assign fm_din = SIO;
               
        sst26wf080b flash (.SCK(fm_sck),.SIO(SIO),.CEb(fm_ce_n));*/
        
    // GPIO Loopback!
    wire [15:0] GPIO_PINS;

    assign GPIO_PINS[15:8] = GPIO_Sys0_S2[7:0];
    assign GPIO_Sys0_S2  = GPIO_PINS;

    // Serial Terminal connected to UART0 TX*/
    terminal term(.rx(RsTx_Sys0_SS0_S0));

    // SPI SRAM connected to SPI0
    wire SPI_HOLD = 1'b1;
    M23LC512 SPI_SRAM(
        .RESET(~HRESETn),
        .SO_SIO1(MSI_Sys0_SS0_S2),
        .SI_SIO0(MSO_Sys0_SS0_S2),
        .CS_N(SSn_Sys0_SS0_S2),
        .SCK(SCLK_Sys0_SS0_S2),
        .HOLD_N_SIO3(SPI_HOLD)
	);

	pullup p1(scl); // pullup scl line
	pullup p2(sda); // pullup sda line

    M24LC16B I2C_E2PROM(
        .A0(1'b0), 
        .A1(1'b0), 
        .A2(1'b0), 
        .WP(1'b0), 
        .SDA(sda), 
        .SCL(scl), 
        .RESET(~HRESETn)
    );

    `ifdef FETCH_FROM_FLASH 
        // Load the application into the flash memory
        integer i;
        initial begin
            #1  $readmemh(`TEST_FILE, FLASH.I0.memory);
            $display("---------N5 Flash -----------");
            for ( i=6304 ; i< 6432 ; i=i+1) begin
            $display("Memory byte [%0d",i,"]:", "0x%0h",FLASH.I0.memory[i]);
                end 
        end
    `endif
    
    
    // Clock and Rest Generation
    initial begin
        //Inputs initialization
        HCLK = 0;
        HRESETn = 1'bx;    
        M_clk =0;    
        #50;
        HRESETn = `RESET_VAL;
        #(`RESET_DELAY)
        @(posedge HCLK);
        HRESETn <= ~(`RESET_VAL);
    end

    always #(`CLK_PERIOD / 2.00) HCLK = ~ HCLK;
    always #(`BITIME) M_clk = ~ M_clk;
    
    /*integer x; 
           initial begin
            UART_MASTER_RX = 1'b0;
               for(x=0; x<10; x=x+1)
               begin
                   UART_MASTER_RX = ~UART_MASTER_RX;
                   #10;
               end
               end*/
      parameter Kilo = 1024;                  // size of kilo 2^10
      parameter Mega = (Kilo * Kilo);         // Size of mega 2^20
      parameter S080B=1;                      // size of SST26WF080B 1M bytes
      parameter S016B=2;                      // size of SST26WF016B 2M bytes
      parameter S032B=4;                      // size of SST26WF032B 4M bytes
      parameter S040B=0;    //For memory size less than 1M
      parameter Ksize = 0;
      parameter Msize = S032B;                            // Size of Memory in Mega bites. use S080B, S016B, S032B
      parameter Memsize = (Msize * Mega ) + (Ksize * Kilo);                // Size of Memory in bytes
               
     // Test Case
                   reg[31:0] data;
                   reg [7:0] D1,D2,D3,D4;
                   //reg[7:0]program[Memsize-1:0];
                   //integer i; 
              /*  initial begin
                   
                       TX = 1;
                       //#80000000;
                       //FW_RD(ID_OFF, data);
                       //$finish;
                       FW_ENABLE;
           
                       SPI_OE(4'b0001);
                      // READ_ST_REG(data); //Z0
                       //assign D1 = data | 8'h02;
                       //ENABLE_QE (D1);
                       //#25000
                      // READ_ST_REG(data); //0a
                       SPI_STATRT;
                       SPI_BYTE_WR(8'hFF);
                       SPI_STOP;
                       SPI_STATRT;
                       SPI_BYTE_WR(8'h9F);
                      // READ_ST_REG(data); //0a
                       SPI_BYTE_RD(data);
                       $display("JEDEC Byte 0:%x", data);
                       SPI_BYTE_RD(data);
                       $display("JEDEC Byte 1:%x", data);
                       SPI_BYTE_RD(data);
                       $display("JEDEC Byte 2:%x", data);
                       SPI_STOP;
                       //ENABLE_QE;
                      
                       end */
                   /*   initial begin
                        FW_ENABLE;
                        SPI_OE(4'b0001);
                       SPI_STATRT;
                       SPI_BYTE_WR(8'hFF);
                        SPI_STOP;
                        
                            SPI_OE(4'b0001);     
                            SPI_STATRT;  
                             SPI_BYTE_WR(8'h05);  
                             SPI_BYTE_RD(data); 
                              SPI_STOP;      
                              
                              SPI_OE(4'b0001);     
                              SPI_STATRT;  
                              SPI_BYTE_WR(8'h35);  
                              SPI_BYTE_RD(data); 
                              SPI_STOP;      
                               //FLASH_WEN;
                              // FLASH_PROT_UNLK;
                               //FLASH_WEN;
                               //#30_000;
                              // FLASH_CHIP_ERASE;
                              // #30_000;
                              // FLASH_WEN;
                              // FLASH_WORD_PROG(24'h0, 32'h0A0B0C0D);
                              // #2_000;
                              // FLASH_WDI;
                              // FLASH_WORD_RD(24'h0, data);
                               
                               //FLASH_BYTE_RD(24'h0, D2);
                               //FLASH_BYTE_RD(24'h0, D3);
                               //FLASH_BYTE_RD(24'h0, D4);
                               
                               
                      
                       $finish;
                   end */
    
    `ifdef ICARUS_VERILOG
        // Dump file
        initial begin
            $dumpfile("N5_FPGA_TB.vcd");
            $dumpvars(`SIM_LEVEL, N5_FPGA_TB);
            #`SIM_TIME;
            $finish;
        end
    `endif
    // Terminate the smulation with ebreak instruction.
    // Calculate the CPI using the CSRs
    /*`ifndef GL
        always @ (posedge HCLK) 
            if(MUV.CPU.N5.instr_ebreak) begin
            //$display("CPI=%d.%0d", MUV.N5.CSR_CYCLE/MUV.N5.CSR_INSTRET,(MUV.N5.CSR_CYCLE%MUV.N5.CSR_INSTRET)*10/MUV.N5.CSR_INSTRET );
            $finish;
            end
    `endif*/
    
    // Monitor Flash memory reads
    //always @(posedge HCLK)
    //    if(MUV.N5.HTRANS[1] & MUV.N5.HREADY & MUV.N5.HSEL_FLASH)
    //    $display("Flash Read A:%X (%0t)", HADDR, $time);

UART_MON MON (.RX(TX));
// Baud rate 1228800
    // Bit time ~ 813.8ns
    // 8N1
    localparam BITTIME = 8000 ; //baud rate 125000
    //localparam BITTIME = 52083.3; //baud rate 19200
    //localparam BITTIME = 813.8; //baud rate 1228800
    //localparam BITTIME = 7812.5 ; //baud rate 128000
task UART_SEND (input [7:0] data);
            begin : task_body
                integer i;
                #BITTIME;
                @(posedge HCLK);
                TX = 0;
                #BITTIME;
                for(i=0; i<8; i=i+1) begin
                    TX = data[i];
                    #BITTIME;
                end
                TX = 1;
                //#BITTIME;
            end
        endtask
    
        task UART_REC (output [7:0] data);
            begin : task_body
                integer i;
                @(negedge RX);
                #(BITTIME+(BITTIME/2));
                for(i=0; i<8; i=i+1) begin
                    data[i] = RX;
                    #BITTIME;
                end
            end
        endtask
    
        task FW_WR(input [31:0] A, input [31:0] D);
            begin
                UART_SEND(8'hA3);
                UART_SEND(A[7:0]);
                UART_SEND(A[15:8]);
                UART_SEND(A[23:16]);
                UART_SEND(A[31:24]);
                UART_SEND(D[7:0]);
                UART_SEND(D[15:8]);
                UART_SEND(D[23:16]);
                UART_SEND(D[31:24]);
            end
        endtask
    
        task FW_RD(input [31:0] A, output [31:0] D); //4c000018
            begin
                UART_SEND(8'hA5);
                
                UART_SEND(A[7:0]);
                
                UART_SEND(A[15:8]);
                
                UART_SEND(A[23:16]);
                
                UART_SEND(A[31:24]);
                
                UART_REC(D[7:0]);
                UART_REC(D[15:8]);
                UART_REC(D[23:16]);
                UART_REC(D[31:24]);
            end
        endtask
    
        task SPI_STATRT;
            FW_WR(SS_OFF, 0);
        endtask
    
        task SPI_STOP;
            FW_WR(SS_OFF, 1);
        endtask
    
        task SPI_OE(input [3:0] data);
            FW_WR(OE_OFF, data);
        endtask
    
        task FW_ENABLE;
            FW_WR(WE_OFF,32'hA5A85501);
        endtask
    
        task SPI_BYTE_WR(input [7:0] data);
            begin : task_body
                integer i;
                for(i=7; i>=0; i=i-1) begin
                    FW_WR(SO_OFF, data[i]);
                    FW_WR(SCK_OFF, 1);
                    FW_WR(SCK_OFF, 0);
                end
            end
        endtask
    
        task SPI_WORD_WR(input [32:0] data);
            begin 
                SPI_BYTE_WR(data[7:0]);
                SPI_BYTE_WR(data[15:8]);
                SPI_BYTE_WR(data[23:16]);
                SPI_BYTE_WR(data[31:24]);
            end
        endtask
    
        task SPI_BYTE_RD(output [7:0] data);
            begin : task_body
                integer i;
                reg [31:0] word;
                for(i=7; i>=0; i=i-1) begin
                    FW_WR(SCK_OFF, 1);
                    FW_RD(SI_OFF, word);
                    data[i] = word[0];
                    FW_WR(SCK_OFF, 0);
                end
            end
        endtask
    task FLASH_WEN;
                begin : task_body
                    SPI_OE(4'b0001);
                    SPI_STATRT;
                    SPI_BYTE_WR(8'h06); // write enable
                    SPI_STOP;
                end
            endtask
            
  
            
  task READ_ST_REG(output [7:0] data);
                       
                            begin : task_body
                                SPI_OE(4'b0001);
                                SPI_STATRT;
                                SPI_BYTE_WR(8'h35); 
                                SPI_BYTE_RD(data);
                                SPI_STOP;
                            end
                        endtask
        
            task FLASH_PROT_UNLK;
                begin : task_body
                    SPI_OE(4'b0001);
                    SPI_STATRT;
                    SPI_BYTE_WR(8'h98); // global protection unlock
                    SPI_STOP;
                end
            endtask
          task ENABLE_QE (input[7:0]data);
                            begin : task_body
                                FLASH_WEN;
                                SPI_OE(4'b0001);
                                SPI_STATRT;
                                SPI_BYTE_WR(8'h01); // global protection unlock
                                SPI_BYTE_WR(data);
                                SPI_BYTE_WR(data);
                                SPI_STOP;
                            end
                        endtask
        
            task FLASH_WDI;
                begin : task_body
                    SPI_OE(4'b0001);
                    SPI_STATRT;
                    SPI_BYTE_WR(8'h04);
                    SPI_STOP;
                end
            endtask
            
            task FLASH_CHIP_ERASE;
                begin : task_body
                    SPI_OE(4'b0001);
                    SPI_STATRT;
                    SPI_BYTE_WR(8'hC7);
                    SPI_STOP;
                end
            endtask
        
            // Page program 
            // Re-implement
            task FLASH_WORD_PROG(input[23:0] A, input[31:0] D);
                begin : task_body
                    SPI_OE(4'b0001);
                    SPI_STATRT;
                    SPI_BYTE_WR(8'h02);
                    SPI_BYTE_WR(A[23:16]);
                    SPI_BYTE_WR(A[15:8]);
                    SPI_BYTE_WR(A[7:0]);
                    SPI_BYTE_WR(D[7:0]);
                    SPI_BYTE_WR(D[15:8]);
                    SPI_BYTE_WR(D[23:16]);
                    SPI_BYTE_WR(D[31:24]);
                    SPI_STOP;
                end
            endtask
   task FLASH_WORD_RD(input[23:0] A, output[31:0] D);
                            begin : task_body
                                SPI_OE(4'b0001);
                                SPI_STATRT;
                                SPI_BYTE_WR(8'h0B);
                                SPI_BYTE_WR(A[23:16]);
                                SPI_BYTE_WR(A[15:8]);
                                SPI_BYTE_WR(A[7:0]);
                                SPI_BYTE_RD(D[7:0]);
                                                    SPI_BYTE_RD(D[15:8]);
                                                    SPI_BYTE_RD(D[23:16]);
                                                    SPI_BYTE_RD(D[31:24]);
                                SPI_STOP;
                            end
                        endtask
    endmodule

module terminal #(parameter bit_time = 8000) (input wire rx);

    integer i;
    reg [7:0] char;
    initial begin
        forever begin
            @(negedge rx);
            i = 0;
            char = 0;
            #(3*bit_time/2);
            for(i=0; i<8; i=i+1) begin
                char[i] = rx;
                #bit_time;
            end
            $write("%c", char);
        end
    end


endmodule

//module UART_MON #(parameter BITTIME=813.8)(input RX);
//module UART_MON #(parameter BITTIME=7812.5)(input RX);
module UART_MON #(parameter BITTIME=8000)(input wire RX);
//module UART_MON #(parameter BITTIME=52083.3)(input RX);
    reg [7:0] data;
    integer i;
    initial begin
        forever begin
            @(negedge RX);
            #BITTIME;
            for(i=0; i<8; i=i+1) begin
                data = {RX, data[7:1]};
                #BITTIME;
            end
            #BITTIME;
            //$write("%c", data);
            $display("0x%X", data);
        end
    end

endmodule
