`default_nettype none
`timescale 1ns/1ns

`ifdef USE_RESET_BTN
	`define HRESETn_PORT .HRESETn(~HRESETn_Sync)
`else
	`define HRESETn_PORT .HRESETn(HRESETn)
`endif

module Clock_divider(clock_in,clock_out
    );
input wire clock_in; // input clock on FPGA
output reg clock_out; // output clock after dividing the input clock by divisor
reg[27:0] counter=28'd0;
parameter DIVISOR = 28'd12; //12
// The frequency of the output clk_out
//  = The frequency of the input clk_in divided by DIVISOR
// For example: Fclk_in = 50Mhz, if you want to get 1Hz signal to blink LEDs
// You will modify the DIVISOR parameter value to 28'd50.000.000
// Then the frequency of the output clk_out = 50Mhz/50.000.000 = 1Hz
always @(posedge clock_in)
begin
 counter <= counter + 28'd1;
 if(counter>=(DIVISOR-1))
  counter <= 28'd0;
 clock_out <= (counter<DIVISOR/2)?1'b1:1'b0;
end
endmodule


module soc_core ( 
`ifdef USE_POWER_PINS
	input VPWR,
	input VGND,
`endif
	input wire HCLK, 
	//input wire HCLK_HF,
	input wire HRESETn,
	
	// input wire 			NMI,
	// input wire [7:0]	SYSTICKCLKDIV,
	
	//input wire 			NMI,
    //input wire             EXT_IRQ,
    //input wire [23:0]    SYSTICKCLKDIV,

	input wire UART_MASTER_RX,
	output wire UART_MASTER_TX,
	
	inout  wire [3: 0] 	fd_Sys0_S0,
	output wire [0: 0] 	fsclk_Sys0_S0,
	output wire [0: 0] 	fcen_Sys0_S0,
	
	/*input wire [3: 0] 	fdi_Sys0_S0,
        output wire [3: 0]     fdo_Sys0_S0,
        output wire [3: 0]     fdoe_Sys0_S0,
        output wire [0: 0]     fsclk_Sys0_S0,
        output wire [0: 0]     fcen_Sys0_S0,*/

	inout wire [15: 0] GPIO_Sys0_S2,

	input  wire [0: 0] RsRx_Sys0_SS0_S0,
	output wire [0: 0] RsTx_Sys0_SS0_S0,

	input  wire [0: 0] RsRx_Sys0_SS0_S1,
	output wire [0: 0] RsTx_Sys0_SS0_S1,

	input  wire [0: 0] MSI_Sys0_SS0_S2,
	output wire [0: 0] MSO_Sys0_SS0_S2,
	output wire [0: 0] SSn_Sys0_SS0_S2,
	output wire [0: 0] SCLK_Sys0_SS0_S2,
	
	input  wire [0: 0] MSI_Sys0_SS0_S3,
	output wire [0: 0] MSO_Sys0_SS0_S3,
	output wire [0: 0] SSn_Sys0_SS0_S3,
	output wire [0: 0] SCLK_Sys0_SS0_S3,
	
	inout wire [0: 0] scl_Sys0_SS0_S4,
	inout wire [0: 0] sda_Sys0_SS0_S4,
	inout wire [0: 0] scl_Sys0_SS0_S5,
	inout wire [0: 0] sda_Sys0_SS0_S5,

	output wire [0: 0] pwm_Sys0_SS0_S6,
	output wire [0: 0] pwm_Sys0_SS0_S7
);
    
    wire [15: 0] GPIOIN_Sys0_S2;
	wire [15: 0] GPIOOUT_Sys0_S2;
	wire [15: 0] GPIOPU_Sys0_S2;
	wire [15: 0] GPIOPD_Sys0_S2;
	wire [15: 0] GPIOOEN_Sys0_S2;
    
    wire [3: 0] 	fdi_Sys0_S0;
    wire [3: 0] 	fdo_Sys0_S0;
    wire [3: 0]     fdoe_Sys0_S0;

	wire [31: 0] HADDR_Sys0;
	wire [31: 0] HWDATA_Sys0;
	wire HWRITE_Sys0;
	wire [1: 0] HTRANS_Sys0;
	wire [2:0] HSIZE_Sys0;

	wire HREADY_Sys0;
	wire [31: 0] HRDATA_Sys0;

	wire [31: 0] SRAMRDATA_Sys0_S1;
	wire [3: 0] SRAMWEN_Sys0_S1;
	wire [31: 0] SRAMWDATA_Sys0_S1;
	wire [0: 0] SRAMCS0_Sys0_S1;
	wire [11: 0] SRAMADDR_Sys0_S1;

	wire [31: 0] ROMRDATA;
	wire [12: 0] ROMADDR;
	wire [0: 0]  ROMCS0;

	// AHB LITE Master1 Signals
	wire [31:0] M1_HADDR;
	wire [0:0] M1_HREADY;
	wire [0:0] M1_HWRITE;
	wire [1:0] M1_HTRANS;
	wire [2:0] M1_HSIZE;
	wire [31:0] M1_HWDATA;
	wire [31:0] M1_HRDATA;
	
	wire [31: 0] M1_IRQ;

	wire [3:0] M1_HPROT;
	wire [2:0] M1_HBURST;
	wire M1_HBUSREQ;
	wire M1_HLOCK;
	wire M1_HGRANT;
	
	// AHB LITE Master2 Signals
	wire [31:0] M2_HADDR;
	wire [0:0] M2_HREADY;
	wire [0:0] M2_HWRITE;
	wire [1:0] M2_HTRANS;
	wire [2:0] M2_HSIZE;
	wire [31:0] M2_HWDATA;
	wire [31:0] M2_HRDATA;
	
	wire [31: 0] M2_IRQ;

	wire [3:0] M2_HPROT;
	wire [2:0] M2_HBURST;
	wire M2_HBUSREQ;
	wire M2_HLOCK;
	wire M2_HGRANT;

	wire [31:0] SRAMRDATA0, SRAMRDATA1, SRAMRDATA2; 

	/*assign M2_HREADY = HREADY_Sys0; 
	assign M2_HRDATA = HRDATA_Sys0;

	assign HADDR_Sys0 = M2_HADDR; 
	assign HWDATA_Sys0 = M2_HWDATA; 
	assign HWRITE_Sys0 = M2_HWRITE; 
	assign HTRANS_Sys0 = M2_HTRANS; 
	assign HSIZE_Sys0 = M2_HSIZE;
	assign M2_HGRANT = 1'b1;
	assign M2_HBUSREQ = 1'b1;*/
	
	wire [7:0] SYSTICKCLKDIV = 8'd100;
	wire NMI = 1'b0;
	wire EXT_IRQ = 1'b0;
	
    //wire HCLK;
	//Clock_divider div (HCLK_HF, HCLK);
    
    gpio_bidir #(
        .WIDTH(16)
    ) gpio_bidir (
        .dio_buf(GPIO_Sys0_S2),
        .din_i(GPIOOUT_Sys0_S2),
        .dout_o(GPIOIN_Sys0_S2),
        .in_not_out_i(~GPIOOEN_Sys0_S2)
    );
    
    wire [3:0] fdoe = {{~fdoe_Sys0_S0}}; // invert enable
    //wire [3:0] fdoe_bus = {4{fdoe_Sys0_S0}}; // invert enable
    
    gpio_bidir #(
        .WIDTH(4)
    ) flash_bidir (
        .dio_buf(fd_Sys0_S0),
        .din_i(fdo_Sys0_S0),
        .dout_o(fdi_Sys0_S0),
        .in_not_out_i(fdoe) 
    );
       
       
    wire [0: 0] scl_i_Sys0_SS0_S4;
    wire [0: 0] scl_o_Sys0_SS0_S4;
    wire [0: 0] scl_oen_o_Sys0_SS0_S4;
      
    gpio_bidir #(
            .WIDTH(1)
    ) i2c_s1_scl_bidir (
            .dio_buf(scl_Sys0_SS0_S4),
            .din_i(scl_o_Sys0_SS0_S4),
            .dout_o(scl_i_Sys0_SS0_S4),
            .in_not_out_i(scl_oen_o_Sys0_SS0_S4) // invert enable
    );
     
    wire [0: 0] sda_i_Sys0_SS0_S4;
    wire [0: 0] sda_o_Sys0_SS0_S4;
    wire [0: 0] sda_oen_o_Sys0_SS0_S4;
    
    gpio_bidir #(
        .WIDTH(1)
    ) i2c_s1_sda_bidir (
        .dio_buf(sda_Sys0_SS0_S4),
        .din_i(sda_o_Sys0_SS0_S4),
        .dout_o(sda_i_Sys0_SS0_S4),
        .in_not_out_i(sda_oen_o_Sys0_SS0_S4)
    );
        
    wire [0: 0] scl_i_Sys0_SS0_S5;
    wire [0: 0] scl_o_Sys0_SS0_S5;
    wire [0: 0] scl_oen_o_Sys0_SS0_S5;
       
    gpio_bidir #(
        .WIDTH(1)
    ) i2c_s2_scl_bidir (
        .dio_buf(scl_Sys0_SS0_S5),
        .din_i(scl_o_Sys0_SS0_S5),
        .dout_o(scl_i_Sys0_SS0_S5),
        .in_not_out_i(scl_oen_o_Sys0_SS0_S5)
    );
     
     
    wire [0: 0] sda_i_Sys0_SS0_S5;
    wire [0: 0] sda_o_Sys0_SS0_S5;
    wire [0: 0] sda_oen_o_Sys0_SS0_S5;
     
    gpio_bidir #(
        .WIDTH(1)
    ) i2c_s2_sda_bidir (
        .dio_buf(sda_Sys0_SS0_S5),
        .din_i(sda_o_Sys0_SS0_S5),
        .dout_o(sda_i_Sys0_SS0_S5),
        .in_not_out_i(sda_oen_o_Sys0_SS0_S5)
    );

	// Add synchronizer for the reset pin if it is mapped to one of the FPGA buttons 
	`ifdef USE_RESET_BTN
		wire HRESETn_Sync;
		btn_sync btn_sync (
			.in(HRESETn),
			.clk(HCLK),
			.out(HRESETn_Sync)	
		);
	`endif
    // baud rate = 1228800
     //   AHB_UART_MASTER #(.PRESCALE(12)) UM( //128000 
     AHB_UART_MASTER UM(
            .HCLK(HCLK),
            `HRESETn_PORT,
            .HREADY(M1_HREADY),
            .HWDATA(M1_HWDATA),
            .HRDATA(M1_HRDATA),
            .HSIZE(M1_HSIZE),
            .HWRITE(M1_HWRITE),
            .HTRANS(M1_HTRANS),
            .HADDR(M1_HADDR),
    
    
            .RX(UART_MASTER_RX),
            .TX(UART_MASTER_TX)
        );
    /*AHB_UART_MASTER #(.PRESCALE(38)) UM( //19200 
                    .HCLK(HCLK),
                    .HRESETn(HRESETn),
            
                    .HREADY(HREADY_Sys0),
                    .HWDATA(HWDATA_Sys0),
                    .HRDATA(HRDATA_Sys0),
                    .HSIZE(HSIZE_Sys0),
                    .HWRITE(HWRITE_Sys0),
                    .HTRANS(HTRANS_Sys0),
                    .HADDR(HADDR_Sys0),
            
            
                    .RX(UART_MASTER_RX),
                    .TX(UART_MASTER_TX)
                );*/
	//AHBlite_SYS0 instantiation
	AHBlite_sys_0 ahb_sys_0_uut(
	`ifdef USE_POWER_PINS
		.VPWR(VPWR),
		.VGND(VGND),
	`endif
		`HRESETn_PORT,
		.HCLK(HCLK),
		.HADDR(HADDR_Sys0),
		.HWDATA(HWDATA_Sys0),
		.HWRITE(HWRITE_Sys0),
		.HTRANS(HTRANS_Sys0),
		.HSIZE(HSIZE_Sys0),
    
		.HREADY(HREADY_Sys0),
		.HRDATA(HRDATA_Sys0),
		
		// QSPI Interface
		.fdi_S0(fdi_Sys0_S0),
		.fdo_S0(fdo_Sys0_S0),
		.fdoe_S0(fdoe_Sys0_S0),
		.fsclk_S0(fsclk_Sys0_S0),
		.fcen_S0(fcen_Sys0_S0),

`ifdef USE_ROM
		// ROM Inteface
		.ROMRDATA(ROMRDATA), 
		.ROMCS0(ROMCS0),
	    .ROMADDR(ROMADDR),  
`endif	
		// SRAM Interface
		.SRAMRDATA_S1(SRAMRDATA_Sys0_S1),
		.SRAMWEN_S1(SRAMWEN_Sys0_S1),
		.SRAMWDATA_S1(SRAMWDATA_Sys0_S1),
		.SRAMCS0_S1(SRAMCS0_Sys0_S1),
		.SRAMADDR_S1(SRAMADDR_Sys0_S1),

		// GPIO Interface
		.GPIOIN_S2(GPIOIN_Sys0_S2),
		.GPIOOUT_S2(GPIOOUT_Sys0_S2),
		.GPIOPU_S2(GPIOPU_Sys0_S2),
		.GPIOPD_S2(GPIOPD_Sys0_S2),
		.GPIOOEN_S2(GPIOOEN_Sys0_S2),
		
		// APB Bus
		// UART 0
		.RsRx_SS0_S0(RsRx_Sys0_SS0_S0),
		.RsTx_SS0_S0(RsTx_Sys0_SS0_S0),
		
		// UART 1
		.RsRx_SS0_S1(RsRx_Sys0_SS0_S1),
		.RsTx_SS0_S1(RsTx_Sys0_SS0_S1),

		// SPI 0 Interface
		.MSI_SS0_S2(MSI_Sys0_SS0_S2),
		.MSO_SS0_S2(MSO_Sys0_SS0_S2),
		.SSn_SS0_S2(SSn_Sys0_SS0_S2),
		.SCLK_SS0_S2(SCLK_Sys0_SS0_S2),

		// SPI 1 Interface
		.MSI_SS0_S3(MSI_Sys0_SS0_S3),
		.MSO_SS0_S3(MSO_Sys0_SS0_S3),
		.SSn_SS0_S3(SSn_Sys0_SS0_S3),
		.SCLK_SS0_S3(SCLK_Sys0_SS0_S3),

		// I2C 0 Interface
		.scl_i_SS0_S4(scl_i_Sys0_SS0_S4),
		.scl_o_SS0_S4(scl_o_Sys0_SS0_S4),
		.scl_oen_o_SS0_S4(scl_oen_o_Sys0_SS0_S4),
		.sda_i_SS0_S4(sda_i_Sys0_SS0_S4),
		.sda_o_SS0_S4(sda_o_Sys0_SS0_S4),
		.sda_oen_o_SS0_S4(sda_oen_o_Sys0_SS0_S4),

		// I2C 1 Interface
		.scl_i_SS0_S5(scl_i_Sys0_SS0_S5),
		.scl_o_SS0_S5(scl_o_Sys0_SS0_S5),
		.scl_oen_o_SS0_S5(scl_oen_o_Sys0_SS0_S5),
		.sda_i_SS0_S5(sda_i_Sys0_SS0_S5),
		.sda_o_SS0_S5(sda_o_Sys0_SS0_S5),
		.sda_oen_o_SS0_S5(sda_oen_o_Sys0_SS0_S5),

		// PMW 0 & 1 Interfaces
		.pwm_SS0_S6(pwm_Sys0_SS0_S6),
		.pwm_SS0_S7(pwm_Sys0_SS0_S7),

		.IRQ(M2_IRQ)

	);

`ifdef USE_ROM
	ROM #(
		.MEM_WORDS(`ROM_MEM_WORDS)
	)ROM (
		.CLK(HCLK),
		.EN(ROMCS0),
		.Do(ROMRDATA),
		.A(ROMADDR)
	);
`endif

	RAM_4Kx32 RAM (
	`ifdef USE_POWER_PINS
		.VPWR(VPWR),
		.VGND(VGND),
	`endif
		.CLK(HCLK),
		.WE(SRAMWEN_Sys0_S1),
		.EN(SRAMCS0_Sys0_S1),
		.Di(SRAMWDATA_Sys0_S1),
		.Do(SRAMRDATA_Sys0_S1),
		.A(SRAMADDR_Sys0_S1[11:0])
	);

	// Instantiation of NfiVe32
	/*NfiVe32_SYS #(
		.PC_START_ADDRESS(`PC_START_ADDRESS)
	) CPU (
`ifdef USE_POWER_PINS
	.VPWR(VPWR),
	.VGND(VGND),
`endif
		.HCLK(HCLK),
		`HRESETn_PORT,
		.HADDR(M2_HADDR),
		.HREADY(M2_HREADY),
		.HWRITE(M2_HWRITE),
		.HTRANS(M2_HTRANS),
		.HSIZE(M2_HSIZE),
		.HWDATA(M2_HWDATA),
		.HRDATA(M2_HRDATA),

		//NMI
		.NMI(NMI),

		//Interrupts
		.IRQ(M2_IRQ),

		// SYSTICK Divisor
		.SYSTICKCLKDIV(SYSTICKCLKDIV)
	);*/
	ibex_wrapper ibex_core (
        `ifdef USE_POWER_PINS
            .VPWR(VPWR),
            .VGND(VGND),
        `endif
            .HCLK(HCLK),
            `HRESETn_PORT,
    
            .HADDR(M2_HADDR),
            .HREADY(M2_HREADY),
            .HWRITE(M2_HWRITE),
            .HTRANS(M2_HTRANS),
            .HSIZE(M2_HSIZE),
            .HWDATA(M2_HWDATA),
            .HRDATA(M2_HRDATA),
    
            .NMI(NMI),
            .EXT_IRQ(EXT_IRQ),
            .IRQ({M2_IRQ[27:16], 4'b0}),
            .SYSTICKCLKDIV(SYSTICKCLKDIV)
    
        );
        
    AHB_MUX_2M1S master_mux (
                .HCLK(HCLK),
                `HRESETn_PORT,
                
                // Port 1
                    // input 
                .HADDR_M1(M1_HADDR),
                .HTRANS_M1(M1_HTRANS),
                .HWRITE_M1(M1_HWRITE),
                .HSIZE_M1(M1_HSIZE),
                .HWDATA_M1(M1_HWDATA),
                //output 
                .HREADY_M1(M1_HREADY),  
                .HRDATA_M1(M1_HRDATA),
                
                // Port 2
                    //input 
                .HADDR_M2(M2_HADDR),
                .HTRANS_M2(M2_HTRANS),
                .HWRITE_M2(M2_HWRITE),
                .HSIZE_M2(M2_HSIZE),
                .HWDATA_M2(M2_HWDATA),
                //output 
                .HREADY_M2(M2_HREADY),
                .HRDATA_M2(M2_HRDATA),
                
                // Master Port
                    //input
                .HREADY(HREADY_Sys0),
                .HRDATA(HRDATA_Sys0),
                //OUTPUT
                .HADDR(HADDR_Sys0),
                .HTRANS(HTRANS_Sys0),
                .HWRITE(HWRITE_Sys0),
                .HSIZE(HSIZE_Sys0),
                .HWDATA(HWDATA_Sys0)
            );
            
 
endmodule

