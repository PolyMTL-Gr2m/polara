// Copyright 2022 ETH Zurich and University of Bologna and Polytechnique Montreal.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Yoan Fournier <yoan.fournier@polymtl.ca>
// Description:
// Polara ASIC SoC, containing :
//      - ARA System
//          - ARA (4 Lanes)
//          - CVA6
//      - L2
//      - Scan Chain
//      - JTAG + Debug
//      - Boot ROM
//      - FLL (blackbox)    
//      - UART

module polara_soc import axi_pkg::*; import ara_pkg::*; import polara_pkg::*; #(
    // RVV Parameters
    parameter  int           unsigned NrLanes      = 0, // Number of parallel vector lanes.
    // Support for floating-point data types
    parameter  fpu_support_e          FPUSupport   = FPUSupportHalfSingleDouble,
    // AXI Interface
    parameter  int           unsigned AxiDataWidth = 32*NrLanes,
    parameter  int           unsigned AxiAddrWidth = 64,
    parameter  int           unsigned AxiUserWidth = 1,
    parameter  int           unsigned AxiIdWidth   = 5,
    // Main memory
    parameter  int           unsigned L2NumWords   = 2**17, // 2 MB : 2**17 * 128 / 8 = 2097152 B
    parameter  int           unsigned NrL2Banks    = 8,
    // Dependant parameters. DO NOT CHANGE!
    localparam type                   axi_data_t   = logic [AxiDataWidth-1:0],
    localparam type                   axi_strb_t   = logic [AxiDataWidth/8-1:0],
    localparam type                   axi_addr_t   = logic [AxiAddrWidth-1:0],
    localparam type                   axi_user_t   = logic [AxiUserWidth-1:0],
    localparam type                   axi_id_t     = logic [AxiIdWidth-1:0]
) (
    // Clock inputs
    input  logic clk_i,
    input  logic clk_ref_i,
    input  logic clk_sel_i,
    input  logic rst_ni,

    // SCAN
    input  logic scan_en_i,
    input  logic scan_i,
    output logic scan_o,

    // Testmode
    input  logic testmode_i,

    // JTAG + Debug
    input  logic tck_i,
    input  logic tms_i,
    input  logic td_i,
    output logic td_o,
    input  logic trst_ni,

    // UART
    input  logic rx_i,
    output logic tx_o

    // Test Probes
);

    `include "axi/assign.svh"
    `include "axi/typedef.svh"
    `include "common_cells/registers.svh"

//////////////////////
//  Memory Regions  //
//////////////////////

    localparam NrAXIMasters = 1; // Actually masters, but slaves on the crossbar

    typedef enum int unsigned {
        L2MEM = 0,
        UART  = 1,
        CTRL  = 2
    } axi_slaves_e;
    localparam NrAXISlaves = CTRL + 1;

    // Memory Map
    // 1GByte of DDR (split between two chips on Genesys2)
    localparam logic [63:0] DRAMLength = 64'h40000000;
    localparam logic [63:0] UARTLength = 64'h1000;
    localparam logic [63:0] CTRLLength = 64'h1000;

    typedef enum logic [63:0] {
        DRAMBase = 64'h8000_0000,
        UARTBase = 64'hC000_0000,
        CTRLBase = 64'hD000_0000
    } soc_bus_start_e;

///////////
//  AXI  //
///////////

    // Ariane's AXI port data width
    localparam AxiNarrowDataWidth = 64;
    localparam AxiNarrowStrbWidth = AxiNarrowDataWidth / 8;
    // Ara's AXI port data width
    localparam AxiWideDataWidth   = AxiDataWidth;
    localparam AXiWideStrbWidth   = AxiWideDataWidth / 8;

    localparam AxiSocIdWidth  = AxiIdWidth - $clog2(NrAXIMasters);
    localparam AxiCoreIdWidth = AxiSocIdWidth - 1;

    // Internal types
    typedef logic [AxiNarrowDataWidth-1:0] axi_narrow_data_t;
    typedef logic [AxiNarrowStrbWidth-1:0] axi_narrow_strb_t;
    typedef logic [AxiSocIdWidth-1:0] axi_soc_id_t;
    typedef logic [AxiCoreIdWidth-1:0] axi_core_id_t;

    // AXI Typedefs
    `AXI_TYPEDEF_ALL(system, axi_addr_t, axi_id_t, axi_data_t, axi_strb_t, axi_user_t)
    `AXI_TYPEDEF_ALL(ara_axi, axi_addr_t, axi_core_id_t, axi_data_t, axi_strb_t, axi_user_t)
    `AXI_TYPEDEF_ALL(ariane_axi, axi_addr_t, axi_core_id_t, axi_narrow_data_t, axi_narrow_strb_t,
    axi_user_t)
    `AXI_TYPEDEF_ALL(soc_narrow, axi_addr_t, axi_soc_id_t, axi_narrow_data_t, axi_narrow_strb_t,
    axi_user_t)
    `AXI_TYPEDEF_ALL(soc_wide, axi_addr_t, axi_soc_id_t, axi_data_t, axi_strb_t, axi_user_t)
    `AXI_LITE_TYPEDEF_ALL(soc_narrow_lite, axi_addr_t, axi_narrow_data_t, axi_narrow_strb_t)

    // Buses
    system_req_t  system_axi_req;
    system_resp_t system_axi_resp;

    soc_wide_req_t    [NrAXISlaves-1:0] periph_wide_axi_req;
    soc_wide_resp_t   [NrAXISlaves-1:0] periph_wide_axi_resp;
    soc_narrow_req_t  [NrAXISlaves-1:0] periph_narrow_axi_req;
    soc_narrow_resp_t [NrAXISlaves-1:0] periph_narrow_axi_resp;

////////////////
//  Crossbar  //
////////////////

    localparam axi_pkg::xbar_cfg_t XBarCfg = '{
        NoSlvPorts        : NrAXIMasters,
        NoMstPorts        : NrAXISlaves,
        MaxMstTrans       : 4,
        MaxSlvTrans       : 4,
        FallThrough       : 1'b0,
        LatencyMode       : axi_pkg::CUT_MST_PORTS,
        AxiIdWidthSlvPorts: AxiSocIdWidth,
        AxiIdUsedSlvPorts : AxiSocIdWidth,
        UniqueIds         : 1'b0,
        AxiAddrWidth      : AxiAddrWidth,
        AxiDataWidth      : AxiWideDataWidth,
        NoAddrRules       : NrAXISlaves
    };

    axi_pkg::xbar_rule_64_t [NrAXISlaves-1:0] routing_rules;
    assign routing_rules = '{
        '{idx: CTRL, start_addr: CTRLBase, end_addr: CTRLBase + CTRLLength},
        '{idx: UART, start_addr: UARTBase, end_addr: UARTBase + UARTLength},
        '{idx: L2MEM, start_addr: DRAMBase, end_addr: DRAMBase + DRAMLength}
    };

    axi_xbar #(
        .Cfg          (XBarCfg                ),
        .slv_aw_chan_t(system_aw_chan_t       ),
        .mst_aw_chan_t(soc_wide_aw_chan_t     ),
        .w_chan_t     (system_w_chan_t        ),
        .slv_b_chan_t (system_b_chan_t        ),
        .mst_b_chan_t (soc_wide_b_chan_t      ),
        .slv_ar_chan_t(system_ar_chan_t       ),
        .mst_ar_chan_t(soc_wide_ar_chan_t     ),
        .slv_r_chan_t (system_r_chan_t        ),
        .mst_r_chan_t (soc_wide_r_chan_t      ),
        .slv_req_t    (system_req_t           ),
        .slv_resp_t   (system_resp_t          ),
        .mst_req_t    (soc_wide_req_t         ),
        .mst_resp_t   (soc_wide_resp_t        ),
        .rule_t       (axi_pkg::xbar_rule_64_t)
    ) i_soc_xbar (
        .clk_i                (clk_i               ),
        .rst_ni               (rst_ni              ),
        .test_i               (1'b0                ),
        .slv_ports_req_i      (system_axi_req      ),
        .slv_ports_resp_o     (system_axi_resp     ),
        .mst_ports_req_o      (periph_wide_axi_req ),
        .mst_ports_resp_i     (periph_wide_axi_resp),
        .addr_map_i           (routing_rules       ),
        .en_default_mst_port_i('0                  ),
        .default_mst_port_i   ('0                  )
    );

//////////
//  L2  //
//////////

    // The L2 memory does not support atomics
    soc_wide_req_t  l2mem_wide_axi_req_wo_atomics;
    soc_wide_resp_t l2mem_wide_axi_resp_wo_atomics;
    axi_atop_filter #(
        .AxiIdWidth     (AxiSocIdWidth  ),
        .AxiMaxWriteTxns(4              ),
        .req_t          (soc_wide_req_t ),
        .resp_t         (soc_wide_resp_t)
    ) i_l2mem_atop_filter (
        .clk_i     (clk_i                         ),
        .rst_ni    (rst_ni                        ),
        .slv_req_i (periph_wide_axi_req[L2MEM]    ),
        .slv_resp_o(periph_wide_axi_resp[L2MEM]   ),
        .mst_req_o (l2mem_wide_axi_req_wo_atomics ),
        .mst_resp_i(l2mem_wide_axi_resp_wo_atomics)
    );

    logic                      l2_req;
    logic                      l2_we;
    logic [AxiAddrWidth-1:0]   l2_addr;
    logic [AxiDataWidth/8-1:0] l2_be;
    logic [AxiDataWidth-1:0]   l2_wdata;
    logic [AxiDataWidth-1:0]   l2_rdata;
    logic                      l2_rvalid;

    axi_to_mem #(
        .AddrWidth (AxiAddrWidth   ),
        .DataWidth (AxiDataWidth   ),
        .IdWidth   (AxiSocIdWidth  ),
        .NumBanks  (1              ),
        .axi_req_t (soc_wide_req_t ),
        .axi_resp_t(soc_wide_resp_t)
    ) i_axi_to_mem (
        .clk_i       (clk_i                         ),
        .rst_ni      (rst_ni                        ),
        .axi_req_i   (l2mem_wide_axi_req_wo_atomics ),
        .axi_resp_o  (l2mem_wide_axi_resp_wo_atomics),
        .mem_req_o   (l2_req                        ),
        .mem_gnt_i   (l2_req                        ), // Always available
        .mem_we_o    (l2_we                         ),
        .mem_addr_o  (l2_addr                       ),
        .mem_strb_o  (l2_be                         ),
        .mem_wdata_o (l2_wdata                      ),
        .mem_rdata_i (l2_rdata                      ),
        .mem_rvalid_i(l2_rvalid                     ),
        .mem_atop_o  (/* Unused */                  ),
        .busy_o      (/* Unused */                  )
    );

    // Number of words per bank
    localparam integer unsigned L2BankNumWords = L2NumWords/NrL2Banks;

    // L2 Bank selection
    logic [$clog2(NrL2Banks)-1:0] l2_sel_d, l2_sel_q;
    assign l2_sel_d = l2_addr[$clog2(AxiDataWidth/8) +: $clog2(NrL2Banks)];
    `FF(l2_sel_q, l2_sel_d, '0)

    // Read data selection
    logic [NrL2Banks-1:0][AxiDataWidth-1:0] l2_bank_rdata;
    assign l2_rdata = l2_bank_rdata[l2_sel_q];


    for (genvar b = 0; b < NrL2Banks; b++) begin : gen_l2_macro
        tc_sram #(
            .NumWords (L2BankNumWords),
            .NumPorts (1             ),
            .DataWidth(AxiDataWidth  )
        ) i_dram (
            .clk_i  (clk_i                                                                        ),
            .rst_ni (rst_ni                                                                       ),
            .req_i  (l2_req                                                                       ),
            .we_i   (l2_we                                                                        ),
            .addr_i (l2_addr[$clog2(AxiDataWidth/8) + $clog2(NrL2Banks) +: $clog2(L2BankNumWords)]),
            .wdata_i(l2_wdata                                                                     ),
            .be_i   (l2_be                                                                        ),
            .rdata_o(l2_rdata[b]                                                                  )
        );

    end

    // One-cycle latency
    `FF(l2_rvalid, l2_req, 1'b0);

////////////
//  UART  //
////////////

    // UART signals
    logic        uart_pneable;
    logic        uart_pwrite;
    logic [31:0] uart_paddr;
    logic        uart_psel;
    logic [31:0] uart_pwdata;
    logic [31:0] uart_prdata;
    logic        uart_pready;
    logic        uart_pslverr;

    axi2apb_64_32 #(
        .AXI4_ADDRESS_WIDTH(AxiAddrWidth      ),
        .AXI4_RDATA_WIDTH  (AxiNarrowDataWidth),
        .AXI4_WDATA_WIDTH  (AxiNarrowDataWidth),
        .AXI4_ID_WIDTH     (AxiSocIdWidth     ),
        .AXI4_USER_WIDTH   (AxiUserWidth      ),
        .BUFF_DEPTH_SLAVE  (2                 ),
        .APB_ADDR_WIDTH    (32                )
    ) i_axi2apb_64_32_uart (
        .ACLK      (clk_i                                ),
        .ARESETn   (rst_ni                               ),
        .test_en_i (1'b0                                 ),
        .AWID_i    (periph_narrow_axi_req[UART].aw.id    ),
        .AWADDR_i  (periph_narrow_axi_req[UART].aw.addr  ),
        .AWLEN_i   (periph_narrow_axi_req[UART].aw.len   ),
        .AWSIZE_i  (periph_narrow_axi_req[UART].aw.size  ),
        .AWBURST_i (periph_narrow_axi_req[UART].aw.burst ),
        .AWLOCK_i  (periph_narrow_axi_req[UART].aw.lock  ),
        .AWCACHE_i (periph_narrow_axi_req[UART].aw.cache ),
        .AWPROT_i  (periph_narrow_axi_req[UART].aw.prot  ),
        .AWREGION_i(periph_narrow_axi_req[UART].aw.region),
        .AWUSER_i  (periph_narrow_axi_req[UART].aw.user  ),
        .AWQOS_i   (periph_narrow_axi_req[UART].aw.qos   ),
        .AWVALID_i (periph_narrow_axi_req[UART].aw_valid ),
        .AWREADY_o (periph_narrow_axi_resp[UART].aw_ready),
        .WDATA_i   (periph_narrow_axi_req[UART].w.data   ),
        .WSTRB_i   (periph_narrow_axi_req[UART].w.strb   ),
        .WLAST_i   (periph_narrow_axi_req[UART].w.last   ),
        .WUSER_i   (periph_narrow_axi_req[UART].w.user   ),
        .WVALID_i  (periph_narrow_axi_req[UART].w_valid  ),
        .WREADY_o  (periph_narrow_axi_resp[UART].w_ready ),
        .BID_o     (periph_narrow_axi_resp[UART].b.id    ),
        .BRESP_o   (periph_narrow_axi_resp[UART].b.resp  ),
        .BVALID_o  (periph_narrow_axi_resp[UART].b_valid ),
        .BUSER_o   (periph_narrow_axi_resp[UART].b.user  ),
        .BREADY_i  (periph_narrow_axi_req[UART].b_ready  ),
        .ARID_i    (periph_narrow_axi_req[UART].ar.id    ),
        .ARADDR_i  (periph_narrow_axi_req[UART].ar.addr  ),
        .ARLEN_i   (periph_narrow_axi_req[UART].ar.len   ),
        .ARSIZE_i  (periph_narrow_axi_req[UART].ar.size  ),
        .ARBURST_i (periph_narrow_axi_req[UART].ar.burst ),
        .ARLOCK_i  (periph_narrow_axi_req[UART].ar.lock  ),
        .ARCACHE_i (periph_narrow_axi_req[UART].ar.cache ),
        .ARPROT_i  (periph_narrow_axi_req[UART].ar.prot  ),
        .ARREGION_i(periph_narrow_axi_req[UART].ar.region),
        .ARUSER_i  (periph_narrow_axi_req[UART].ar.user  ),
        .ARQOS_i   (periph_narrow_axi_req[UART].ar.qos   ),
        .ARVALID_i (periph_narrow_axi_req[UART].ar_valid ),
        .ARREADY_o (periph_narrow_axi_resp[UART].ar_ready),
        .RID_o     (periph_narrow_axi_resp[UART].r.id    ),
        .RDATA_o   (periph_narrow_axi_resp[UART].r.data  ),
        .RRESP_o   (periph_narrow_axi_resp[UART].r.resp  ),
        .RLAST_o   (periph_narrow_axi_resp[UART].r.last  ),
        .RUSER_o   (periph_narrow_axi_resp[UART].r.user  ),
        .RVALID_o  (periph_narrow_axi_resp[UART].r_valid ),
        .RREADY_i  (periph_narrow_axi_req[UART].r_ready  ),
        .PENABLE   (uart_penable                         ),
        .PWRITE    (uart_pwrite                          ),
        .PADDR     (uart_paddr                           ),
        .PSEL      (uart_psel                            ),
        .PWDATA    (uart_pwdata                          ),
        .PRDATA    (uart_prdata                          ),
        .PREADY    (uart_pready                          ),
        .PSLVERR   (uart_pslverr                         )
    );

    axi_dw_converter #(
        .AxiSlvPortDataWidth(AxiWideDataWidth     ),
        .AxiMstPortDataWidth(AxiNarrowDataWidth   ),
        .AxiAddrWidth       (AxiAddrWidth         ),
        .AxiIdWidth         (AxiSocIdWidth        ),
        .AxiMaxReads        (2                    ),
        .ar_chan_t          (soc_wide_ar_chan_t   ),
        .mst_r_chan_t       (soc_narrow_r_chan_t  ),
        .slv_r_chan_t       (soc_wide_r_chan_t    ),
        .aw_chan_t          (soc_narrow_aw_chan_t ),
        .b_chan_t           (soc_wide_b_chan_t    ),
        .mst_w_chan_t       (soc_narrow_w_chan_t  ),
        .slv_w_chan_t       (soc_wide_w_chan_t    ),
        .axi_mst_req_t      (soc_narrow_req_t     ),
        .axi_mst_resp_t     (soc_narrow_resp_t    ),
        .axi_slv_req_t      (soc_wide_req_t       ),
        .axi_slv_resp_t     (soc_wide_resp_t      )
    ) i_axi_slave_uart_dwc (
        .clk_i     (clk_i                       ),
        .rst_ni    (rst_ni                      ),
        .slv_req_i (periph_wide_axi_req[UART]   ),
        .slv_resp_o(periph_wide_axi_resp[UART]  ),
        .mst_req_o (periph_narrow_axi_req[UART] ),
        .mst_resp_i(periph_narrow_axi_resp[UART])
    );
    apb_uart i_apb_uart (
        .CLK     ( clk_i           ),
        .RSTN    ( rst_ni          ),
        .PSEL    ( uart_psel       ),
        .PENABLE ( uart_penable    ),
        .PWRITE  ( uart_pwrite     ),
        .PADDR   ( uart_paddr[4:2] ),
        .PWDATA  ( uart_pwdata     ),
        .PRDATA  ( uart_prdata     ),
        .PREADY  ( uart_pready     ),
        .PSLVERR ( uart_pslverr    ),
        .INT     (                 ),
        .OUT1N   (                 ),
        .OUT2N   (                 ),
        .RTSN    (                 ),
        .DTRN    (                 ),
        .CTSN    ( 1'b0            ),
        .DSRN    ( 1'b0            ),
        .DCDN    ( 1'b0            ),
        .RIN     ( 1'b0            ),
        .SIN     ( rx_i            ),
        .SOUT    ( tx_o            )
    );

///////////////////////////
//  JTAG + Debug Module  //
///////////////////////////

    import dm::hartinfo_t;
    import dm::dmi_req_t;
    import dm::dmi_resp_t;

    // Debug parameters
    localparam logic [15:0]     PartNumber = 1;
    localparam logic [31:0]     IDCODE     = (dm::DbgVersion013 << 28) | (PartNumber << 12) | 32'b1;
    localparam                  NrHarts    = 1;
    hartinfo_t                  hartinfo;

    assign hartinfo = '{
        zero1:      0, 
        nscratch:   0, 
        zero0:      0, 
        dataaccess: 0, 
        datasize:   0, 
        dataaddr:   0
    };

    // Debug Module interface (DMI)
    logic                       debug_req_ready;
    dmi_resp_t                  debug_resp;
    logic                       jtag_req_valid;
    dmi_req_t                   jtag_dmi_req;
    logic                       jtag_resp_ready;
    logic                       jtag_resp_valid;
    logic         [NrHarts-1:0] dm_debug_req;
    logic                       ndmreset;
    logic                       dmi_rst_no;

    // System Bus Acces (SBA)
    logic           sb_req;
    logic [31:0]    sb_addr;
    logic           sb_we;
    logic [31:0]    sb_wdata;
    logic [3:0]     sb_be;
    logic           sb_gnt;
    logic           sb_rvalid;
    logic [31:0]    sb_rdata;

    dmi_jtag #(
        .IdcodeValue (IDCODE)
    ) i_jtag (
        .clk_i           (clk_i           ),
        .rst_ni          (rst_ni          ),
        .testmode_i      (testmode_i      ),
        .dmi_rst_no      (dmi_rst_no      ),
        .dmi_req_o       (jtag_dmi_req    ),
        .dmi_req_valid_o (jtag_req_valid  ),
        .dmi_req_ready_i (jtag_resp_ready ),
        .dmi_resp_i      (debug_resp      ),
        .dmi_resp_ready_o(jtag_resp_ready ),
        .dmi_resp_valid_i(jtag_resp_valid ),
        .tck_i           (tck_i           ), // TODO: Clock ?? same as system
        .tms_i           (tms_i           ),
        .trst_ni         (trst_ni         ),
        .td_i            (td_i            ),
        .td_o            (td_o            ),
        .tdo_oe_o        (/* Unused */    )
    );  

    dm_top #(
        .NrHarts  (NrHarts),
        .BusWidth (32     )
    ) i_dm_top (
        .clk_i           (clk_i           ),
        .rst_ni          (rst_ni          ),
        .testmode_i      (testmode_i      ),
        .ndmreset_o      (ndmreset        ), // TODO: Access to CVA6 IRQ through ARA system ?
        .dmactive_o      (/* Unused */    ),
        .debug_req_o     (                ),
        .unavailable_i   ('1              ),
        .hartinfo_i      (hartinfo        ),
        .slave_req_i     ('0              ),
        .slave_we_i      ('0              ),
        .slave_addr_i    ('0              ),
        .slave_be_i      ('0              ),
        .slave_wdata_i   ('0              ),
        .slave_rdata_o   (/* Unused */    ),
        .master_req_o    (sb_req          ),
        .master_add_o    (sb_addr         ),
        .master_we_o     (sb_we           ),
        .master_wdata_o  (sb_wdata        ),
        .master_be_o     (sb_be           ),
        .master_gnt_i    (master_gnt_i    ),
        .master_r_valid_i(sb_rvalid       ),
        .master_r_rdata_i(sb_rdata        ),
        .dmi_rst_ni      (dmi_rst_no      ),
        .dmi_req_valid_i (jtag_req_valid  ),
        .dmi_req_ready_o (debug_req_ready ),
        .dmi_req_i       (jtag_dmi_req    ),
        .dmi_resp_valid_o(jtag_resp_valid ),
        .dmi_resp_ready_i(jtag_resp_ready ),
        .dmi_resp_o      (debug_resp      )
    );

/////////////////////////
//  Control registers  //
/////////////////////////

    soc_narrow_lite_req_t  axi_lite_ctrl_registers_req;
    soc_narrow_lite_resp_t axi_lite_ctrl_registers_resp;

    axi_to_axi_lite #(
        .AxiAddrWidth   (AxiAddrWidth          ),
        .AxiDataWidth   (AxiNarrowDataWidth    ),
        .AxiIdWidth     (AxiSocIdWidth         ),
        .AxiUserWidth   (AxiUserWidth          ),
        .AxiMaxReadTxns (1                     ),
        .AxiMaxWriteTxns(1                     ),
        .FallThrough    (1'b0                  ),
        .full_req_t     (soc_narrow_req_t      ),
        .full_resp_t    (soc_narrow_resp_t     ),
        .lite_req_t     (soc_narrow_lite_req_t ),
        .lite_resp_t    (soc_narrow_lite_resp_t)
    ) i_axi_to_axi_lite (
        .clk_i     (clk_i                        ),
        .rst_ni    (rst_ni                       ),
        .test_i    (1'b0                         ),
        .slv_req_i (periph_narrow_axi_req[CTRL]  ),
        .slv_resp_o(periph_narrow_axi_resp[CTRL] ),
        .mst_req_o (axi_lite_ctrl_registers_req  ),
        .mst_resp_i(axi_lite_ctrl_registers_resp )
    );

    ctrl_registers #(
        .DRAMBaseAddr   (DRAMBase              ),
        .DRAMLength     (DRAMLength            ),
        .DataWidth      (AxiNarrowDataWidth    ),
        .AddrWidth      (AxiAddrWidth          ),
        .axi_lite_req_t (soc_narrow_lite_req_t ),
        .axi_lite_resp_t(soc_narrow_lite_resp_t)
    ) i_ctrl_registers (
        .clk_i                (clk_i                       ),
        .rst_ni               (rst_ni                      ),
        .axi_lite_slave_req_i (axi_lite_ctrl_registers_req ),
        .axi_lite_slave_resp_o(axi_lite_ctrl_registers_resp),
        .dram_base_addr_o     (/* Unused */                ),
        .dram_end_addr_o      (/* Unused */                ),
        .exit_o               (exit_o                      )
    );

    axi_dw_converter #(
        .AxiSlvPortDataWidth(AxiWideDataWidth    ),
        .AxiMstPortDataWidth(AxiNarrowDataWidth  ),
        .AxiAddrWidth       (AxiAddrWidth        ),
        .AxiIdWidth         (AxiSocIdWidth       ),
        .AxiMaxReads        (2                   ),
        .ar_chan_t          (soc_wide_ar_chan_t  ),
        .mst_r_chan_t       (soc_narrow_r_chan_t ),
        .slv_r_chan_t       (soc_wide_r_chan_t   ),
        .aw_chan_t          (soc_narrow_aw_chan_t),
        .b_chan_t           (soc_narrow_b_chan_t ),
        .mst_w_chan_t       (soc_narrow_w_chan_t ),
        .slv_w_chan_t       (soc_wide_w_chan_t   ),
        .axi_mst_req_t      (soc_narrow_req_t    ),
        .axi_mst_resp_t     (soc_narrow_resp_t   ),
        .axi_slv_req_t      (soc_wide_req_t      ),
        .axi_slv_resp_t     (soc_wide_resp_t     )
    ) i_axi_slave_ctrl_dwc (
        .clk_i     (clk_i                       ),
        .rst_ni    (rst_ni                      ),
        .slv_req_i (periph_wide_axi_req[CTRL]   ),
        .slv_resp_o(periph_wide_axi_resp[CTRL]  ),
        .mst_req_o (periph_narrow_axi_req[CTRL] ),
        .mst_resp_i(periph_narrow_axi_resp[CTRL])
    );

//////////////
//  System  //
//////////////

    localparam ariane_pkg::ariane_cfg_t ArianeAraConfig = '{
        RASDepth             : 2,
        BTBEntries           : 32,
        BHTEntries           : 128,
        // idempotent region
        NrNonIdempotentRules : 2,
        NonIdempotentAddrBase: {64'b0, 64'b0},
        NonIdempotentLength  : {64'b0, 64'b0},
        NrExecuteRegionRules : 3,
        //                      DRAM,       Boot ROM,   Debug Module
        ExecuteRegionAddrBase: {DRAMBase, 64'h1_0000, 64'h0},
        ExecuteRegionLength  : {DRAMLength, 64'h10000, 64'h1000},
        // cached region
        NrCachedRegionRules  : 1,
        CachedRegionAddrBase : {DRAMBase},
        CachedRegionLength   : {DRAMLength},
        //  cache config
        Axi64BitCompliant    : 1'b1,
        SwapEndianess        : 1'b0,
        // debug
        DmBaseAddress        : 64'h0,
        NrPMPEntries         : 0
    };

`ifndef TARGET_GATESIM
    ara_system #(
        .NrLanes           (NrLanes              ),
        .FPUSupport        (FPUSupport           ),
        .ArianeCfg         (ArianeAraConfig      ),
        .AxiAddrWidth      (AxiAddrWidth         ),
        .AxiIdWidth        (AxiCoreIdWidth       ),
        .AxiNarrowDataWidth(AxiNarrowDataWidth   ),
        .AxiWideDataWidth  (AxiDataWidth         ),
        .ara_axi_ar_t      (ara_axi_ar_chan_t    ),
        .ara_axi_aw_t      (ara_axi_aw_chan_t    ),
        .ara_axi_b_t       (ara_axi_b_chan_t     ),
        .ara_axi_r_t       (ara_axi_r_chan_t     ),
        .ara_axi_w_t       (ara_axi_w_chan_t     ),
        .ara_axi_req_t     (ara_axi_req_t        ),
        .ara_axi_resp_t    (ara_axi_resp_t       ),
        .ariane_axi_ar_t   (ariane_axi_ar_chan_t ),
        .ariane_axi_aw_t   (ariane_axi_aw_chan_t ),
        .ariane_axi_b_t    (ariane_axi_b_chan_t  ),
        .ariane_axi_r_t    (ariane_axi_r_chan_t  ),
        .ariane_axi_w_t    (ariane_axi_w_chan_t  ),
        .ariane_axi_req_t  (ariane_axi_req_t     ),
        .ariane_axi_resp_t (ariane_axi_resp_t    ),
        .system_axi_ar_t   (system_ar_chan_t     ),
        .system_axi_aw_t   (system_aw_chan_t     ),
        .system_axi_b_t    (system_b_chan_t      ),
        .system_axi_r_t    (system_r_chan_t      ),
        .system_axi_w_t    (system_w_chan_t      ),
        .system_axi_req_t  (system_req_t         ),
        .system_axi_resp_t (system_resp_t        ))
`else
    ara_system
`endif
    i_system (
        .clk_i        (clk_i              ),
        .rst_ni       (rst_ni             ),
        .boot_addr_i  (DRAMBase           ), // start fetching from DRAM
        .scan_enable_i(1'b0               ),
        .scan_data_i  (1'b0               ),
        .scan_data_o  (/* Unconnected */  ),
        .axi_req_o    (system_axi_req     ),
        .axi_resp_i   (system_axi_resp    )
    );

//////////////////
//  Assertions  //
//////////////////

if (NrLanes == 0)
    $error("[ara_soc] Ara needs to have at least one lane.");

if (AxiDataWidth == 0)
    $error("[ara_soc] The AXI data width must be greater than zero.");

if (AxiAddrWidth == 0)
    $error("[ara_soc] The AXI address width must be greater than zero.");

if (AxiUserWidth == 0)
    $error("[ara_soc] The AXI user width must be greater than zero.");

if (AxiIdWidth == 0)
    $error("[ara_soc] The AXI ID width must be greater than zero.");

endmodule : polara_soc