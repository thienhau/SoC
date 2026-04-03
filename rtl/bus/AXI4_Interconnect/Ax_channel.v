module dsp_Ax_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    parameter OUTSTANDING_AMT   = 8,
    parameter OUTST_CTN_W       = $clog2(OUTSTANDING_AMT) + 1,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_BURST_W     = 2,    // Width of xBURST 
    parameter TRANS_DATA_LEN_W  = 8,    // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter TRANS_DATA_SIZE_W = 3,    // Bus width of xSIZE
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT),
    parameter SLV_ID_MSB_IDX    = 30,
    parameter SLV_ID_LSB_IDX    = 30,
    // Timeout configuration
    parameter TIMEOUT_W         = 10    // Address-phase timeout width (2^W-1 cycles), 0 = disabled
)
(
    // Input declaration
    // Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // To Master (slave interface of the interconnect)
    // Write/Read address channel
    input   [TRANS_MST_ID_W-1:0]            m_AxID_i,
    input   [ADDR_WIDTH-1:0]                m_AxADDR_i,
    input   [TRANS_BURST_W-1:0]             m_AxBURST_i,
    input   [TRANS_DATA_LEN_W-1:0]          m_AxLEN_i,
    input   [TRANS_DATA_SIZE_W-1:0]         m_AxSIZE_i,
    input                                   m_AxLOCK_i,
    input   [3:0]                            m_AxCACHE_i,
    input   [2:0]                            m_AxPROT_i,
    input   [3:0]                            m_AxQOS_i,
    input   [3:0]                            m_AxREGION_i,
    input                                   m_AxVALID_i,
    // To xDATA channel Dispatcher
    input                                   m_xVALID_i,
    input                                   m_xREADY_i,
    // To Slave Arbitration
    // Write/Read address channel (master)
    input   [SLV_AMT-1:0]                   sa_AxREADY_i,
    // Output declaration
    // To Master (slave interface of interconnect)
    // Write/Read address channel (master)
    output                                  m_AxREADY_o,
    // To Slave Arbitration
    // Write/Read address channel
    output  [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_AxID_o,
    output  [ADDR_WIDTH*SLV_AMT-1:0]        sa_AxADDR_o,
    output  [TRANS_BURST_W*SLV_AMT-1:0]     sa_AxBURST_o,
    output  [TRANS_DATA_LEN_W*SLV_AMT-1:0]  sa_AxLEN_o,
    output  [TRANS_DATA_SIZE_W*SLV_AMT-1:0] sa_AxSIZE_o,
    output  [SLV_AMT-1:0]                   sa_AxLOCK_o,
    output  [4*SLV_AMT-1:0]                 sa_AxCACHE_o,
    output  [3*SLV_AMT-1:0]                 sa_AxPROT_o,
    output  [4*SLV_AMT-1:0]                 sa_AxQOS_o,
    output  [4*SLV_AMT-1:0]                 sa_AxREGION_o,
    output  [SLV_AMT-1:0]                   sa_AxVALID_o,
    output  [OUTST_CTN_W-1:0]               sa_Ax_outst_ctn_o,
    // To xDATA channel Dispatcher
    output  [SLV_ID_W-1:0]                  dsp_xDATA_slv_id_o,
    output                                  dsp_xDATA_disable_o,
    output                                  dsp_xDATA_err_o,
    output  [TRANS_MST_ID_W-1:0]            dsp_xDATA_axid_o,
    output                                  dsp_xDATA_last_o,
    // To WRESP channel Dispatcher
    output  [SLV_ID_W-1:0]                  dsp_WRESP_slv_id_o,
    output                                  dsp_WRESP_shift_en_o,
    output                                  dsp_WRESP_err_o,
    output  [TRANS_MST_ID_W-1:0]            dsp_WRESP_axid_o
);
    // Local parameters initialization
    localparam SLV_ID_MAP_W = SLV_ID_MSB_IDX - SLV_ID_LSB_IDX + 1;
    localparam ADDR_INFO_W  = 1 + TRANS_MST_ID_W + SLV_ID_MAP_W + TRANS_DATA_LEN_W;  // err + axid + slvid + len
    localparam Ax_INFO_W    = TRANS_MST_ID_W + ADDR_WIDTH + TRANS_BURST_W + TRANS_DATA_LEN_W + TRANS_DATA_SIZE_W + 1 + 4 + 3 + 4 + 4; // +LOCK+CACHE+PROT+QOS+REGION
    // Internal variable declaration
    genvar slv_idx;
    
    // Internal signal declaration
    // xADDR order fifo
    wire    [ADDR_INFO_W-1:0]       addr_info;
    wire    [ADDR_INFO_W-1:0]       addr_info_valid;
    wire                            fifo_xa_order_wr_en;
    wire                            fifo_xa_order_rd_en;
    wire                            fifo_xa_order_empty;
    wire                            fifo_xa_order_full;
    // Handshake detector
    wire                            Ax_handshake_occur;
    wire                            xDATA_handshake_occur;
    // Misc
    wire    [SLV_ID_MAP_W-1:0]      slv_id;
    wire    [SLV_AMT-1:0]           slv_sel;
    wire    [SLV_ID_MAP_W-1:0]      addr_slv_mapping;
    wire    [TRANS_DATA_LEN_W-1:0]  AxLEN_valid;
    wire    [TRANS_MST_ID_W-1:0]    AxID_valid;
    wire                            addr_err_valid;
    // Error / timeout
    wire                            addr_invalid;
    wire                            addr_err;
    wire                            normal_accept;
    wire                            timeout_expire;
    // Transfer counter
    wire    [TRANS_DATA_LEN_W-1:0]  transfer_ctn_nxt;
    wire    [TRANS_DATA_LEN_W-1:0]  transfer_ctn_incr;
    wire                            transfer_ctn_match;
    // Master skid buffer
    wire    [Ax_INFO_W-1:0]         msb_bwd_data;
    wire                            msb_bwd_valid;
    wire                            msb_bwd_ready;
    wire    [Ax_INFO_W-1:0]         msb_fwd_data;
    wire                            msb_fwd_valid;
    wire                            msb_fwd_ready;
    wire    [TRANS_MST_ID_W-1:0]    msb_fwd_AxID;
    wire    [ADDR_WIDTH-1:0]        msb_fwd_AxADDR;
    wire    [TRANS_BURST_W-1:0]     msb_fwd_AxBURST;
    wire    [TRANS_DATA_LEN_W-1:0]  msb_fwd_AxLEN;
    wire    [TRANS_DATA_SIZE_W-1:0] msb_fwd_AxSIZE;
    wire                            msb_fwd_AxLOCK;
    wire    [3:0]                   msb_fwd_AxCACHE;
    wire    [2:0]                   msb_fwd_AxPROT;
    wire    [3:0]                   msb_fwd_AxQOS;
    wire    [3:0]                   msb_fwd_AxREGION;
    
    // Reg declaration
    reg     [TRANS_DATA_LEN_W-1:0]  transfer_ctn_r;
    
    // Module
    // xADDR order FIFO 
    sync_fifo #(
        .FIFO_TYPE(0),
        .DATA_WIDTH(ADDR_INFO_W),
        .FIFO_DEPTH(OUTSTANDING_AMT)
    ) fifo_xaddr_order (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .data_i(addr_info),
        .data_o(addr_info_valid),
        .wr_valid_i(fifo_xa_order_wr_en),
        .rd_valid_i(fifo_xa_order_rd_en),
        .empty_o(fifo_xa_order_empty),
        .full_o(fifo_xa_order_full),
        .wr_ready_o(),
        .rd_ready_o(),
        .almost_empty_o(),
        .almost_full_o(),
        .counter(sa_Ax_outst_ctn_o)
    );
    // Master skid buffer
    skid_buffer #(
        .SBUF_TYPE(1),
        .DATA_WIDTH(Ax_INFO_W)
    ) mst_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (msb_bwd_data),
        .bwd_valid_i(msb_bwd_valid),
        .fwd_ready_i(msb_fwd_ready),
        .fwd_data_o (msb_fwd_data),
        .bwd_ready_o(msb_bwd_ready),
        .fwd_valid_o(msb_fwd_valid)
    );
    
    // Combinational logic
    // xADDR order FIFO
    assign addr_slv_mapping = msb_fwd_AxADDR[SLV_ID_MSB_IDX:SLV_ID_LSB_IDX];
    assign addr_info = {addr_err, msb_fwd_AxID, addr_slv_mapping, msb_fwd_AxLEN};
    assign {addr_err_valid, AxID_valid, slv_id, AxLEN_valid} = addr_info_valid;
    assign fifo_xa_order_wr_en = Ax_handshake_occur;
    assign fifo_xa_order_rd_en = transfer_ctn_match & xDATA_handshake_occur;
    // Error / timeout detection
    assign addr_invalid = (addr_slv_mapping >= SLV_AMT);
    assign normal_accept = (~addr_invalid) & sa_AxREADY_i[addr_slv_mapping[SLV_ID_W-1:0]] & (~fifo_xa_order_full);
    assign addr_err = addr_invalid | timeout_expire;
    // Handshake detector
    assign Ax_handshake_occur = msb_fwd_valid & msb_fwd_ready;
    assign xDATA_handshake_occur = m_xVALID_i & m_xREADY_i;
    // Transfer counter
    assign transfer_ctn_nxt = (transfer_ctn_match) ? {TRANS_DATA_LEN_W{1'b0}} : transfer_ctn_incr;
    assign transfer_ctn_incr = transfer_ctn_r + 1'b1;
    assign transfer_ctn_match = transfer_ctn_r == AxLEN_valid;
    // Output 
    // Output to Slave Arbitration
    generate 
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
            assign sa_AxID_o[TRANS_MST_ID_W*(slv_idx+1)-1-:TRANS_MST_ID_W]          = msb_fwd_AxID;
            assign sa_AxADDR_o[ADDR_WIDTH*(slv_idx+1)-1-:ADDR_WIDTH]                = msb_fwd_AxADDR;
            assign sa_AxBURST_o[TRANS_BURST_W*(slv_idx+1)-1-:TRANS_BURST_W]         = msb_fwd_AxBURST;
            assign sa_AxLEN_o[TRANS_DATA_LEN_W*(slv_idx+1)-1-:TRANS_DATA_LEN_W]     = msb_fwd_AxLEN;
            assign sa_AxSIZE_o[TRANS_DATA_SIZE_W*(slv_idx+1)-1-:TRANS_DATA_SIZE_W]  = msb_fwd_AxSIZE;
            assign sa_AxLOCK_o[slv_idx]                                             = msb_fwd_AxLOCK;
            assign sa_AxCACHE_o[4*(slv_idx+1)-1-:4]                                 = msb_fwd_AxCACHE;
            assign sa_AxPROT_o[3*(slv_idx+1)-1-:3]                                  = msb_fwd_AxPROT;
            assign sa_AxQOS_o[4*(slv_idx+1)-1-:4]                                   = msb_fwd_AxQOS;
            assign sa_AxREGION_o[4*(slv_idx+1)-1-:4]                                = msb_fwd_AxREGION;
            assign sa_AxVALID_o[slv_idx]                                            = msb_fwd_valid & (addr_slv_mapping == slv_idx) & (~fifo_xa_order_full) & (~addr_err);
        end
    endgenerate
    // Master skid buffer
    assign msb_bwd_data     = {m_AxID_i, m_AxADDR_i, m_AxBURST_i, m_AxLEN_i, m_AxSIZE_i, m_AxLOCK_i, m_AxCACHE_i, m_AxPROT_i, m_AxQOS_i, m_AxREGION_i};
    assign msb_bwd_valid    = m_AxVALID_i;
    assign msb_fwd_ready    = normal_accept | ((~fifo_xa_order_full) & addr_err);
    assign {msb_fwd_AxID, msb_fwd_AxADDR, msb_fwd_AxBURST, msb_fwd_AxLEN, msb_fwd_AxSIZE, msb_fwd_AxLOCK, msb_fwd_AxCACHE, msb_fwd_AxPROT, msb_fwd_AxQOS, msb_fwd_AxREGION} = msb_fwd_data;
    // Output to xDATA dispatcher
    assign dsp_xDATA_slv_id_o = slv_id;
    assign dsp_xDATA_disable_o = fifo_xa_order_empty;
    assign dsp_xDATA_err_o = addr_err_valid;
    assign dsp_xDATA_axid_o = AxID_valid;
    assign dsp_xDATA_last_o = transfer_ctn_match;
    // Output to WRESP dispatcher
    assign dsp_WRESP_slv_id_o = slv_id;
    assign dsp_WRESP_shift_en_o = fifo_xa_order_rd_en;
    assign dsp_WRESP_err_o = addr_err_valid;
    assign dsp_WRESP_axid_o = AxID_valid;
    // Output to Slave Arbitration
    assign m_AxREADY_o = msb_bwd_ready;
    
    // Flip-flop
    always @(posedge ACLK_i) begin
        if(~ARESETn_i) begin
            transfer_ctn_r <= {TRANS_DATA_LEN_W{1'b0}};
        end
        else if(xDATA_handshake_occur) begin
            transfer_ctn_r <= transfer_ctn_nxt;
        end
    end
    
    // Timeout counter
    generate
        if (TIMEOUT_W > 0) begin : TIMEOUT_LOGIC
            reg [TIMEOUT_W-1:0] timeout_ctn_r;
            wire timeout_counting = msb_fwd_valid & (~addr_invalid) & (~normal_accept) & (~fifo_xa_order_full);
            assign timeout_expire = timeout_counting & (&timeout_ctn_r);
            always @(posedge ACLK_i) begin
                if (~ARESETn_i || ~timeout_counting)
                    timeout_ctn_r <= {TIMEOUT_W{1'b0}};
                else if (~timeout_expire)
                    timeout_ctn_r <= timeout_ctn_r + 1'b1;
            end
        end else begin : NO_TIMEOUT
            assign timeout_expire = 1'b0;
        end
    endgenerate

endmodule
