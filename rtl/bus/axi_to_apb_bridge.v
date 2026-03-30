`timescale 1ns / 1ps

module axi_to_apb_bridge #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 5
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ==========================================
    // AXI4 FULL SLAVE INTERFACE
    // ==========================================
    // Write Address Channel
    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire [2:0]               s_axi_awprot,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    // Write Data Channel
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    // Write Response Channel
    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // Read Address Channel
    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire [2:0]               s_axi_arprot,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    // Read Data Channel
    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // ==========================================
    // APB4 MASTER INTERFACE
    // ==========================================
    output reg  [ADDR_WIDTH-1:0]    m_apb_paddr,
    output reg  [2:0]               m_apb_pprot,
    output reg                      m_apb_psel,
    output reg                      m_apb_penable,
    output reg                      m_apb_pwrite,
    output reg  [DATA_WIDTH-1:0]    m_apb_pwdata,
    output reg  [(DATA_WIDTH/8)-1:0]m_apb_pstrb,
    input  wire                     m_apb_pready,
    input  wire [DATA_WIDTH-1:0]    m_apb_prdata,
    input  wire                     m_apb_pslverr
);

    // ==========================================
    // KHAI BÁO TÍN HIỆU NỘI BỘ VÀ TRẠNG THÁI
    // ==========================================
    // Write FSM States
    localparam W_IDLE    = 3'd0;
    localparam W_DATA    = 3'd1;
    localparam W_ARB_REQ = 3'd2;
    localparam W_RESP    = 3'd3;

    // Read FSM States
    localparam R_IDLE    = 3'd0;
    localparam R_ARB_REQ = 3'd1;
    localparam R_DATA    = 3'd2;

    // APB Arbiter States
    localparam APB_IDLE   = 2'd0;
    localparam APB_SETUP  = 2'd1;
    localparam APB_ACCESS = 2'd2;

    // Write Context Registers
    reg [2:0]            w_state;
    reg [ID_WIDTH-1:0]   w_id_reg;
    reg [ADDR_WIDTH-1:0] w_addr_reg;
    reg [7:0]            w_len_reg;
    reg [2:0]            w_size_reg;
    reg [2:0]            w_prot_reg;
    reg [7:0]            w_beat_cnt;
    reg [1:0]            w_err_accum;
    reg [DATA_WIDTH-1:0] w_data_reg;
    reg [(DATA_WIDTH/8)-1:0] w_strb_reg;

    // Read Context Registers
    reg [2:0]            r_state;
    reg [ID_WIDTH-1:0]   r_id_reg;
    reg [ADDR_WIDTH-1:0] r_addr_reg;
    reg [7:0]            r_len_reg;
    reg [2:0]            r_size_reg;
    reg [2:0]            r_prot_reg;
    reg [7:0]            r_beat_cnt;

    // Arbiter Signals
    reg  req_w;
    wire ack_w;
    reg  req_r;
    wire ack_r;
    reg  [1:0] apb_state;
    reg  apb_grant_read; // 1: Đang phục vụ Read, 0: Đang phục vụ Write

    // ==========================================
    // 1. WRITE FSM (Quản lý AW, W và B Channels)
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state       <= W_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            s_axi_bid     <= {ID_WIDTH{1'b0}};
            req_w         <= 1'b0;
            w_beat_cnt    <= 8'd0;
            w_err_accum   <= 2'b00;
        end else begin
            // Xóa cờ bvalid khi Master đã nhận response
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            case (w_state)
                W_IDLE: begin
                    s_axi_awready <= 1'b1;
                    if (s_axi_awvalid && s_axi_awready) begin
                        s_axi_awready <= 1'b0;
                        w_id_reg      <= s_axi_awid;
                        w_addr_reg    <= s_axi_awaddr;
                        w_len_reg     <= s_axi_awlen;
                        w_size_reg    <= s_axi_awsize;
                        w_prot_reg    <= s_axi_awprot;
                        w_beat_cnt    <= 8'd0;
                        w_err_accum   <= 2'b00;
                        w_state       <= W_DATA;
                    end
                end

                W_DATA: begin
                    s_axi_wready <= 1'b1;
                    if (s_axi_wvalid && s_axi_wready) begin
                        s_axi_wready <= 1'b0;
                        w_data_reg   <= s_axi_wdata;
                        w_strb_reg   <= s_axi_wstrb;
                        req_w        <= 1'b1; // Yêu cầu Arbiter đẩy ra APB
                        w_state      <= W_ARB_REQ;
                    end
                end

                W_ARB_REQ: begin
                    if (ack_w) begin
                        req_w <= 1'b0;
                        // Ghi nhận lỗi từ APB nếu có
                        if (m_apb_pslverr) w_err_accum <= 2'b10;
                        
                        if (w_beat_cnt == w_len_reg) begin
                            // Đã xong burst
                            w_state <= W_RESP;
                        end else begin
                            // Tiếp tục beat tiếp theo
                            w_beat_cnt <= w_beat_cnt + 1'b1;
                            w_addr_reg <= w_addr_reg + (1 << w_size_reg);
                            w_state    <= W_DATA;
                        end
                    end
                end

                W_RESP: begin
                    if (!s_axi_bvalid) begin
                        s_axi_bvalid <= 1'b1;
                        s_axi_bid    <= w_id_reg;
                        s_axi_bresp  <= w_err_accum; // Trả về lỗi nếu có bất kỳ beat nào bị lỗi
                        w_state      <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // ==========================================
    // 2. READ FSM (Quản lý AR và R Channels)
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state       <= R_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rlast   <= 1'b0;
            s_axi_rid     <= {ID_WIDTH{1'b0}};
            s_axi_rdata   <= {DATA_WIDTH{1'b0}};
            req_r         <= 1'b0;
            r_beat_cnt    <= 8'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_axi_arready <= 1'b0;
                        r_id_reg      <= s_axi_arid;
                        r_addr_reg    <= s_axi_araddr;
                        r_len_reg     <= s_axi_arlen;
                        r_size_reg    <= s_axi_arsize;
                        r_prot_reg    <= s_axi_arprot;
                        r_beat_cnt    <= 8'd0;
                        req_r         <= 1'b1; // Yêu cầu Arbiter đẩy ra APB
                        r_state       <= R_ARB_REQ;
                    end
                end

                R_ARB_REQ: begin
                    if (ack_r) begin
                        req_r        <= 1'b0;
                        s_axi_rvalid <= 1'b1;
                        s_axi_rid    <= r_id_reg;
                        s_axi_rdata  <= m_apb_prdata;
                        s_axi_rresp  <= m_apb_pslverr ? 2'b10 : 2'b00;
                        s_axi_rlast  <= (r_beat_cnt == r_len_reg);
                        r_state      <= R_DATA;
                    end
                end

                R_DATA: begin
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        s_axi_rlast  <= 1'b0;
                        if (r_beat_cnt == r_len_reg) begin
                            // Đã xong burst
                            r_state <= R_IDLE;
                        end else begin
                            // Tiếp tục beat tiếp theo
                            r_beat_cnt <= r_beat_cnt + 1'b1;
                            r_addr_reg <= r_addr_reg + (1 << r_size_reg);
                            req_r      <= 1'b1;
                            r_state    <= R_ARB_REQ;
                        end
                    end
                end
            endcase
        end
    end

    // ==========================================
    // 3. APB ARBITER FSM (Định tuyến ra APB Bus)
    // ==========================================
    assign ack_w = (apb_state == APB_ACCESS) && m_apb_pready && !apb_grant_read;
    assign ack_r = (apb_state == APB_ACCESS) && m_apb_pready &&  apb_grant_read;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apb_state      <= APB_IDLE;
            m_apb_psel     <= 1'b0;
            m_apb_penable  <= 1'b0;
            m_apb_pwrite   <= 1'b0;
            m_apb_paddr    <= {ADDR_WIDTH{1'b0}};
            m_apb_pwdata   <= {DATA_WIDTH{1'b0}};
            m_apb_pstrb    <= {(DATA_WIDTH/8){1'b0}};
            m_apb_pprot    <= 3'd0;
            apb_grant_read <= 1'b0;
        end else begin
            case (apb_state)
                APB_IDLE: begin
                    m_apb_penable <= 1'b0;
                    m_apb_psel    <= 1'b0;
                    // Mức ưu tiên: Read > Write (tránh nghẽn kênh CPU fetch lệnh)
                    if (req_r) begin
                        m_apb_psel     <= 1'b1;
                        m_apb_pwrite   <= 1'b0;
                        m_apb_paddr    <= r_addr_reg;
                        m_apb_pprot    <= r_prot_reg;
                        m_apb_pstrb    <= {(DATA_WIDTH/8){1'b0}}; // Đọc không cần STRB
                        apb_grant_read <= 1'b1;
                        apb_state      <= APB_SETUP;
                    end 
                    else if (req_w) begin
                        m_apb_psel     <= 1'b1;
                        m_apb_pwrite   <= 1'b1;
                        m_apb_paddr    <= w_addr_reg;
                        m_apb_pwdata   <= w_data_reg;
                        m_apb_pstrb    <= w_strb_reg;
                        m_apb_pprot    <= w_prot_reg;
                        apb_grant_read <= 1'b0;
                        apb_state      <= APB_SETUP;
                    end
                end

                APB_SETUP: begin
                    m_apb_penable <= 1'b1;
                    apb_state     <= APB_ACCESS;
                end

                APB_ACCESS: begin
                    if (m_apb_pready) begin
                        m_apb_psel    <= 1'b0;
                        m_apb_penable <= 1'b0;
                        apb_state     <= APB_IDLE;
                    end
                end
            endcase
        end
    end

endmodule