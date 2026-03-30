`timescale 1ns / 1ps

module apb_interconnect #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    
    // Cấu hình Memory Map (Base Address & Size Mask)
    // Mask = ~(Size - 1). Ví dụ: Kích thước 4KB (0x1000) -> Mask = 32'hFFFF_F000
    parameter SLV0_BASE = 32'h4000_0000,
    parameter SLV0_MASK = 32'hFFFF_F000, // 4KB

    parameter SLV1_BASE = 32'h4000_1000,
    parameter SLV1_MASK = 32'hFFFF_F000, // 4KB

    parameter SLV2_BASE = 32'h4000_2000,
    parameter SLV2_MASK = 32'hFFFF_E000, // 8KB

    parameter SLV3_BASE = 32'h5000_0000,
    parameter SLV3_MASK = 32'hFFFF_0000  // 64KB
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ==========================================
    // APB4 MASTER PORT (Tới từ Bridge)
    // ==========================================
    input  wire [ADDR_WIDTH-1:0]    m_paddr,
    input  wire [2:0]               m_pprot,
    input  wire                     m_psel,
    input  wire                     m_penable,
    input  wire                     m_pwrite,
    input  wire [DATA_WIDTH-1:0]    m_pwdata,
    input  wire [(DATA_WIDTH/8)-1:0]m_pstrb,
    output reg                      m_pready,
    output reg  [DATA_WIDTH-1:0]    m_prdata,
    output reg                      m_pslverr,

    // ==========================================
    // APB4 SLAVE 0 PORT
    // ==========================================
    output wire [ADDR_WIDTH-1:0]    s0_paddr,
    output wire [2:0]               s0_pprot,
    output wire                     s0_psel,
    output wire                     s0_penable,
    output wire                     s0_pwrite,
    output wire [DATA_WIDTH-1:0]    s0_pwdata,
    output wire [(DATA_WIDTH/8)-1:0]s0_pstrb,
    input  wire                     s0_pready,
    input  wire [DATA_WIDTH-1:0]    s0_prdata,
    input  wire                     s0_pslverr,

    // ==========================================
    // APB4 SLAVE 1 PORT
    // ==========================================
    output wire [ADDR_WIDTH-1:0]    s1_paddr,
    output wire [2:0]               s1_pprot,
    output wire                     s1_psel,
    output wire                     s1_penable,
    output wire                     s1_pwrite,
    output wire [DATA_WIDTH-1:0]    s1_pwdata,
    output wire [(DATA_WIDTH/8)-1:0]s1_pstrb,
    input  wire                     s1_pready,
    input  wire [DATA_WIDTH-1:0]    s1_prdata,
    input  wire                     s1_pslverr,

    // ==========================================
    // APB4 SLAVE 2 PORT
    // ==========================================
    output wire [ADDR_WIDTH-1:0]    s2_paddr,
    output wire [2:0]               s2_pprot,
    output wire                     s2_psel,
    output wire                     s2_penable,
    output wire                     s2_pwrite,
    output wire [DATA_WIDTH-1:0]    s2_pwdata,
    output wire [(DATA_WIDTH/8)-1:0]s2_pstrb,
    input  wire                     s2_pready,
    input  wire [DATA_WIDTH-1:0]    s2_prdata,
    input  wire                     s2_pslverr,

    // ==========================================
    // APB4 SLAVE 3 PORT
    // ==========================================
    output wire [ADDR_WIDTH-1:0]    s3_paddr,
    output wire [2:0]               s3_pprot,
    output wire                     s3_psel,
    output wire                     s3_penable,
    output wire                     s3_pwrite,
    output wire [DATA_WIDTH-1:0]    s3_pwdata,
    output wire [(DATA_WIDTH/8)-1:0]s3_pstrb,
    input  wire                     s3_pready,
    input  wire [DATA_WIDTH-1:0]    s3_prdata,
    input  wire                     s3_pslverr
);

    // ==========================================
    // LOGIC GIẢI MÃ ĐỊA CHỈ (ADDRESS DECODER)
    // ==========================================
    wire match_s0 = ((m_paddr & SLV0_MASK) == SLV0_BASE);
    wire match_s1 = ((m_paddr & SLV1_MASK) == SLV1_BASE);
    wire match_s2 = ((m_paddr & SLV2_MASK) == SLV2_BASE);
    wire match_s3 = ((m_paddr & SLV3_MASK) == SLV3_BASE);

    wire match_any = match_s0 | match_s1 | match_s2 | match_s3;

    // Truyền tín hiệu điều khiển chung xuống tất cả Slaves
    // PSEL của Slave nào được bật thì Slave đó mới phản hồi
    assign s0_paddr   = m_paddr;
    assign s0_pprot   = m_pprot;
    assign s0_penable = m_penable;
    assign s0_pwrite  = m_pwrite;
    assign s0_pwdata  = m_pwdata;
    assign s0_pstrb   = m_pstrb;
    assign s0_psel    = m_psel & match_s0;

    assign s1_paddr   = m_paddr;
    assign s1_pprot   = m_pprot;
    assign s1_penable = m_penable;
    assign s1_pwrite  = m_pwrite;
    assign s1_pwdata  = m_pwdata;
    assign s1_pstrb   = m_pstrb;
    assign s1_psel    = m_psel & match_s1;

    assign s2_paddr   = m_paddr;
    assign s2_pprot   = m_pprot;
    assign s2_penable = m_penable;
    assign s2_pwrite  = m_pwrite;
    assign s2_pwdata  = m_pwdata;
    assign s2_pstrb   = m_pstrb;
    assign s2_psel    = m_psel & match_s2;

    assign s3_paddr   = m_paddr;
    assign s3_pprot   = m_pprot;
    assign s3_penable = m_penable;
    assign s3_pwrite  = m_pwrite;
    assign s3_pwdata  = m_pwdata;
    assign s3_pstrb   = m_pstrb;
    assign s3_psel    = m_psel & match_s3;

    // ==========================================
    // LOGIC DEFAULT SLAVE (CHỐNG TREO BUS)
    // ==========================================
    // Trạng thái cho Default Slave FSM
    reg def_slv_ready;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            def_slv_ready <= 1'b0;
        end else begin
            // Nếu có truy cập không hợp lệ và đã vào giai đoạn PENABLE
            if (m_psel && !match_any && m_penable && !def_slv_ready) begin
                def_slv_ready <= 1'b1; // Báo ready ở chu kỳ PENABLE
            end else begin
                def_slv_ready <= 1'b0;
            end
        end
    end

    // ==========================================
    // BỘ MUX TÍN HIỆU PHẢN HỒI VỀ MASTER
    // ==========================================
    always @(*) begin
        if (match_s0) begin
            m_prdata  = s0_prdata;
            m_pready  = s0_pready;
            m_pslverr = s0_pslverr;
        end 
        else if (match_s1) begin
            m_prdata  = s1_prdata;
            m_pready  = s1_pready;
            m_pslverr = s1_pslverr;
        end 
        else if (match_s2) begin
            m_prdata  = s2_prdata;
            m_pready  = s2_pready;
            m_pslverr = s2_pslverr;
        end 
        else if (match_s3) begin
            m_prdata  = s3_prdata;
            m_pready  = s3_pready;
            m_pslverr = s3_pslverr;
        end 
        else begin
            // Chọn Default Slave
            m_prdata  = 32'hDEADBEEF; // Dữ liệu rác dễ nhận diện khi debug
            m_pready  = def_slv_ready;
            m_pslverr = 1'b1;         // Luôn báo lỗi vì địa chỉ không tồn tại
        end
    end

endmodule