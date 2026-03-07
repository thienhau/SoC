`timescale 1ns / 1ps

module apb_interconnect #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    // --- Interface từ Master (AXI to APB Bridge) ---
    input  wire [ADDR_WIDTH-1:0]  m_paddr,
    input  wire [2:0]             m_pprot,
    input  wire                   m_psel,
    input  wire                   m_penable,
    input  wire                   m_pwrite,
    input  wire [DATA_WIDTH-1:0]  m_pwdata,
    input  wire [3:0]             m_pstrb,
    output reg  [DATA_WIDTH-1:0]  m_prdata,
    output reg                    m_pready,
    output reg                    m_pslverr,

    // --- Broadcast chung cho tất cả Slaves ---
    output wire [ADDR_WIDTH-1:0]  s_paddr,
    output wire [2:0]             s_pprot,
    output wire                   s_penable,
    output wire                   s_pwrite,
    output wire [DATA_WIDTH-1:0]  s_pwdata,
    output wire [3:0]             s_pstrb,

    // --- S0: SYSCON (Reset Vector) @ 0x4000 ---
    output wire                   s0_psel,
    input  wire [DATA_WIDTH-1:0]  s0_prdata,
    input  wire                   s0_pready,
    input  wire                   s0_pslverr,

    // --- S1: GPIO @ 0x4100 ---
    output wire                   s1_psel,
    input  wire [DATA_WIDTH-1:0]  s1_prdata,
    input  wire                   s1_pready,
    input  wire                   s1_pslverr,
    
    // --- S2: PLIC / INT CTRL @ 0x4200 ---
    output wire                   s2_psel,
    input  wire [DATA_WIDTH-1:0]  s2_prdata,
    input  wire                   s2_pready,
    input  wire                   s2_pslverr
);

    // Broadcast tín hiệu
    assign s_paddr   = m_paddr;
    assign s_pprot   = m_pprot;
    assign s_penable = m_penable;
    assign s_pwrite  = m_pwrite;
    assign s_pwdata  = m_pwdata;
    assign s_pstrb   = m_pstrb;

    // Giải mã địa chỉ dựa trên bit [11:8]
    wire dec_syscon = (m_paddr[11:8] == 4'h0); // 0x40xx
    wire dec_gpio   = (m_paddr[11:8] == 4'h1); // 0x41xx
    wire dec_plic   = (m_paddr[11:8] == 4'h2); // 0x42xx
    wire dec_err    = !(dec_syscon || dec_gpio || dec_plic);

    // Kích hoạt PSEL
    assign s0_psel = m_psel && dec_syscon;
    assign s1_psel = m_psel && dec_gpio;
    assign s2_psel = m_psel && dec_plic;

    // Mux phản hồi về Master
    always @(*) begin
        if (dec_syscon) begin
            m_prdata  = s0_prdata;
            m_pready  = s0_pready;
            m_pslverr = s0_pslverr;
        end else if (dec_gpio) begin
            m_prdata  = s1_prdata;
            m_pready  = s1_pready;
            m_pslverr = s1_pslverr;
        end else if (dec_plic) begin
            m_prdata  = s2_prdata;
            m_pready  = s2_pready;
            m_pslverr = s2_pslverr;
        end else begin
            // Trả về lỗi nếu cố truy cập ngoại vi không tồn tại
            m_prdata  = 32'hDEADBEEF;
            m_pready  = 1'b1;
            m_pslverr = m_psel; // Chỉ báo lỗi khi có Select
        end
    end

endmodule