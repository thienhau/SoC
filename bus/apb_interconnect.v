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

    // GIẢI MÃ ĐỊA CHỈ: Kiểm tra vùng 0x5xxx và phân dải bằng bit [11:8]
    // Vì Bridge chỉ vứt địa chỉ vào đây, ta biết chắc nó đã nằm trong 0x5000-0x7FFF
    // Ta lấy paddr[11:8] làm định tuyến.
    wire dec_syscon = (m_paddr[11:8] == 4'h0); // 0x50xx
    wire dec_plic   = (m_paddr[11:8] == 4'h1); // 0x51xx
    wire dec_timer  = (m_paddr[11:8] == 4'h2); // 0x52xx
    wire dec_uart   = (m_paddr[11:8] == 4'h3); // 0x53xx
    wire dec_spi    = (m_paddr[11:8] == 4'h4); // 0x54xx
    wire dec_i2c    = (m_paddr[11:8] == 4'h5); // 0x55xx
    wire dec_gpio   = (m_paddr[11:8] == 4'h6); // 0x56xx
    wire dec_accel  = (m_paddr[11:8] == 4'h7); // 0x57xx

    assign s0_psel = m_psel && dec_syscon;
    assign s1_psel = m_psel && dec_plic;
    assign s2_psel = m_psel && dec_timer;
    assign s3_psel = m_psel && dec_uart;
    assign s4_psel = m_psel && dec_spi;
    assign s5_psel = m_psel && dec_i2c;
    assign s6_psel = m_psel && dec_gpio;
    assign s7_psel = m_psel && dec_accel;

    always @(*) begin
        if      (dec_syscon) {m_prdata, m_pready, m_pslverr} = {s0_prdata, s0_pready, s0_pslverr};
        else if (dec_plic)   {m_prdata, m_pready, m_pslverr} = {s1_prdata, s1_pready, s1_pslverr};
        else if (dec_timer)  {m_prdata, m_pready, m_pslverr} = {s2_prdata, s2_pready, s2_pslverr};
        else if (dec_uart)   {m_prdata, m_pready, m_pslverr} = {s3_prdata, s3_pready, s3_pslverr};
        else if (dec_spi)    {m_prdata, m_pready, m_pslverr} = {s4_prdata, s4_pready, s4_pslverr};
        else if (dec_i2c)    {m_prdata, m_pready, m_pslverr} = {s5_prdata, s5_pready, s5_pslverr};
        else if (dec_gpio)   {m_prdata, m_pready, m_pslverr} = {s6_prdata, s6_pready, s6_pslverr};
        else if (dec_accel)  {m_prdata, m_pready, m_pslverr} = {s7_prdata, s7_pready, s7_pslverr};
        else                 {m_prdata, m_pready, m_pslverr} = {32'hDEADBEEF, 1'b1, m_psel}; // Báo lỗi nếu địa chỉ không hợp lệ
    end
endmodule