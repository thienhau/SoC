`timescale 1ns / 1ps

module rv_debug_module (
    input  wire clk_sys,
    input  wire rst_sys_n,

    // Giao tiếp với DTM
    input  wire        dmi_req_valid,
    input  wire [6:0]  dmi_req_addr,
    input  wire [31:0] dmi_req_data,
    input  wire [1:0]  dmi_req_op,
    output reg         dmi_resp_ready,
    output reg         dmi_resp_valid,
    output reg  [31:0] dmi_resp_data,
    output reg  [1:0]  dmi_resp_op,

    // Giao tiếp với DTM AXI Master
    output reg         axi_req,
    output reg  [1:0]  axi_op,      
    output reg  [1:0]  axi_size,    // 0: 8-bit, 1: 16-bit, 2: 32-bit
    output reg  [31:0] axi_addr,
    output reg  [31:0] axi_wdata,
    input  wire        axi_ack,
    input  wire [31:0] axi_rdata,
    input  wire [1:0]  axi_resp
);

    localparam DMCONTROL  = 7'h10;
    localparam DMSTATUS   = 7'h11;
    localparam ABSTRACTCS = 7'h16;
    localparam SBCS       = 7'h38;
    localparam SBADDRESS0 = 7'h39;
    localparam SBDATA0    = 7'h3C;

    reg [31:0] sbaddress0;
    reg [31:0] sbdata0;

    reg dmactive;
    reg halted_fake; // Đánh lừa OpenOCD rằng chip đang Halt nếu có lệnh

    reg [2:0] sbaccess;
    reg       sbautoincrement;
    reg       sbreadondata;
    reg [2:0] sberror;

    wire [31:0] sbcs_val = {
        3'b001,          // [31:29] sbversion = 1
        7'b0,            // [28:22] reserved
        1'b0,            // [21] sbbusy = 0
        1'b0,            // [20] sbreadonaddr = 0
        sbaccess,        // [19:17] sbaccess
        sbautoincrement, // [16] sbautoincrement
        sbreadondata,    // [15] sbreadondata
        sberror,         // [14:12] sberror
        7'd32,           // [11:5] sbasize = 32 bit
        2'b0,            // [4:3] 128/64-bit access support = 0
        1'b1,            // [2] 32-bit access support = 1
        1'b1,            // [1] 16-bit support = 1
        1'b1             // [0] 8-bit support = 1
    };

    reg [1:0] state;
    localparam IDLE=0, WAIT_AXI=1, RESP=2;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            state <= IDLE;
            dmi_resp_ready <= 1'b1;
            dmi_resp_valid <= 1'b0;
            dmi_resp_op    <= 2'b00;
            dmi_resp_data  <= 32'b0;
            axi_req <= 1'b0;
            dmactive <= 1'b0;
            halted_fake <= 1'b0;
            sbaccess <= 3'd2; 
            sbautoincrement <= 1'b0;
            sbreadondata <= 1'b0;
            sberror <= 3'd0;
            sbaddress0 <= 32'b0;
        end else begin
            case (state)
                IDLE: begin
                    dmi_resp_valid <= 1'b0;
                    if (dmi_req_valid && dmi_resp_ready) begin
                        dmi_resp_ready <= 1'b0;
                        dmi_resp_op <= 2'b00; 
                        
                        if (dmi_req_op == 2'd2) begin // WRITE
                            case (dmi_req_addr)
                                DMCONTROL: begin
                                    dmactive <= dmi_req_data[0];
                                    if (dmi_req_data[31]) halted_fake <= 1'b1; // haltreq
                                    else if (dmi_req_data[30]) halted_fake <= 1'b0; // resumereq
                                    state <= RESP;
                                end
                                SBCS: begin
                                    sbaccess <= dmi_req_data[19:17];
                                    sbautoincrement <= dmi_req_data[16];
                                    sbreadondata <= dmi_req_data[15];
                                    if (dmi_req_data[14]) sberror[2] <= 1'b0;
                                    if (dmi_req_data[13]) sberror[1] <= 1'b0;
                                    if (dmi_req_data[12]) sberror[0] <= 1'b0;
                                    state <= RESP;
                                end
                                SBADDRESS0: begin 
                                    sbaddress0 <= dmi_req_data;
                                    state <= RESP; 
                                end
                                SBDATA0: begin
                                    sbdata0 <= dmi_req_data;
                                    if (sberror == 0) begin
                                        axi_addr <= sbaddress0;
                                        axi_wdata <= dmi_req_data;
                                        axi_size <= sbaccess[1:0];
                                        axi_op <= 2'd2; // Write
                                        axi_req <= 1'b1;
                                        state <= WAIT_AXI;
                                    end else begin
                                        state <= RESP;
                                    end
                                end
                                default: state <= RESP;
                            endcase
                        end else if (dmi_req_op == 2'd1) begin // READ
                            case (dmi_req_addr)
                                DMCONTROL: begin
                                    dmi_resp_data <= {31'b0, dmactive};
                                    state <= RESP;
                                end
                                DMSTATUS: begin
                                    if (dmactive) begin
                                        if (halted_fake) dmi_resp_data <= 32'h003A0382; // allhalted & anyhalted
                                        else             dmi_resp_data <= 32'h003A0C82; // allrunning & anyrunning
                                    end else begin
                                        dmi_resp_data <= 32'h00000002; // version 0.13
                                    end
                                    state <= RESP;
                                end
                                ABSTRACTCS: begin
                                    dmi_resp_data <= 32'h0; // Không hỗ trợ abstract command
                                    state <= RESP;
                                end
                                SBCS: begin
                                    dmi_resp_data <= sbcs_val;
                                    state <= RESP;
                                end
                                SBADDRESS0: begin
                                    dmi_resp_data <= sbaddress0;
                                    state <= RESP;
                                end
                                SBDATA0: begin
                                    if (sberror == 0) begin
                                        axi_addr <= sbaddress0;
                                        axi_size <= sbaccess[1:0];
                                        axi_op <= 2'd1; // Read
                                        axi_req <= 1'b1;
                                        state <= WAIT_AXI;
                                    end else begin
                                        dmi_resp_data <= 32'h0;
                                        state <= RESP;
                                    end
                                end
                                default: begin
                                    dmi_resp_data <= 32'h0;
                                    state <= RESP;
                                end
                            endcase
                        end
                    end
                end

                WAIT_AXI: begin
                    if (axi_ack) begin
                        axi_req <= 1'b0;
                        if (axi_op == 2'd1) dmi_resp_data <= axi_rdata;
                        
                        if (axi_resp != 0) begin
                            sberror <= 3'd2; // AXI Lỗi
                        end else if (sbautoincrement) begin
                            sbaddress0 <= sbaddress0 + (1 << sbaccess);
                        end
                        state <= RESP;
                    end
                end

                RESP: begin
                    dmi_resp_valid <= 1'b1;
                    if (!dmi_req_valid) begin 
                        dmi_resp_ready <= 1'b1;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule