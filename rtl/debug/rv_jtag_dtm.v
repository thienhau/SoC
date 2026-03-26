`timescale 1ns / 1ps

module rv_jtag_dtm #(
    parameter ABITS = 7
)(
    input  wire             tck,
    input  wire             trst_n,
    input  wire             tms,
    input  wire             tdi,
    output reg              tdo,

    // Giao tiếp DMI với Debug Module 
    output reg              dmi_req_valid,
    output reg  [ABITS-1:0] dmi_req_addr,
    output reg  [31:0]      dmi_req_data,
    output reg  [1:0]       dmi_req_op,     // 1: Read, 2: Write
    input  wire             dmi_resp_ready, 
    input  wire             dmi_resp_valid,
    input  wire [31:0]      dmi_resp_data,
    input  wire [1:0]       dmi_resp_op     
);

    localparam TLR=4'h0, RTI=4'h1, SDS=4'h2, CDR=4'h3, SDR=4'h4, E1D=4'h5, PDR=4'h6, E2D=4'h7, UDR=4'h8,
               SIS=4'h9, CIR=4'hA, SIR=4'hB, E1I=4'hC, PIR=4'hD, E2I=4'hE, UIR=4'hF;

    localparam IR_IDCODE = 5'h01;
    localparam IR_DTMCS  = 5'h10;
    localparam IR_DMI    = 5'h11;
    localparam IR_BYPASS = 5'h1F;

    reg [3:0] state, next_state;
    reg [4:0] ir;
    reg [31:0] idcode = 32'h10e31913; 
    wire [31:0] dtmcs = 32'h00000071;

    reg [40:0] dmi_shift_reg;
    reg [31:0] dr_shift_reg;
    reg [4:0]  ir_shift_reg;

    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) state <= TLR;
        else         state <= next_state;
    end

    always @(*) begin
        case (state)
            TLR: next_state = tms ? TLR : RTI;
            RTI: next_state = tms ? SDS : RTI;
            SDS: next_state = tms ? SIS : CDR;
            CDR: next_state = tms ? E1D : SDR;
            SDR: next_state = tms ? E1D : SDR;
            E1D: next_state = tms ? UDR : PDR;
            PDR: next_state = tms ? E2D : PDR;
            E2D: next_state = tms ? UDR : SDR;
            UDR: next_state = tms ? SDS : RTI;
            SIS: next_state = tms ? TLR : CIR;
            CIR: next_state = tms ? E1I : SIR;
            SIR: next_state = tms ? E1I : SIR;
            E1I: next_state = tms ? UIR : PIR;
            PIR: next_state = tms ? E2I : PIR;
            E2I: next_state = tms ? UIR : SIR;
            UIR: next_state = tms ? SDS : RTI;
            default: next_state = TLR;
        endcase
    end

    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            ir <= IR_IDCODE;
            dmi_req_valid <= 0;
            dmi_shift_reg <= 0;
            dr_shift_reg  <= 0;
            ir_shift_reg  <= 5'b00001;
        end else begin
            if (state == TLR) begin
                ir <= IR_IDCODE;
                dmi_req_valid <= 1'b0; 
                
            end else if (state == CIR) begin
                ir_shift_reg <= 5'b00001;
            end else if (state == SIR) begin
                ir_shift_reg <= {tdi, ir_shift_reg[4:1]};
            end else if (state == UIR) begin
                ir <= ir_shift_reg;

            end else if (state == CDR) begin
                case (ir)
                    IR_IDCODE: dr_shift_reg <= idcode;
                    IR_DTMCS:  dr_shift_reg <= dtmcs;
                    IR_DMI: begin
                        dmi_shift_reg <= {dmi_req_addr, dmi_resp_data, dmi_resp_valid ? dmi_resp_op : 2'b11};
                        if (dmi_resp_valid) begin
                            dmi_req_valid <= 1'b0;
                        end
                    end
                    default:   dr_shift_reg <= 32'b0;
                endcase
            end else if (state == SDR) begin
                if (ir == IR_DMI) 
                    dmi_shift_reg <= {tdi, dmi_shift_reg[40:1]};
                else 
                    dr_shift_reg <= {tdi, dr_shift_reg[31:1]};
            end else if (state == UDR) begin
                if (ir == IR_DMI && dmi_shift_reg[1:0] != 2'b00) begin
                    dmi_req_op    <= dmi_shift_reg[1:0];
                    dmi_req_data  <= dmi_shift_reg[33:2];
                    dmi_req_addr  <= dmi_shift_reg[40:34];
                    dmi_req_valid <= 1'b1;
                end
            end
        end
    end

    always @(negedge tck) begin
        if (state == SIR) begin
            tdo <= ir_shift_reg[0];
        end else if (state == SDR) begin
            if (ir == IR_DMI) tdo <= dmi_shift_reg[0];
            else              tdo <= dr_shift_reg[0];
        end else begin
            tdo <= 1'b0;
        end
    end
endmodule