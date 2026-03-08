//==================================================================================================
// File: branch_prediction_unit.v
//==================================================================================================
module branch_prediction_unit (
    input clk,
    input reset_n,
    input [31:0] pc_in,
    input [31:0] ex_mem_pc_in,
    input ex_mem_branch,
    input ex_mem_branch_taken,
    input ex_mem_predict_taken,
    input ex_mem_btb_hit,
    input [31:0] ex_mem_branch_target,
    output bpu_correct,
    output predict_taken,
    output btb_hit,
    output actual_taken,
    output [31:0] predict_target
);

    assign actual_taken = ex_mem_branch && ex_mem_branch_taken;
    assign bpu_correct = (ex_mem_predict_taken == actual_taken);
    
    wire [1:0] update_btb;
    
    assign update_btb = ((!ex_mem_btb_hit && ex_mem_branch && actual_taken) || 
                         (ex_mem_btb_hit && ex_mem_branch && !ex_mem_predict_taken && actual_taken)) ? 2'b01 :
                        ((ex_mem_btb_hit && !ex_mem_branch) ? 2'b10 : 2'b00);
    
    wire update_bht = ex_mem_branch;

    branch_target_buffer BTB (
        .clk(clk),
        .reset_n(reset_n),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .update_btb(update_btb), 
        .actual_target(ex_mem_branch_target),
        .predict_target(predict_target),
        .btb_hit(btb_hit)
    );

    branch_history_table BHT (
        .clk(clk), 
        .reset_n(reset_n),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .update_bht(update_bht),
        .btb_hit(btb_hit),
        .actual_taken(actual_taken),
        .predict_taken(predict_taken)
    );
    
endmodule


module branch_target_buffer (
    input clk,
    input reset_n,
    input [31:0] pc_in,
    input [31:0] ex_mem_pc_in,
    input [1:0] update_btb,
    input [31:0] actual_target,
    output [31:0] predict_target,
    output btb_hit
);

    parameter ENTRY = 32;
    parameter INDEX = 5;
    parameter TAG = 5;
    parameter TARGET_ADDR = 10;

    wire [TAG-1:0] tag = ex_mem_pc_in[31:27];
    wire [INDEX-1:0] index = ex_mem_pc_in[26:22];
    
    (* ram_style = "distributed" *) reg [TAG-1:0] tags [0:ENTRY-1];
    (* ram_style = "distributed" *) reg [TARGET_ADDR-1:0] targets [0:ENTRY-1];
    
    reg valids [0:ENTRY-1];

    wire [INDEX-1:0] read_idx = pc_in[26:22];
    
    assign btb_hit = valids[read_idx] && (tags[read_idx] == pc_in[31:27]);
    assign predict_target = btb_hit ? {targets[read_idx], 22'b0} : (pc_in + 32'd4);
    
    integer i;
    
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                valids[i] <= 1'b0;
            end
        end else begin
            if (update_btb == 2'b01) begin 
                tags[index] <= tag;
                targets[index] <= actual_target[31:22];
                valids[index] <= 1'b1;
            end else if (update_btb == 2'b10) begin 
                valids[index] <= 1'b0;
            end
        end
    end
    
endmodule


module branch_history_table (
    input clk,
    input reset_n,
    input [31:0] pc_in,
    input [31:0] ex_mem_pc_in,
    input update_bht,
    input btb_hit,
    input actual_taken,
    output predict_taken
);

    parameter ENTRY = 32;
    parameter INDEX = 5;

    reg [1:0] bht [0:ENTRY-1];
    
    wire [INDEX-1:0] read_idx = pc_in[26:22];
    wire [INDEX-1:0] write_idx = ex_mem_pc_in[26:22];
    
    reg [1:0] current_state;
    reg [1:0] next_state;
    
    assign predict_taken = (btb_hit && (current_state == 2'b11 || current_state == 2'b10));
    
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            current_state <= 2'b00;
        end else begin
            current_state <= bht[read_idx];
        end
    end
    
    always @(*) begin
        case (bht[write_idx])
            2'b00: next_state = actual_taken ? 2'b01 : 2'b00;
            2'b01: next_state = actual_taken ? 2'b11 : 2'b00;
            2'b10: next_state = actual_taken ? 2'b11 : 2'b00;
            2'b11: next_state = actual_taken ? 2'b11 : 2'b10;
            default: next_state = 2'b00;
        endcase
    end
    
    integer i;
    
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                bht[i] <= 2'b00;
            end
        end else if (update_bht) begin
            bht[write_idx] <= next_state;
        end
    end
    
endmodule