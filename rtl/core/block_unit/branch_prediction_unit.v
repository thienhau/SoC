module branch_prediction_unit (
    input clk, reset_n,
    input [31:0] pc_in, ex_mem_pc_in,
    input ex_mem_branch, ex_mem_branch_taken, ex_mem_predict_taken, ex_mem_btb_hit,
    input [31:0] ex_mem_branch_target,
    output bpu_correct, predict_taken, btb_hit, actual_taken,
    output [31:0] predict_target
);
    assign actual_taken = ex_mem_branch && ex_mem_branch_taken;
    assign bpu_correct = (ex_mem_predict_taken == actual_taken);
    wire [1:0] update_btb = ((!ex_mem_btb_hit && ex_mem_branch && actual_taken) || 
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
    input clk, reset_n,
    input [31:0] pc_in, ex_mem_pc_in,
    input [1:0] update_btb,
    input [31:0] actual_target,
    output [31:0] predict_target,
    output btb_hit
);
    parameter ENTRY = 256;
    parameter INDEX = 8;
    parameter TAG = 22;
    parameter TARGET_ADDR = 30;

    wire [29:0] address = ex_mem_pc_in[31:2];
    wire [TAG-1:0] tag = address[29:8];
    wire [INDEX-1:0] index = address[7:0];
    
    reg [TAG-1:0] tags [0:ENTRY-1];
    reg [TARGET_ADDR-1:0] targets [0:ENTRY-1];
    reg valids [0:ENTRY-1];

    assign btb_hit = valids[pc_in[9:2]] && (tags[pc_in[9:2]] == tag);
    assign predict_target = btb_hit ? {targets[pc_in[9:2]], 2'b00} : (pc_in + 4);
    
    integer i;
    always @(posedge clk) begin
        if (!reset_n) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                valids[i] <= 1'b0;
                tags[i] <= 8'b0;
                targets[i] <= 30'b0;
            end
        end else begin
            if (update_btb == 2'b01) begin // New or update entry
                tags[index] <= tag;
                targets[index] <= actual_target[31:2];
                valids[index] <= 1'b1;
            end else if (update_btb == 2'b10) begin // Clear entry
                tags[index] <= 0;
                targets[index] <= 0;
                valids[index] <= 0;
            end
        end
    end
endmodule

module branch_history_table (
    input clk, reset_n,
    input [31:0] pc_in, ex_mem_pc_in,
    input update_bht,
    input btb_hit,
    input actual_taken,
    output predict_taken
);
    parameter ENTRY = 64;
    parameter INDEX = 6;
    parameter PREDICT = 2;

    wire [5:0] address = ex_mem_pc_in[7:2];
    wire [INDEX-1:0] index = address[5:0];
    
    reg [PREDICT-1:0] predicts [0:ENTRY-1];

    assign predict_taken = !btb_hit ? 1'b0 : (predicts[pc_in[7:2]] == 2'b10 || predicts[pc_in[7:2]] == 2'b11) ? 1'b1 : 1'b0;

    reg [INDEX-1:0] prev_index = 0;
    reg prev_update_bht = 0;
    reg prev_actual_taken = 0;
    
    integer i;
    always @(posedge clk) begin
        if (!reset_n) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                predicts[i] <= 2'b10; // Reset to weakly taken
            end 
        end else if (update_bht) begin
            if (index == prev_index && update_bht == prev_update_bht && actual_taken == prev_actual_taken) begin

            end else begin
                if (actual_taken) begin
                    if (predicts[index] != 2'b11) begin
                        predicts[index] <= predicts[index] + 1;
                    end
                end else if (!actual_taken) begin
                    if (predicts[index] != 2'b00) begin
                        predicts[index] <= predicts[index] - 1;
                    end
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!reset_n) begin
            prev_index <= 0;
            prev_update_bht <= 0;
            prev_actual_taken <= 0;
        end else begin
            prev_index <= index;
            prev_update_bht <= update_bht;
            prev_actual_taken <= actual_taken;
        end
    end
endmodule