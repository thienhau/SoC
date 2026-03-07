module branch_prediction_unit (
    input clk, reset,
    input [11:0] pc_in, ex_mem_pc_in,
    input ex_mem_branch, ex_mem_branch_taken, ex_mem_predict_taken, ex_mem_btb_hit,
    input [11:0] ex_mem_branch_target,
    output bpu_correct, predict_taken, btb_hit, actual_taken,
    output [11:0] predict_target
);
    assign actual_taken = ex_mem_branch && ex_mem_branch_taken;
    assign bpu_correct = (ex_mem_predict_taken == actual_taken);
    wire [1:0] update_btb = ((!ex_mem_btb_hit && ex_mem_branch && actual_taken) || 
                            (ex_mem_btb_hit && ex_mem_branch && !ex_mem_predict_taken && actual_taken)) ? 2'b01 :
                            ((ex_mem_btb_hit && !ex_mem_branch) ? 2'b10 : 2'b00);
    wire update_bht = ex_mem_branch;

    branch_target_buffer BTB (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .update_btb(update_btb), 
        .actual_target(ex_mem_branch_target),
        .predict_target(predict_target),
        .btb_hit(btb_hit)
    );

    branch_history_table BHT (
        .clk(clk), 
        .reset(reset),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .update_bht(update_bht),
        .btb_hit(btb_hit),
        .actual_taken(actual_taken),
        .predict_taken(predict_taken)
    );
endmodule

module branch_target_buffer (
    input clk, reset,
    input [11:0] pc_in, ex_mem_pc_in,
    input [1:0] update_btb,
    input [11:0] actual_target,
    output [11:0] predict_target,
    output btb_hit
);
    parameter ENTRY = 32;
    parameter INDEX = 5;
    parameter TAG = 5;
    parameter TARGET_ADDR = 10;

    wire [TAG-1:0] tag = ex_mem_pc_in[11:7]; // Chỉnh lại bit để khớp INDEX
    wire [INDEX-1:0] index = ex_mem_pc_in[6:2];
    
    // Ép vào LUTRAM (Không Reset)
    (* ram_style = "distributed" *) reg [TAG-1:0] tags [0:ENTRY-1];
    (* ram_style = "distributed" *) reg [TARGET_ADDR-1:0] targets [0:ENTRY-1];
    
    // Dùng Flip-Flops (Có Reset) để quản lý trạng thái
    reg valids [0:ENTRY-1];

    // Read logic
    wire [INDEX-1:0] read_idx = pc_in[6:2];
    assign btb_hit = valids[read_idx] && (tags[read_idx] == pc_in[11:7]);
    assign predict_target = btb_hit ? {targets[read_idx], 2'b00} : (pc_in + 4);
    
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            // Chỉ reset bảng valids (Tốn 32 FFs)
            for (i = 0; i < ENTRY; i = i + 1) valids[i] <= 1'b0;
        end else begin
            if (update_btb == 2'b01) begin 
                tags[index] <= tag;
                targets[index] <= actual_target[11:2];
                valids[index] <= 1'b1;
            end else if (update_btb == 2'b10) begin 
                valids[index] <= 1'b0; // Chỉ cần xóa valid bit
            end
        end
    end
endmodule