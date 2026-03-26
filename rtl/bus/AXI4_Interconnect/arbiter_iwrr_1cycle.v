module arbiter_iwrr_1cycle
#(
    parameter P_REQUESTER_NUM     = 4,
    parameter [0:(P_REQUESTER_NUM*32)-1] P_REQUESTER_WEIGHT = {32'd5, 32'd3, 32'd2, 32'd1},
    parameter P_NUM_GRANT_REQ_W   = 1
)
(
    input                               clk,
    input                               rst_n,
    input  [P_REQUESTER_NUM-1:0]        req_i,
    input  [0:(P_REQUESTER_NUM*32)-1]   req_weight_i,       // dynamic weights (unused, use static)
    input  [P_NUM_GRANT_REQ_W-1:0]      num_grant_req_i,    // unused
    input                               grant_ready_i,
    output [P_REQUESTER_NUM-1:0]        grant_valid_o
);
    localparam REQ_W    = P_REQUESTER_NUM;
    localparam WEIGHT_W = 32;
    localparam IDX_W    = (REQ_W > 1) ? $clog2(REQ_W) : 1;

    // Weight lookup
    function [WEIGHT_W-1:0] get_weight;
        input integer idx;
        begin
            get_weight = P_REQUESTER_WEIGHT[idx * 32 +: 32];
        end
    endfunction

    // Internal signals
    reg  [IDX_W-1:0]      last_grant_idx;
    reg  [WEIGHT_W-1:0]   credit [0:REQ_W-1];
    wire                   grant_handshake;

    integer i;

    // Round-robin with credit-based weighting
    reg [REQ_W-1:0] grant_candidate;
    reg found;
    reg [IDX_W:0]   idx_sum;
    reg [IDX_W-1:0] idx_wrapped;

    always @(*) begin
        grant_candidate = {REQ_W{1'b0}};
        found = 1'b0;

        // Phase 1: Search from (last_grant_idx+1), prefer requester with remaining credit
        for (i = 0; i < REQ_W; i = i + 1) begin
            if (!found) begin
                idx_sum = {1'b0, last_grant_idx} + 1 + i;
                idx_wrapped = (idx_sum >= REQ_W) ? (idx_sum - REQ_W) : idx_sum[IDX_W-1:0];
                if (req_i[idx_wrapped] && (credit[idx_wrapped] > 0)) begin
                    grant_candidate[idx_wrapped] = 1'b1;
                    found = 1'b1;
                end
            end
        end

        // Phase 2: If no one with credit, pick any active requester (triggers refill)
        if (!found) begin
            for (i = 0; i < REQ_W; i = i + 1) begin
                if (!found) begin
                    idx_sum = {1'b0, last_grant_idx} + 1 + i;
                    idx_wrapped = (idx_sum >= REQ_W) ? (idx_sum - REQ_W) : idx_sum[IDX_W-1:0];
                    if (req_i[idx_wrapped]) begin
                        grant_candidate[idx_wrapped] = 1'b1;
                        found = 1'b1;
                    end
                end
            end
        end
    end

    assign grant_valid_o = (|req_i) ? grant_candidate : {REQ_W{1'b0}};
    assign grant_handshake = (|grant_valid_o) & grant_ready_i;

    // One-hot to binary for indexing
    reg [IDX_W-1:0] grant_idx;
    always @(*) begin
        grant_idx = {IDX_W{1'b0}};
        for (i = 0; i < REQ_W; i = i + 1) begin
            if (grant_candidate[i])
                grant_idx = i[IDX_W-1:0];
        end
    end

    // Sequential: update last_grant_idx and credits
    always @(posedge clk) begin
        if (!rst_n) begin
            last_grant_idx <= {IDX_W{1'b0}};
            for (i = 0; i < REQ_W; i = i + 1)
                credit[i] <= get_weight(i);
        end else if (grant_handshake) begin
            last_grant_idx <= grant_idx;
            // Decrement credit for granted requester
            if (credit[grant_idx] > 0)
                credit[grant_idx] <= credit[grant_idx] - 1'b1;
            // Refill when granted requester exhausts credit
            if (credit[grant_idx] <= 1) begin
                for (i = 0; i < REQ_W; i = i + 1)
                    credit[i] <= get_weight(i);
            end
        end
    end

endmodule
