module onehot_encoder
#(
    parameter INPUT_W  = 4,
    parameter OUTPUT_W = $clog2(INPUT_W)
)
(
    input  [INPUT_W-1:0]  i,
    output [OUTPUT_W-1:0] o
);
    // Priority encoder: returns index of the lowest set bit
    reg [OUTPUT_W-1:0] encoded;
    integer idx;

    always @(*) begin
        encoded = {OUTPUT_W{1'b0}};
        for (idx = INPUT_W-1; idx >= 0; idx = idx - 1) begin
            if (i[idx])
                encoded = idx[OUTPUT_W-1:0];
        end
    end

    assign o = encoded;

endmodule
