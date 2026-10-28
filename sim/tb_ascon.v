`timescale 1ns/1ps

module ascon_core_tb;

    reg          clk;
    reg          rst_n;
    reg          start;
    reg          mode;
    reg  [127:0] key_in;
    reg  [127:0] nonce_in;
    reg  [63:0]  data_in;
    reg          data_valid;
    reg  [2:0]   data_cmd;

    wire         ready;
    wire         done;
    wire [63:0]  data_out;
    wire [127:0] tag_out;
    wire [255:0] hash_out;

    integer fd, status;
    reg [7:0]   op;      // Holds command character: M, K, S, D, T, H
    reg [31:0]  msg_id;
    reg [127:0] expected_tag;
    reg [255:0] expected_hash;
    integer pass_cnt, fail_cnt, total_cnt;

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    ascon_core dut (
        .clk(clk), .rst_n(rst_n), .start(start), .mode(mode),
        .key_in(key_in), .nonce_in(nonce_in), .data_in(data_in),
        .data_valid(data_valid), .data_cmd(data_cmd),
        .data_out(data_out), .tag_out(tag_out), .hash_out(hash_out),
        .ready(ready), .done(done)
    );

    task send_data;
        input [2:0]  cmd_in;
        input [63:0] val_in;
        begin
            wait(ready == 1'b1);
            @(posedge clk);
            #1;
            data_in = val_in;
            data_cmd = cmd_in;
            data_valid = 1;
            @(posedge clk);
            #1;
            data_valid = 0;
        end
    endtask

    initial begin
        rst_n = 0; start = 0; mode = 0; data_in = 0; key_in = 0; nonce_in = 0;
        data_valid = 0; data_cmd = 0; pass_cnt = 0; fail_cnt = 0; total_cnt = 0;
        
        #20 rst_n = 1; #10;

        fd = $fopen("D:/ModelSim/Project/SoC_merge/sim/stimulus.txt", "r");
        if (fd == 0) begin
            $display("ERROR: Cannot open stimulus.txt.");
            $finish;
        end

        $display("=========================================");
        $display("   STARTING ASCON NIST LWC BATCH TEST    ");
        $display("=========================================");

        while (!$feof(fd)) begin
            status = $fscanf(fd, "%s", op);
            if (status == 1) begin
                if (op == "M") begin
                    status = $fscanf(fd, "%d\n", msg_id);
                    total_cnt = total_cnt + 1;
                end
                else if (op == "K") begin
                    status = $fscanf(fd, "%x %x\n", key_in, nonce_in);
                end
                else if (op == "S") begin
                    status = $fscanf(fd, "%d\n", mode);
                    @(posedge clk); #1 start = 1;
                    @(posedge clk); #1 start = 0;
                end
                else if (op == "D") begin
                    status = $fscanf(fd, "%d %x\n", data_cmd, data_in);
                    send_data(data_cmd, data_in);
                end
                else if (op == "T") begin
                    status = $fscanf(fd, "%x\n", expected_tag);
                    wait(done == 1'b1);
                    if (tag_out !== expected_tag) begin
                        $display("=> [FAIL] AEAD Msg %0d | EXP: %h | GOT: %h", msg_id, expected_tag, tag_out);
                        fail_cnt = fail_cnt + 1;
                        $finish;
                    end else begin
                        pass_cnt = pass_cnt + 1;
                        if (pass_cnt % 50 == 0) $display("Passed %0d Messages...", pass_cnt);
                    end
                end
                else if (op == "H") begin
                    status = $fscanf(fd, "%x\n", expected_hash);
                    wait(done == 1'b1);
                    if (hash_out !== expected_hash) begin
                        $display("=> [FAIL] HASH Msg %0d | EXP: %h | GOT: %h", msg_id, expected_hash, hash_out);
                        fail_cnt = fail_cnt + 1;
                        $finish;
                    end else begin
                        pass_cnt = pass_cnt + 1;
                        if (pass_cnt % 50 == 0) $display("Passed %0d Messages...", pass_cnt);
                    end
                end
            end
        end
        $fclose(fd);
        
        $display("=========================================");
        $display(" RESULT: PASSED %0d / %0d TEST VECTORS", pass_cnt, total_cnt);
        if (fail_cnt == 0) $display("             CORE STATUS: PERFECT!       ");
        $display("=========================================");
        $finish;
    end
endmodule