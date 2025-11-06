// 该模块首先采用缓存命中与缺失策略，构建以32个像素位一个缓冲区的缓存表，寄存器形式
// 拼接矫正后的图像数据，当拼接至16个像素后，输出数据
// 采用直接相连映射，以便进行命中检查
// 缓冲区的tag为4位，前三位标志行号模5，后1位标志列号模32

module undistort #(
        parameter CACHE_NUM = 10,  //缓冲区个数
        parameter CACHE_WIDTH = 32,  //缓冲区的像素点个数
        parameter ADDR_WIDTH = 16,  //cache的地址宽度，行号+列号，720行——10位，1280/32列——6位
        parameter LINE_CNT = 2  //一行的读取次数，应该等于猝发长度
    ) (
        input         clk,      //应该为axi4时钟域
        input         rst,      
        input [127:0] r_data,   // 从axi4读到的数据
        input         r_valid,  // 读数据有效
        input         r_last,  // axi4 rlast，用于对齐

        input rframe_vsync_neg, //场同步下降沿

        // 与axi4连接
        output reg                  r_request,  //读请求
        output reg [ADDR_WIDTH-1 :0]r_addr,  //当前要访存读取的行号

        //与FIFO连接
        input              fifo_full,
        output reg [127:0] out_data,  //输出数据
        output reg         out_valid  //输出数据有效
        
    );
    // reg [1:0]out_state; //输出状态机，如果正在输出，0表示输出偶数位，1表示输出奇数位

    (* ram_style = "block" *)
    reg [7:0] cache[CACHE_NUM-1:0][CACHE_WIDTH-1:0]; //缓存块
    (* ram_style = "block" *)
    reg [ADDR_WIDTH-1:0] cache_addr [CACHE_NUM-1:0]; //缓存块地址

    reg [CACHE_NUM-1:0] cache_valid;  //缓存块合法性
    reg rcnt; //猝发拍数计数，现在两拍猝发
    wire [2:0] tag_h = r_addr[15:6]%5;
    wire [3:0] r_wtag = {tag_h, r_addr[0]};//表示当前写的缓冲区的地址

    //单次访存读取模块
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cache_valid <= 0;
            rcnt <= 0;
        end
        else if(!r_valid) begin //如果当前读数据无效,则维持line合法性，清空位数，可能应该用其他的判断
            cache_valid <= cache_valid;
            rcnt <= rcnt;
        end
        else if(r_last) begin // 如果line_cnt不正确，则会被对齐一次，后面就应该正确
            rcnt <= 0;
            cache[r_wtag][16] <= r_data[0*8 +: 8];  
            cache[r_wtag][17] <= r_data[1*8 +: 8];  
            cache[r_wtag][18] <= r_data[2*8 +: 8];  
            cache[r_wtag][19] <= r_data[3*8 +: 8];  
            cache[r_wtag][20] <= r_data[4*8 +: 8];  
            cache[r_wtag][21] <= r_data[5*8 +: 8];  
            cache[r_wtag][22] <= r_data[6*8 +: 8];  
            cache[r_wtag][23] <= r_data[7*8 +: 8];  
            cache[r_wtag][24] <= r_data[8*8 +: 8];  
            cache[r_wtag][25] <= r_data[9*8 +: 8];  
            cache[r_wtag][26] <= r_data[10*8 +: 8]; 
            cache[r_wtag][27] <= r_data[11*8 +: 8]; 
            cache[r_wtag][28] <= r_data[12*8 +: 8]; 
            cache[r_wtag][29] <= r_data[13*8 +: 8]; 
            cache[r_wtag][30] <= r_data[14*8 +: 8]; 
            cache[r_wtag][31] <= r_data[15*8 +: 8]; 
            cache_addr[r_wtag] <= (rcnt == LINE_CNT-1)? r_addr:cache_addr[r_wtag];
            cache_valid[r_wtag] <= (rcnt == LINE_CNT-1) ? 1 : cache_valid[r_wtag];
        end
        else begin
            rcnt <= rcnt + 1;
            cache[r_wtag][0 ] <= r_data[0*8 +: 8];  
            cache[r_wtag][1 ] <= r_data[1*8 +: 8];  
            cache[r_wtag][2 ] <= r_data[2*8 +: 8];  
            cache[r_wtag][3 ] <= r_data[3*8 +: 8];  
            cache[r_wtag][4 ] <= r_data[4*8 +: 8];  
            cache[r_wtag][5 ] <= r_data[5*8 +: 8];  
            cache[r_wtag][6 ] <= r_data[6*8 +: 8];  
            cache[r_wtag][7 ] <= r_data[7*8 +: 8];  
            cache[r_wtag][8 ] <= r_data[8*8 +: 8];  
            cache[r_wtag][9 ] <= r_data[9*8 +: 8];  
            cache[r_wtag][10] <= r_data[10*8 +: 8]; 
            cache[r_wtag][11] <= r_data[11*8 +: 8]; 
            cache[r_wtag][12] <= r_data[12*8 +: 8]; 
            cache[r_wtag][13] <= r_data[13*8 +: 8]; 
            cache[r_wtag][14] <= r_data[14*8 +: 8]; 
            cache[r_wtag][15] <= r_data[15*8 +: 8]; 
        end
    end

    //lut握手信号
    wire lut_valid_in1;//低位数据合法
    wire lut_valid_in2;//高位数据合法
    reg  lut_valid;//lut状态暂存
    wire [127:0] lut_in;//lut模块输入

    reg lut_request; //lut请求信号
    always @(posedge clk or posedge rst)begin
        if(rst) lut_request <= 1;
        else if(lut_valid_in2) lut_request <= 0;
        else if(out_valid)lut_request <= 1;
        else lut_request <= lut_request;
    end

    //例化查找表模块
    LUT_decompress u_LUT_decompress(
        .clk(clk),
        .rst(rst),
        .LUT_request(lut_request),
        .data_valid_out_1(lut_valid_in1),
        .data_valid_out_2(lut_valid_in2),
        .interpolated_data(lut_in)
    );

    reg [31:0] lut [7:0]; //16个点8个lut, 按01,23,45...存储
    always @(posedge clk or posedge rst) begin
        if (rst);
        else if (lut_valid_in1) begin
            lut[0] <= lut_in[32*0 +: 32]; 
            lut[1] <= lut_in[32*1 +: 32];  
            lut[2] <= lut_in[32*2 +: 32];  
            lut[3] <= lut_in[32*3 +: 32];  

        end else if (lut_valid_in2) begin
            lut[4] <= lut_in[32*0 +: 32];
            lut[5] <= lut_in[32*1 +: 32];
            lut[6] <= lut_in[32*2 +: 32];
            lut[7] <= lut_in[32*3 +: 32];
        end
    end

    wire all_hit;
    wire [1:0] hit[7:0]; //目前每个像素点需要2组数据，后面引回双线性插值的话，应当改成3位
    reg [20:0] out_addr;  //21位地址数据，标明当前处理的像素位置，前11位是列号，后10位是行号
    wire frame_pending = out_addr[9:0] == 10'd720; //这个magic number是最后一次读的位置
    wire cache_ready = all_hit && ~frame_pending && lut_valid && !fifo_full;//表示缓存块已经准备好，实际用处为控制像素组切换。

    always @(posedge clk or posedge rst) begin
        if(rst) lut_valid <= 0;
        else if(cache_ready) lut_valid <= 0; // 输出结束后立即拉低lut有效信号
        else if(lut_valid_in2) lut_valid <= 1; // lut模块valid信号后，拉高lut有效
        else lut_valid <= lut_valid; 
    end

    //一次并行处理16位像素数据

    // 预计算所有可能的 cache 索引
    wire [2:0] idx_mid_h[7:0];
    wire [2:0] idx_top_h[7:0];
    wire [3:0] idx_mid [7:0];   // 中间行缓存索引
    wire [3:0] idx_top [7:0];   // 上一行缓存索引
    wire [ADDR_WIDTH-1:0] addr_mid [7:0]; // 中间行期望地址
    wire [ADDR_WIDTH-1:0] addr_top [7:0]; // 上一行期望地址
    wire [7:0] hit_ok;
    generate
        for (genvar i = 0; i < 8; i = i+1) begin
            assign idx_mid_h[i] = lut[i][14:5]%5;
            assign idx_top_h[i] = (lut[i][14:5]-10'd1)%5;
            assign idx_mid[i] = {idx_mid_h[i], lut[i][26]};
            assign idx_top[i] = {idx_top_h[i], lut[i][26]}; 
            assign addr_mid[i] = {lut[i][14:5], lut[i][31:26]};
            assign addr_top[i] = {lut[i][14:5] - 10'd1, lut[i][31:26]};
            assign hit[i][1] = (cache_addr[idx_mid[i]] == addr_mid[i]) && cache_valid[idx_mid[i]]; //命中的要求：地址相等，且缓存块有效
            assign hit[i][0] = (cache_addr[idx_top[i]] == addr_top[i]) && cache_valid[idx_top[i]];
            assign hit_ok[i] = (hit[i] == 2'b11); //暂时的用处只是过渡信号来计算all_hit，后面可以拓展输出时序
        end
    endgenerate
    assign all_hit = &hit_ok;

    //处理地址切换逻辑
    always @(posedge clk or posedge rst) begin
        if (rst)
            out_addr <= 21'b0;
        else if (rframe_vsync_neg) //地址重置：当外部的帧同步下降沿出现时，重置地址，离开节点阻塞
            out_addr <= 21'b0;
        else if (frame_pending) //地址阻塞：如果处理到最后一个点，应当阻塞并等待外部FIFO读完
            out_addr <= out_addr;
        else if (cache_ready) begin
            out_addr[20:10] <= (out_addr[20:10] == 11'd1264) ? 0: out_addr[20:10] + 16;
            out_addr[9:0]   <= (out_addr[20:10] == 11'd1264) ? out_addr[9:0] + 1 : out_addr[9:0];
        end
        else
            out_addr <= out_addr;
    end

    // 生成访存地址，按顺序检索
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_addr <= 0;
        end else if (!all_hit) begin
            // 优先级：i=0 到 i=15，每个 i 先检查 hit[i][1]，再检查 hit[i][0]
            if (hit[0][1] == 1'b0) begin
                r_addr <= addr_mid[0];
            end else if (hit[0][0] == 1'b0) begin
                r_addr <= addr_top[0];
            end else if (hit[1][1] == 1'b0) begin
                r_addr <= addr_mid[1];
            end else if (hit[1][0] == 1'b0) begin
                r_addr <= addr_top[1];
            end else if (hit[2][1] == 1'b0) begin
                r_addr <= addr_mid[2];
            end else if (hit[2][0] == 1'b0) begin
                r_addr <= addr_top[2];
            end else if (hit[3][1] == 1'b0) begin
                r_addr <= addr_mid[3];
            end else if (hit[3][0] == 1'b0) begin
                r_addr <= addr_top[3];
            end else if (hit[4][1] == 1'b0) begin
                r_addr <= addr_mid[4];
            end else if (hit[4][0] == 1'b0) begin
                r_addr <= addr_top[4];
            end else if (hit[5][1] == 1'b0) begin
                r_addr <= addr_mid[5];
            end else if (hit[5][0] == 1'b0) begin
                r_addr <= addr_top[5];
            end else if (hit[6][1] == 1'b0) begin
                r_addr <= addr_mid[6];
            end else if (hit[6][0] == 1'b0) begin
                r_addr <= addr_top[6];
            end else if (hit[7][1] == 1'b0) begin
                r_addr <= addr_mid[7];
            end else begin
                r_addr <= addr_top[7];
            end
        end
    end

    // 生成访存请求，未命中时拉高，否则拉低
    always @(posedge clk or posedge rst) begin
        if (rst)
            r_request <= 0;
        else if (!all_hit) begin 
            r_request <= 1;
        end
        else
            r_request <= 0;
    end
    
    //核心部分，对输出及其valid信号赋值
    function [7:0] get_even_byte; //获取偶数
        input [2:0] i;
        reg [4:0] base_row;

        begin
            base_row= lut[i][25:21];
            case({out_addr[0],lut[i][21],lut[i][5]}) 
                3'b000,3'b011:get_even_byte = cache[idx_mid[i]][base_row]; 
                3'b001,3'b010:get_even_byte = (base_row == 5'd31)? cache[idx_mid[i]][base_row- 5'b1]:cache[idx_mid[i]][base_row+ 5'b1];
                3'b100,3'b101:get_even_byte = cache[idx_top[i]][base_row];
                3'b110,3'b111:get_even_byte = (base_row == 5'd0)?  cache[idx_mid[i]][base_row+ 5'b1]:cache[idx_mid[i]][base_row- 5'b1];
            endcase
        end
    endfunction 

    function [7:0] get_odd_byte; //获取偶数
        input [2:0] i;
        reg [4:0] base_row;

        begin
            base_row = lut[i][25:21];
            case({out_addr[0],lut[i][21],lut[i][5]})
                3'b100,3'b111:get_odd_byte = cache[idx_mid[i]][base_row]; 
                3'b101,3'b110:get_odd_byte = (base_row == 5'd31)? cache[idx_mid[i]][base_row- 5'b1]:cache[idx_mid[i]][base_row+ 5'b1];
                3'b011,3'b010:get_odd_byte = cache[idx_top[i]][base_row];
                3'b001,3'b000:get_odd_byte = (base_row == 5'd0)?  cache[idx_mid[i]][base_row+ 5'b1]:cache[idx_mid[i]][base_row- 5'b1];
            endcase
        end
    endfunction
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            out_data <= 128'b0;
        end
        else if(cache_ready) begin
                out_data[0   +:8] <= get_even_byte(3'd0);
                out_data[16  +:8] <= get_even_byte(3'd1);
                out_data[32  +:8] <= get_even_byte(3'd2);
                out_data[48  +:8] <= get_even_byte(3'd3);
                out_data[64  +:8] <= get_even_byte(3'd4);
                out_data[80  +:8] <= get_even_byte(3'd5);
                out_data[96  +:8] <= get_even_byte(3'd6);
                out_data[112 +:8] <= get_even_byte(3'd7);
                out_data[8   +:8] <= get_odd_byte(3'd0);
                out_data[24  +:8] <= get_odd_byte(3'd1);
                out_data[40  +:8] <= get_odd_byte(3'd2);
                out_data[56  +:8] <= get_odd_byte(3'd3);
                out_data[72  +:8] <= get_odd_byte(3'd4);
                out_data[88  +:8] <= get_odd_byte(3'd5);
                out_data[104 +:8] <= get_odd_byte(3'd6);
                out_data[120 +:8] <= get_odd_byte(3'd7);
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) out_valid <= 0;
        else if(cache_ready) out_valid <= 1;
        else out_valid <= 0;
    end

    wire [7:0]debug_cache02 = cache[0][2];
    wire [7:0]debug_cache13 = cache[1][3]; 
    wire [127:0] debug_rdata = r_data;
    wire [10:0] debug_row_addr = out_addr[20:10];
    wire [9:0] debug_line_addr = out_addr[9:0];

endmodule