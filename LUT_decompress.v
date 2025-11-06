module LUT_decompress (

    input wire clk,                              //输入像素时钟，74.25MHZ
    input wire rst,                              //复位信号
    input wire LUT_request,                      // 16个坐标请求

    output reg data_valid_out_1,                 // 前8个输出数据有效
    output reg data_valid_out_2,                 // 后8个输出数据有效
    output  [255:0] interpolated_data            // 解压后的插值数据
    );
    //参数
    localparam COMPRESSED_WIDTH = 80;            //80  、120
    localparam COMPRESSED_HEIGHT = 90;           //90  、135
    localparam ORIGINAL_WIDTH = 1280;            //1280、1920
    localparam ORIGINAL_HEIGHT = 720;            //720 、1080
    
    reg[15:0] compressed_buffer_X_L1[7:0] ;   //6个16位                  
    reg[15:0] compressed_buffer_Y_L1[7:0] ;   //6个16位  
    reg[15:0] compressed_buffer_X_L2[7:0] ;   //6个16位                  
    reg[15:0] compressed_buffer_Y_L2[7:0] ;   //6个16位 
    reg[2:0]  rd_count;          //累计8次跳转下一行。
    reg[7:0]  rd_row_count;      //正在读取的压缩数据列数（0~WIDTH-1）
    reg[7:0]  rd_line_count;     //正在读取的压缩数据行数（0~HEIGHT-1）
    reg[2:0]  rd_position;       //目前读取到缓存区的位置。//0、2、4之间转换。
    reg buffer_data_valid;       //缓冲数据有效
    
    //读取BRAM
    wire[63:0] bram_rdata; //获取数据(2个32位数据)
    reg [12:0]  rd_addr;   //数据地址
    reg rd_en;             //读取使能
   
    //解压    
    reg [7:0] compress_row_count;    //压缩列计数
    reg [2:0] line_8X8=0;            //对应8×8块中的行数
    reg [43:0] output_buffer [3:0];  // 8个32位数据输出缓存区

    // 状态机
    reg [1:0] state;
    reg state_out;
    always @(posedge clk or posedge rst) begin
        if(rst) state <= 0;
        else if(state_out) begin
            if(state == 0) state <= (buffer_data_valid)? 2'b01:2'b00;
            else state <= state+1;
        end
        else state <= state;
    end


    /*********BRAM读取？*********/
    //BRAM例化
    BRAM_LUT u_bram
    (
    .re      (1'b1     ),   //读出数据使能  
    .raddren (1'b1      ),  //读入地址永远使能
    .reset   (0    ),       //1复位
    .waddr   (0),           
    .wdata_a (0),          
    .rdata_b (bram_rdata),  //读出的数据（11位：5位）
    .raddr   (rd_addr   ),  //需要读入的地址
    .clk     (clk       )   //时钟
    );


    //读缓冲区控制
    reg [2:0] init_state;
    always@(posedge clk or posedge rst) begin
        if(rst)begin
            //读取部分的reg
            rd_addr<=0;
            rd_count<= 0;
            rd_position<=0;
            rd_row_count<=0;
            rd_line_count<=0;
            buffer_data_valid<=0;
            init_state <=0;
        end
        //全局状态1或3：读取
        else begin
            case(state)
            2'b01: begin
                rd_addr<=rd_addr+COMPRESSED_WIDTH;
                if(rd_addr == 0) begin
                    compressed_buffer_X_L1[rd_position]<=16'h192A;                
                    compressed_buffer_Y_L1[rd_position]<=16'h0E9C;
                end
                else begin
                    compressed_buffer_X_L1[rd_position]<=bram_rdata[31:16];                
                    compressed_buffer_Y_L1[rd_position]<=bram_rdata[15:0];
                end
                compressed_buffer_X_L1[rd_position+3'b001]<=bram_rdata[63:48];                 
                compressed_buffer_Y_L1[rd_position+3'b001]<=bram_rdata[47:32];    
            end
            2'b11: begin
                rd_row_count<=(rd_row_count == COMPRESSED_WIDTH + 1)? 0 : rd_row_count+1;
                rd_count <= (rd_row_count == COMPRESSED_WIDTH + 1)?rd_count+1:rd_count;
                if(rd_row_count == COMPRESSED_WIDTH + 1) begin 
                    if(rd_count == 3'b111) begin
                        rd_line_count <= (rd_line_count == COMPRESSED_HEIGHT - 1)? 0:rd_line_count+1;
                        rd_addr <= (rd_line_count == COMPRESSED_HEIGHT - 2)? rd_addr - COMPRESSED_WIDTH - COMPRESSED_WIDTH - 1 : rd_addr -COMPRESSED_WIDTH - 1;
                    end
                    else rd_addr <= rd_addr - COMPRESSED_WIDTH - COMPRESSED_WIDTH - 1;
                end else rd_addr <= rd_addr - COMPRESSED_WIDTH + 1;
                buffer_data_valid <= (rd_row_count == COMPRESSED_WIDTH + 1)? 0:buffer_data_valid;
                rd_position <= (rd_row_count == COMPRESSED_WIDTH + 1)? 0: rd_position+3'b010;
                compressed_buffer_X_L2[rd_position+3'b001]<=bram_rdata[63:48];                 
                compressed_buffer_Y_L2[rd_position+3'b001]<=bram_rdata[47:32];    
                compressed_buffer_X_L2[rd_position]<=bram_rdata[31:16];                
                compressed_buffer_Y_L2[rd_position]<=bram_rdata[15:0];
            end
            2'b10: ;
            2'b00: begin
                if(buffer_data_valid == 0) begin
                    case(init_state)
                        3'b000: begin 
                            rd_addr <= rd_addr + COMPRESSED_WIDTH;
                            init_state <= 3'b001;
                        end
                        3'b001: begin
                            rd_addr <= rd_addr-COMPRESSED_WIDTH+1;
                            if(rd_addr == 10'd80) begin
                                compressed_buffer_X_L1[rd_position]<=16'h192A;                
                                compressed_buffer_Y_L1[rd_position]<=16'h0E9C;
                            end
                            else begin
                                compressed_buffer_X_L1[rd_position]<=bram_rdata[31:16];                
                                compressed_buffer_Y_L1[rd_position]<=bram_rdata[15:0];
                            end
                            init_state <= 3'b010;
                            compressed_buffer_X_L1[rd_position+1]<=bram_rdata[63:48];                 
                            compressed_buffer_Y_L1[rd_position+1]<=bram_rdata[47:32];    
                        end
                        3'b010:begin
                            rd_addr<= rd_addr+COMPRESSED_WIDTH;
                            rd_position <= 2'd2;
                            init_state <= 3'b011;
                            compressed_buffer_X_L2[rd_position+1]<=bram_rdata[63:48];                 
                            compressed_buffer_Y_L2[rd_position+1]<=bram_rdata[47:32];    
                            compressed_buffer_X_L2[rd_position]<=bram_rdata[31:16];                
                            compressed_buffer_Y_L2[rd_position]<=bram_rdata[15:0];
                        end
                        3'b011:begin
                            init_state <= 3'b100;
                            rd_addr <= rd_addr-COMPRESSED_WIDTH+1;
                            compressed_buffer_X_L1[rd_position+1]<=bram_rdata[63:48];                 
                            compressed_buffer_Y_L1[rd_position+1]<=bram_rdata[47:32];    
                            compressed_buffer_X_L1[rd_position]<=bram_rdata[31:16];                
                            compressed_buffer_Y_L1[rd_position]<=bram_rdata[15:0];
                        end
                        3'b100: begin
                            init_state <= 3'b000;
                            rd_row_count<= 2;
                            buffer_data_valid <= 1;
                            rd_position <= 3'd4;
                            compressed_buffer_X_L2[rd_position+1]<=bram_rdata[63:48];                 
                            compressed_buffer_Y_L2[rd_position+1]<=bram_rdata[47:32];    
                            compressed_buffer_X_L2[rd_position]<=bram_rdata[31:16];                
                            compressed_buffer_Y_L2[rd_position]<=bram_rdata[15:0];
                        end
                    endcase
                end
                else begin 
                    init_state <= 0;
                end
            end
            endcase
        end 
    end
    /**********以下为缓冲区解压部分************/
    /*****解压位置更换******/
    reg [3:0]de_count;//0到7
    //数据有效、且输出之后在自增（跟输出对齐）
    always@(posedge clk or posedge rst) begin
        if(rst)begin
            de_count<=0;
        end
        //情况：列边界
        else if(buffer_data_valid == 0)begin
            de_count<=0;
        end
        //情况：一般读取
        else if(state[0] == 1)begin
            de_count<=de_count+1; 
        end
        //情况：其他。(则锁存、不需要读取。)
        else begin
            de_count<=de_count;
        end
    end

    /*****计数部分******/
    //输出数据有效时再递增（输出一次递增一次）
    always@(posedge clk or posedge rst) begin
        if(rst)begin
            //解压部分的reg
            compress_row_count<=0;  //压缩数据中的列数
            line_8X8<=0;            //8*8解压块中的行数
        end
        //（1）情况：列边界(刚好到最后一列、且不到最后一行)
        //行增加。列重置。
        else if(state==3
        &&compress_row_count==((COMPRESSED_WIDTH<<1)-2)) begin
            line_8X8<=line_8X8 + 3'b001;
            compress_row_count<=0;
        end
        //（1）情况：一般情况下(有需求、缓冲区数据有效、且不到最后一列)，块中列数增加2
        else if(state==3) begin 
            compress_row_count<=compress_row_count+2;
        end
    end

    /*******提前计算参数与数据****/
    wire [15:0] L0R0_HIGH;// 
    wire [15:0] L0R1_HIGH;//
    wire [15:0] L1R0_HIGH;//
    wire [15:0] L1R1_HIGH;//
    wire [15:0] L0R0_LOW; // 
    wire [15:0] L0R1_LOW; //
    wire [15:0] L1R0_LOW; //
    wire [15:0] L1R1_LOW; //
    assign L0R0_HIGH=compressed_buffer_X_L1[de_count];
    assign L0R1_HIGH=compressed_buffer_X_L1[de_count+1];
    assign L1R0_HIGH=compressed_buffer_X_L2[de_count];
    assign L1R1_HIGH=compressed_buffer_X_L2[de_count+1];
    assign L0R0_LOW=compressed_buffer_Y_L1[de_count];
    assign L0R1_LOW=compressed_buffer_Y_L1[de_count+1];
    assign L1R0_LOW=compressed_buffer_Y_L2[de_count];
    assign L1R1_LOW=compressed_buffer_Y_L2[de_count+1];
    wire [6:0] parameter_a[7:0];//8个参数
    wire [6:0] parameter_b[7:0];
    wire [6:0] parameter_c[7:0];
    wire [6:0] parameter_d[7:0];
    generate
    for(genvar i = 0; i < 8; i = i + 1) begin
        assign parameter_a[i] = (8-line_8X8)*(8-i);
        assign parameter_b[i] = (8-line_8X8)*i;
        assign parameter_c[i] = line_8X8*(8-i);
        assign parameter_d[i] = line_8X8*i;
    end
    endgenerate

    // 请求信号对齐
    reg request_dly;
    always@(posedge clk or posedge rst)begin
        if(rst) request_dly <= 0;
        else request_dly <= LUT_request;
    end

    always @(posedge clk or posedge rst) begin
        if(rst) state_out <= 0;
        else if(state == 3) state_out <= 0;
        else if(LUT_request && ~request_dly) state_out <= 1;
        else state_out <= state_out;
    end

    // 合法信号输出
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            data_valid_out_1<=0;
            data_valid_out_2<=0;
        end
        else begin
            case(state)
            2'b00,2'b10:begin
                data_valid_out_1<=0;
                data_valid_out_2<=0; 
            end
            2'b01: begin
                data_valid_out_1<=1;
                data_valid_out_2<=0; 
            end
            2'b11: begin
                data_valid_out_1<=0;
                data_valid_out_2<=1;
            end
            endcase
        end
    end
    
    // 数据处理并输出
    always @(posedge clk) begin
        if(compress_row_count == (COMPRESSED_WIDTH << 1) -2) ;
        else if(state[0] == 0) begin
            output_buffer[0][43:22] <=(L0R0_HIGH*parameter_a[0]
                                      +L0R1_HIGH*parameter_b[0]
                                      +L1R0_HIGH*parameter_c[0]
                                      +L1R1_HIGH*parameter_d[0])>>6; 
            output_buffer[0][21:0 ] <=( L0R0_LOW*parameter_a[0]
                                      + L0R1_LOW*parameter_b[0]
                                      + L1R0_LOW*parameter_c[0]
                                      + L1R1_LOW*parameter_d[0])>>6;
            output_buffer[1][43:22] <=(L0R0_HIGH*parameter_a[2]
                                      +L0R1_HIGH*parameter_b[2]
                                      +L1R0_HIGH*parameter_c[2]
                                      +L1R1_HIGH*parameter_d[2])>>6;
            output_buffer[1][21:0 ] <=( L0R0_LOW*parameter_a[2]
                                      + L0R1_LOW*parameter_b[2]
                                      + L1R0_LOW*parameter_c[2]
                                      + L1R1_LOW*parameter_d[2])>>6;
        end
        else begin
            output_buffer[2][43:22] <=(L0R0_HIGH*parameter_a[4]
                                      +L0R1_HIGH*parameter_b[4]
                                      +L1R0_HIGH*parameter_c[4]
                                      +L1R1_HIGH*parameter_d[4])>>6; 
            output_buffer[2][21:0 ] <=(L0R0_LOW*parameter_a[4]
                                      +L0R1_LOW*parameter_b[4]
                                      +L1R0_LOW*parameter_c[4]
                                      +L1R1_LOW*parameter_d[4])>>6;
            output_buffer[3][43:22] <=(L0R0_HIGH*parameter_a[6]
                                      +L0R1_HIGH*parameter_b[6]
                                      +L1R0_HIGH*parameter_c[6]
                                      +L1R1_HIGH*parameter_d[6])>>6;
            output_buffer[3][21:0 ] <=(L0R0_LOW*parameter_a[6]
                                      +L0R1_LOW*parameter_b[6]
                                      +L1R0_LOW*parameter_c[6]
                                      +L1R1_LOW*parameter_d[6])>>6;
        end
    end    
    //组合赋值：高放高、低放低
    assign interpolated_data = {
            output_buffer[3][37:22], output_buffer[3][15:0],
            output_buffer[2][37:22], output_buffer[2][15:0],
            output_buffer[1][37:22], output_buffer[1][15:0],   
            output_buffer[0][37:22], output_buffer[0][15:0]
            };
endmodule    
    

