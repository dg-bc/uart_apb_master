module uart_apb_master(
input wire PCLK,
input wire PRESETn,
output wire o_uart_tx,
input wire i_uart_rx,
output reg PSEL,
output reg PENABLE,
output reg PWRITE,
input wire PREADY,
input wire PSLVERR,
output reg [31:0] PADDR,
output reg [31:0] PWDATA,
input wire [31:0] PRDATA
);

  //----Code starts here: integrated by Robei-----
  
        parameter MASTER_RESPONCE_CNT = 9;  // uart_透传提示字符计数：N-1
        
        wire                    rx_valid;
        wire [ 7:0]             rx_byte;
        wire                    rx_char_w  = (rx_valid && (rx_byte == 8'h57 || rx_byte == 8'h77));    // W, w
        wire                    rx_char_r  = (rx_valid && (rx_byte == 8'h52 || rx_byte == 8'h72));    // R, r
        wire                    rx_is_hex  = (rx_valid && ((rx_byte>=8'h30 && rx_byte<=8'h39) || (rx_byte>=8'h41 && rx_byte<=8'h46) || (rx_byte>=8'h61 && rx_byte<=8'h66)));    // 0~9, A~F, a~f
        wire [3:0] 		   	    rx_hex     = (rx_byte>=8'h30 && rx_byte<=8'h39) ? rx_byte[3:0] : (rx_byte[3:0] + 4'd9);        
   
    /*======================== uart-apb透传 ==============================*/
        reg             [ 2:0]  apb_c_state,apb_n_state  ;
        
        reg                   user_w;		    // 用户写
        reg                   user_r;  	        // 用户读
        reg  [31:0]           user_addr;	    // 解析地址
        reg  [31:0]           user_wdata;	    // 解析写数据
        reg  [31:0]           user_rdata;       // 暂存读数据
        reg  [31:0] 		  user_wcnt;	    // 写入字符计数
        reg  [3 :0]           maseter_rcnt;     // 回传提示字符计数 
        
      // === 接收命令 ===
          wire user_valid = apb_c_state == APB_IDLE;
        // --- 输入字符计数 ---
            always @(posedge PCLK or negedge PRESETn) begin
                if (!PRESETn) begin
                    user_wcnt <= 0;
                end else if(apb_c_state==APB_FINISH) begin
                    user_wcnt <= 0;
                end else if(rx_valid) begin	
                    user_wcnt <= user_wcnt + 1;
                end else begin
                    user_wcnt <= user_wcnt; // 保持
                end
            end
        
        // --- 判写操作码 --- 
            always @(posedge PCLK or negedge PRESETn) begin
                if (!PRESETn) begin
                    user_w <= 0;
                end else if(apb_c_state==APB_FINISH && maseter_rcnt == MASTER_RESPONCE_CNT) begin
                    user_w <= 0;
                end else if(apb_c_state==APB_IDLE && rx_char_w) begin
                    user_w <= 1;
                end	else begin
                    user_w <= user_w; // 保持
                end
            end
            
        // --- 判读操作码 --- 
            always @(posedge PCLK or negedge PRESETn) begin
                if (!PRESETn) begin
                    user_r <= 0;
                end else if(apb_c_state==APB_FINISH && maseter_rcnt == MASTER_RESPONCE_CNT) begin
                    user_r <= 0;
                end else if(apb_c_state==APB_IDLE && rx_char_r) begin
                    user_r <= 1;
                end else begin
                    user_r <= user_r; // 保持
                end
            end
            
        // --- 解析地址 --- 
            always @(posedge PCLK or negedge PRESETn) begin
                if (!PRESETn) begin
                    user_addr <= 0;
                end else if(apb_c_state==APB_FINISH && maseter_rcnt == MASTER_RESPONCE_CNT) begin
                    user_addr <= 0;
                end else if(user_wcnt>=1 && user_wcnt<9 && rx_is_hex) begin
                    user_addr <= user_addr<<4 | {28'b0,rx_hex[3:0]};
                end	else begin
                    user_addr <= user_addr;	// 保持
                end
            end
        
        // --- 解析写数据 --- 
            always @(posedge PCLK or negedge PRESETn) begin
                if (!PRESETn) begin
                    user_wdata <= 0;
                end else if(apb_c_state==APB_FINISH && maseter_rcnt == MASTER_RESPONCE_CNT) begin
                    user_wdata <= 0;
                end else if(user_w && user_wcnt>=9 && user_wcnt<17 && rx_is_hex) begin
                    user_wdata <= user_wdata<<4 | {28'b0,rx_hex[3:0]};		
                end else begin
                    user_wdata <= user_wdata; // 保持
                end
            end
        
        // --- 收指令结束,准备执行 ---
            wire rx_w_rcv_end = user_w && user_wcnt==17;
            wire rx_r_rcv_end = user_r && user_wcnt==9;
      
      // === apb主机输出控制(回传提示信息状态控制) ===
            localparam      [ 1:0] APB_IDLE     = 2'd0;
            localparam      [ 1:0] APB_SETUP  	= 2'd1;
            localparam      [ 1:0] APB_ENABLE 	= 2'd2;
            localparam      [ 1:0] APB_FINISH   = 2'd3;
        // -- 状态转移 --
            always @(posedge PCLK or negedge PRESETn) begin
                if (!PRESETn) begin
                    apb_c_state <= APB_IDLE;
                end else begin
                    apb_c_state <= apb_n_state;
                end
            end
        
        // -- 转移事件 --
            always @(*) begin
                case (apb_c_state)
                    APB_IDLE: begin
                        if (rx_w_rcv_end | rx_r_rcv_end) begin
                            apb_n_state = APB_SETUP;
                        end else begin
                            apb_n_state = APB_IDLE;
                        end
                    end
                    APB_SETUP: begin
                        apb_n_state = APB_ENABLE;
                    end
                    APB_ENABLE: begin
                        if (PREADY) begin
                            apb_n_state = APB_FINISH;
                        end else begin
                            apb_n_state = APB_ENABLE;
                        end
                    end
                    APB_FINISH: begin				// 在此阶段即刻发送数据[3]
                        if(maseter_rcnt == MASTER_RESPONCE_CNT)   apb_n_state = APB_IDLE;
                        else                    apb_n_state = APB_FINISH;
                    end
                    default: begin
                        apb_n_state = APB_IDLE;
                    end
                endcase
            end
        
        // -- 输出信号控制 --
            always @(posedge PCLK) begin
                if (!PRESETn) begin
                    PSEL     <= 0;
                    PENABLE  <= 0;
                    PWRITE   <= 0;
                    PADDR    <= 0;
                    PWDATA   <= 0;
                end else begin
                    case (apb_c_state)
                        APB_IDLE: begin
                            PSEL    <= 0;
                            PENABLE <= 0;
                        end
                        APB_SETUP: begin
                            PSEL   <= 1;
                            PWRITE <= user_w;
                            PADDR  <= user_addr;
                            if (user_w) begin
                                PWDATA <= user_wdata;
                            end	else begin
                                PWDATA <= 0;
                            end
                        end
                        APB_ENABLE: begin
                            PENABLE <= 1;
                        end
                        APB_FINISH: begin
                            PENABLE <= 0;
                            PSEL <= 0;
                        end
                        default: begin
                            PSEL <= 0;
                            PENABLE <= 0;
                        end
                    endcase
                end
            end
        
        // --- master读走数据 --- 
            always @(posedge PCLK) begin
                if (!PRESETn) begin
                    user_rdata <= 0;
                end else if (user_r && PSEL & PENABLE & PREADY) begin
                    user_rdata <= PRDATA;
                end	else begin
                    user_rdata <= user_rdata;
                end
            end 
      
      // === 响应帧回传 === 
            reg 			tx_valid;	// 主机发送start
            reg [7:0] 		tx_byte; 	// tx发送字节
            wire 			tx_ready;	// tx准备好（不满）
            /* 发送字符计数 */
            always @(posedge PCLK or negedge PRESETn)begin
                if(!PRESETn)begin
                    maseter_rcnt <= 0;
                end else begin
                    if(tx_ready && apb_c_state == APB_FINISH)   maseter_rcnt <= maseter_rcnt + 1;
                    else if(apb_c_state == APB_IDLE)            maseter_rcnt <= 0;
                    else                                    maseter_rcnt <= maseter_rcnt;
                end
            end
    
    
        // -- 启动tx --
            always @(posedge PCLK) begin
                if (!PRESETn) begin
                    tx_valid     <= 0;
                end else if(apb_c_state==APB_IDLE) begin 
                    tx_valid     <= 0;
                end else if(apb_c_state==APB_FINISH) begin
                    tx_valid     <= 1;
                end	else begin
                    tx_valid 	<= 0;
                end
            end 
        
        // --- 字节填充 --- 
            always @(posedge PCLK or negedge PRESETn) begin
                if(!PRESETn) begin
                    tx_byte <= 0;
                end else begin
                    case(maseter_rcnt)
                        // 操作码起始
                        0: tx_byte <= (user_w ? 8'h77 : 8'h72); // w or r

                        // 数据转assic回传
                        1: tx_byte <= (user_w ? user_wdata[31:28] : user_rdata[31:28]) + ((user_w ? user_wdata[31:28] : user_rdata[31:28]) < 4'd10 ? 8'h30 : 8'h37);
                        2: tx_byte <= (user_w ? user_wdata[27:24] : user_rdata[27:24]) + ((user_w ? user_wdata[27:24] : user_rdata[27:24]) < 4'd10 ? 8'h30 : 8'h37);
                        3: tx_byte <= (user_w ? user_wdata[23:20] : user_rdata[23:20]) + ((user_w ? user_wdata[23:20] : user_rdata[23:20]) < 4'd10 ? 8'h30 : 8'h37);
                        4: tx_byte <= (user_w ? user_wdata[19:16] : user_rdata[19:16]) + ((user_w ? user_wdata[19:16] : user_rdata[19:16]) < 4'd10 ? 8'h30 : 8'h37);
                        5: tx_byte <= (user_w ? user_wdata[15:12] : user_rdata[15:12]) + ((user_w ? user_wdata[15:12] : user_rdata[15:12]) < 4'd10 ? 8'h30 : 8'h37);
                        6: tx_byte <= (user_w ? user_wdata[11:8]  : user_rdata[11:8])  + ((user_w ? user_wdata[11:8]  : user_rdata[11:8])  < 4'd10 ? 8'h30 : 8'h37);
                        7: tx_byte <= (user_w ? user_wdata[7:4]   : user_rdata[7:4])   + ((user_w ? user_wdata[7:4]   : user_rdata[7:4])   < 4'd10 ? 8'h30 : 8'h37);
                        8: tx_byte <= (user_w ? user_wdata[3:0]   : user_rdata[3:0])   + ((user_w ? user_wdata[3:0]   : user_rdata[3:0])   < 4'd10 ? 8'h30 : 8'h37);
                        
                        // 结束符
                        9: tx_byte <= 8'h24;    // $
                    endcase
                end
            end
        
  
      
  
//---Module instantiation---
  uart_rx #(
.FIFO_EA(4)
) 
uart_rx1(
    .clk(PCLK),
    .rstn(PRESETn),
    .i_uart_rx(i_uart_rx),
    .o_tready(user_valid),
    .o_tvalid(rx_valid),
    .o_tdata(rx_byte),
    .o_overflow()
);

  uart_tx #(
.FIFO_EA(4)
) 
uart_tx2(
    .clk(PCLK),
    .rstn(PRESETn),
    .i_tvalid(tx_valid),
    .i_tdata(tx_byte),
    .i_tkeep(),
    .i_tlast(1'b0),
    .o_uart_tx(o_uart_tx),
    .o_tready(tx_ready)
);

endmodule    //uart_apb_master
