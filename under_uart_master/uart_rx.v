

module uart_rx #(
    // RX fifo depth
    parameter  FIFO_EA   = 0             // 0:no fifo   1,2:depth=4   3:depth=8   4:depth=16  ...  10:depth=1024   11:depth=2048  ...
) (
    input  wire        rstn,
    input  wire        clk,
    // UART RX input signal
    input  wire        i_uart_rx,
    // output master. Associated clock = clk. 
    input  wire        o_tready,
    output reg         o_tvalid,
    output reg  [ 7:0] o_tdata,
    // report whether there's a overflow
    output reg         o_overflow
);

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
// Generate fractional precise upper limit for counter
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
localparam uart_baud    = 50000000 / 115200;
localparam S_IDLE       = 4'b0001;
localparam S_START      = 4'b0010;
localparam S_SEND_BYTE  = 4'b0100;
localparam S_STOP       = 4'b1000;


reg[3:0]                state;
reg[3:0]                next_state;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
// RX result byte
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
reg                     rx_q0;
reg                     rx_q1;
wire                    rx_negedge;
reg                     rx_start;                       // RX使能
reg[3:0]                rx_clk_edge_cnt;          		// clk沿的个数
reg                     rx_clk_edge_level;              // clk沿电平
reg                     rx_done;
reg[15:0]               rx_clk_cnt;
reg[15:0]               rx_div_cnt;
reg[7:0]                rx_data;
reg                     rx_over;
reg                     rx_rcv_ing;

// *************************** RX接收 ****************************

always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        state <= S_IDLE;
    end else begin
        state <= next_state;
    end
end

always @ (*) begin
    case (state)
        S_IDLE: begin
            if (rx_negedge) begin
                next_state = S_START;
            end else begin
                next_state = S_IDLE;
            end
        end
        S_START: begin
            if (rx_clk_edge_cnt == 4'd9) begin
                next_state = S_STOP;
            end else begin
                next_state = S_START;
            end
        end
        S_STOP: begin
            if (rx_over) begin
                next_state = S_IDLE;
            end else begin
                next_state = S_STOP;
            end
        end
        default: begin
            next_state = S_IDLE;
        end
    endcase
end


always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        rx_q0 <= 1'b0;
        rx_q1 <= 1'b0;	
    end else begin
        rx_q0 <= i_uart_rx;
        rx_q1 <= rx_q0;
    end
end

// 下降沿检测(检测起始信号)
assign rx_negedge = rx_q1 & (~rx_q0);

// 产生开始接收数据信号，接收期间一直有效
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        rx_start <= 1'b0;
    end else begin
        if (state==S_IDLE && rx_negedge) begin
            rx_start <= 1'b1;
        end else if (state==S_START && rx_clk_edge_cnt == 4'd9) begin
            rx_start <= 1'b0;
        end else if(state==S_STOP) begin
            rx_start <= 1'b0;
        end
    end
end

always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        rx_div_cnt <= 16'h0;
    end else begin
        // 第一个时钟沿只需波特率分频系数的一半
        if (rx_start == 1'b1 && rx_clk_edge_cnt == 4'h0) begin
            rx_div_cnt <= {1'b0, uart_baud[15:1]};
        end else begin
            rx_div_cnt <= uart_baud[15:0];
        end
    end
end

// 对时钟进行计数
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        rx_clk_cnt <= 16'h0;
    end else if (rx_start == 1'b1) begin
        // 计数达到分频值
        if (rx_clk_cnt == rx_div_cnt) begin
            rx_clk_cnt <= 16'h0;
        end else begin
            rx_clk_cnt <= rx_clk_cnt + 16'h1;
        end
    end else begin
        rx_clk_cnt <= 16'h0;
    end
end

// 每当时钟计数达到分频值时产生一个上升沿脉冲
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        rx_clk_edge_cnt <= 4'h0;
        rx_clk_edge_level <= 1'b0;
    end else if (rx_start == 1'b1) begin
        // 计数达到分频值
        if (rx_clk_cnt == rx_div_cnt) begin
            // 时钟沿个数达到最大值
            if (rx_clk_edge_cnt == 4'd9) begin
                rx_clk_edge_cnt <= 4'h0;
                rx_clk_edge_level <= 1'b0;
            end else begin
                // 时钟沿个数加1
                rx_clk_edge_cnt <= rx_clk_edge_cnt + 4'h1;
                // 产生上升沿脉冲
                rx_clk_edge_level <= 1'b1;
            end
        end else begin
            rx_clk_edge_level <= 1'b0;
        end
    end else begin
        rx_clk_edge_cnt <= 4'h0;
        rx_clk_edge_level <= 1'b0;
    end
end

// uart读入
// bit序列
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        rx_data <= 8'h0;
        rx_over <= 1'b0;
    end else begin
        if (rx_start == 1'b1) begin
            // 上升沿
            if (rx_clk_edge_level == 1'b1) begin
                case (rx_clk_edge_cnt)
                    // 起始位
                    1: begin

                    end
                    // 第1位数据位
                    2: begin
                        if (i_uart_rx) begin
                            rx_data <= 8'h80;
                        end else begin
                            rx_data <= 8'h0;
                        end
                    end
                    // 剩余数据位
                    3, 4, 5, 6, 7, 8, 9: begin
                        rx_data <= {i_uart_rx, rx_data[7:1]};
                        // 最后一位接收完成，置位接收完成标志
                        if (rx_clk_edge_cnt == 4'h9) begin
                            rx_over <= 1'b1;
                        end
                    end
                endcase
            end
        end else begin
            rx_data <= 8'h0;
            rx_over <= 1'b0;
        end
    end
end


wire                     f_tvalid = rx_over;
wire [7:0]               f_tdata  = rx_data;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
// RX fifo
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
wire f_tready;

generate if (FIFO_EA <= 0) begin          // no RX fifo
    
    assign       f_tready = o_tready;
    always @ (*) o_tvalid = f_tvalid;
    always @ (*) o_tdata  = f_tdata;

end else begin                            // TX fifo

    localparam        EA     = (FIFO_EA <= 2) ? 2 : FIFO_EA;

    reg  [7:0] buffer [ ((1<<EA)-1) : 0 ];

    localparam [EA:0] A_ZERO = {{EA{1'b0}}, 1'b0};
    localparam [EA:0] A_ONE  = {{EA{1'b0}}, 1'b1};

    reg  [EA:0] wptr      = A_ZERO;
    reg  [EA:0] wptr_d1   = A_ZERO;
    reg  [EA:0] wptr_d2   = A_ZERO;
    reg  [EA:0] rptr      = A_ZERO;
    wire [EA:0] rptr_next = (o_tvalid & o_tready) ? (rptr+A_ONE) : rptr;

    assign f_tready = ( wptr != {~rptr[EA], rptr[EA-1:0]} );

    always @ (posedge clk or negedge rstn)
        if (~rstn) begin
            wptr    <= A_ZERO;
            wptr_d1 <= A_ZERO;
            wptr_d2 <= A_ZERO;
        end else begin
            if (f_tvalid & f_tready)
                wptr <= wptr + A_ONE;
            wptr_d1 <= wptr;
            wptr_d2 <= wptr_d1;
        end

    always @ (posedge clk)
        if (f_tvalid & f_tready)
            buffer[wptr[EA-1:0]] <= f_tdata;

    always @ (posedge clk or negedge rstn)
        if (~rstn) begin
            rptr <= A_ZERO;
            o_tvalid <= 1'b0;
        end else begin
            rptr <= rptr_next;
            o_tvalid <= (rptr_next != wptr_d2);
        end

    always @ (posedge clk)
        o_tdata <= buffer[rptr_next[EA-1:0]];

    initial o_tvalid = 1'b0;
    initial o_tdata  = 8'h0;
end endgenerate



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
// detect RX fifo overflow
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
initial o_overflow = 1'b0;

always @ (posedge clk or negedge rstn)
    if (~rstn)
        o_overflow <= 1'b0;
    else
        o_overflow <= (f_tvalid & (~f_tready));




endmodule