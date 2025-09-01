
module uart_tx #(
   // TX fifo depth
    parameter  FIFO_EA                   = 0           // 0:no fifo   1,2:depth=4   3:depth=8   4:depth=16  ...  10:depth=1024   11:depth=2048  ...
) (
    input  wire                    rstn,
    input  wire                    clk,
    // input  slave. Associated clock = clk
    output wire                    o_tready,
    input  wire                    i_tvalid,
    input  wire [8-1:0]            i_tdata,
    input  wire [1-1:0]            i_tkeep,
    input  wire                    i_tlast,
    // UART TX output signal
    output wire                    o_uart_tx
);



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
// TX fifo
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
wire                    f_tready;
reg                     f_tvalid;
reg  [8-1:0]            f_tdata;
reg  [  1-1:0]          f_tkeep;
reg                     f_tlast;

generate if (FIFO_EA <= 0) begin          // no TX fifo

    assign       o_tready = f_tready;
    always @ (*) f_tvalid = i_tvalid;
    always @ (*) f_tdata  = i_tdata;
    always @ (*) f_tkeep  = i_tkeep;
    always @ (*) f_tlast  = i_tlast;

end else begin                            // TX fifo

    localparam        EA     = (FIFO_EA<=2) ? 2 : FIFO_EA;
    localparam        DW     = ( 1 + 1 + 8 );     // 1-bit tlast, (1)-bit tkeep, (8)-bit tdata
    
    reg  [DW-1:0] buffer [ ((1<<EA)-1) : 0 ];
    
    localparam [EA:0] A_ZERO = {{EA{1'b0}}, 1'b0};
    localparam [EA:0] A_ONE  = {{EA{1'b0}}, 1'b1};

    reg  [EA:0] wptr      = A_ZERO;
    reg  [EA:0] wptr_d1   = A_ZERO;
    reg  [EA:0] wptr_d2   = A_ZERO;
    reg  [EA:0] rptr      = A_ZERO;
    wire [EA:0] rptr_next = (f_tvalid & f_tready) ? (rptr+A_ONE) : rptr;
    
    assign o_tready = ( wptr != {~rptr[EA], rptr[EA-1:0]} );

    always @ (posedge clk or negedge rstn)
        if (~rstn) begin
            wptr    <= A_ZERO;
            wptr_d1 <= A_ZERO;
            wptr_d2 <= A_ZERO;
        end else begin
            if (i_tvalid & o_tready)
                wptr <= wptr + A_ONE;
            wptr_d1 <= wptr;
            wptr_d2 <= wptr_d1;
        end

    always @ (posedge clk)
        if (i_tvalid & o_tready)
            buffer[wptr[EA-1:0]] <= {i_tlast, i_tkeep, i_tdata};

    always @ (posedge clk or negedge rstn)
        if (~rstn) begin
            rptr <= A_ZERO;
            f_tvalid <= 1'b0;
        end else begin
            rptr <= rptr_next;
            f_tvalid <= (rptr_next != wptr_d2);
        end

    always @ (posedge clk)
        {f_tlast, f_tkeep, f_tdata} <= buffer[rptr_next[EA-1:0]];
    
    initial {f_tvalid, f_tlast, f_tkeep, f_tdata} = 0;
    
end endgenerate




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
reg[15:0]               cycle_cnt;
reg                     tx_bit;
reg[3:0]                bit_cnt;


assign f_tready = state==S_STOP && next_state==S_IDLE;

// *************************** TX发送 ****************************

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
            if (f_tvalid) begin
                next_state = S_START;
            end else begin
                next_state = S_IDLE;
            end
        end
        S_START: begin
            if (cycle_cnt == uart_baud[15:0]) begin
                next_state = S_SEND_BYTE;
            end else begin
                next_state = S_START;
            end
        end
        S_SEND_BYTE: begin
            if ((cycle_cnt == uart_baud[15:0]) & (bit_cnt == 4'd7)) begin
                next_state = S_STOP;
            end else begin
                next_state = S_SEND_BYTE;
            end
        end
        S_STOP: begin
            if (cycle_cnt == uart_baud[15:0]) begin
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

// cycle_cnt
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        cycle_cnt <= 16'h0;
    end else begin
        if (state == S_IDLE) begin
            cycle_cnt <= 16'h0;
        end else begin
            if (cycle_cnt == uart_baud[15:0]) begin
                cycle_cnt <= 16'h0;
            end else begin
                cycle_cnt <= cycle_cnt + 16'h1;
            end
        end
    end
end

// bit_cnt
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        bit_cnt <= 4'h0;
    end else begin
        case (state)
            S_IDLE: begin
                bit_cnt <= 4'h0;
            end
            S_SEND_BYTE: begin
                if (cycle_cnt == uart_baud[15:0]) begin
                    bit_cnt <= bit_cnt + 4'h1;
                end
            end
            // ?这里不加default什么的吗
        endcase
    end
end

// tx_bit
always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        tx_bit <= 1'b0;
    end else begin
        case (state)
            S_IDLE, S_STOP: begin
                tx_bit <= 1'b1;
            end
            S_START: begin
                tx_bit <= 1'b0;
            end
            S_SEND_BYTE: begin
                tx_bit <= f_tdata[bit_cnt];
            end
            // ?这里不加default什么的吗
        endcase
    end
end
assign o_uart_tx = tx_bit;

endmodule