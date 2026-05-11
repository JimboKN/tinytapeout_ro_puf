`default_nettype none

module tt_um_jimbok_ro_puf(
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered, so you can ignore it
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);

  //////////////////// Parameters ////////////////////

  localparam SPI_WIDTH = 129;
  localparam COUNTER_WIDTH = 20;
  localparam RO_SIZE = 25;
  localparam NUM_ROS = 32; 

  //////////////////// Pin Configuration ////////////////////

  wire start = ui_in[0];

  reg done;
  reg busy;
  assign uo_out = {6'b0, busy, done};

  // SPI pin configuration
  wire spi_cs = uio_in[0];
  wire spi_mosi = uio_in[1];
  wire spi_sck = uio_in[3];

  wire spi_miso;
  assign uio_out = {5'b0, spi_miso, 2'b0};

  wire cs_active;
  assign uio_oe = {5'b0, cs_active, 2'b0};

  //////////////////// Sync Signals ////////////////////
  reg [2:0] sck_sync, start_sync, cs_sync;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sck_sync <= 3'b000;
      cs_sync <= 3'b111;
      start_sync <= 3'b000;
    end else begin
      sck_sync <= {sck_sync[1:0], spi_sck};
      cs_sync <= {cs_sync[1:0], spi_cs};
      start_sync <= {start_sync[1:0], start};
    end
  end

  wire sck_falling = (sck_sync[2:1] == 2'b10);
  assign cs_active = ~cs_sync[1];
  wire cs_falling = (cs_sync[2:1] == 2'b10);
  wire start_falling = (start_sync[2:1] == 2'b10);

  //////////////////// SPI Slave Logic ////////////////////

  reg [SPI_WIDTH-1:0] tx_data = 0;

  // SPI mode 0: MISO changes on falling SCK
  reg [SPI_WIDTH-1:0] tx_shift;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      tx_shift <= {SPI_WIDTH{1'b0}};
    else if (!cs_active)
      tx_shift <= tx_data;
    else if (sck_falling)
      tx_shift <= {tx_shift[SPI_WIDTH-2:0], 1'b0};
  end

  assign spi_miso = tx_shift[SPI_WIDTH-1];

  // suppress unused-signal warnings
  wire _unused = &{ena, spi_mosi, 1'b0};

  //////////////////// Ring Oscillators ////////////////////

  wire [NUM_ROS-1:0] ro_en;
  wire [NUM_ROS-1:0] ro_clk;

  ro #(
    .RO_SIZE(RO_SIZE)
  ) ring_osc [NUM_ROS-1:0] (
    .en (ro_en),
    .ro_clk (ro_clk)
  );

  //////////////////// Counters ////////////////////

  reg  [4:0] i_idx, j_idx;
  reg        ro_run;
  reg        cnt_rst;

  // route outputs of ROs to counter
  wire ro_i_clk = ro_clk[i_idx];
  wire ro_j_clk = ro_clk[j_idx];

  reg [COUNTER_WIDTH-1:0] cnt_i, cnt_j;

  always @(posedge ro_i_clk or posedge cnt_rst) begin
    if (cnt_rst)        cnt_i <= {COUNTER_WIDTH{1'b0}};
    else if (cnt_stop)  cnt_i <= cnt_i;
    else                cnt_i <= cnt_i + 1'b1;
  end

  always @(posedge ro_j_clk or posedge cnt_rst) begin
    if (cnt_rst)        cnt_j <= {COUNTER_WIDTH{1'b0}};
    else if (cnt_stop)  cnt_j <= cnt_j;
    else                cnt_j <= cnt_j + 1'b1;
  end

  wire msb_i = cnt_i[COUNTER_WIDTH-1];
  wire msb_j = cnt_j[COUNTER_WIDTH-1];

  // Stop counters as soon as MSB is filled.
  // Sync to main clock domain to signal that count is finished
  wire cnt_stop = msb_i || msb_j;
  reg [1:0] cnt_stop_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cnt_stop_sync <= 2'b00;
    else        cnt_stop_sync <= {cnt_stop_sync[0], cnt_stop};
  end

  // ro_run ensures ROs start at the same time
  reg [NUM_ROS-1:0] ro_en_reg;
  assign ro_en = ro_en_reg;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ro_en_reg <= {NUM_ROS{1'b0}};
    end
    else if (ro_run) begin
      ro_en_reg[i_idx] <= 1'b1;
      ro_en_reg[j_idx] <= 1'b1;
    end
    else ro_en_reg <= {NUM_ROS{1'b0}};
  end

  //////////////////// Lehmer-digit width lookup ////////////////////

  wire [2:0] lehmer_shift = (i_idx <= 15) ? 3'd5 :
                            (i_idx <= 23) ? 3'd4 :
                            (i_idx <= 27) ? 3'd3 :
                            (i_idx <= 29) ? 3'd2 : 3'd1;


  //////////////////// FSM ////////////////////
  localparam [2:0]
    S_IDLE    = 3'd0,
    S_RESET_C = 3'd1,
    S_COUNT   = 3'd2,
    S_NEXT    = 3'd3,
    S_FINISH  = 3'd4;

  reg [2:0] state;
  reg [4:0] lehmer_acc;
  reg [2:0] rst_timer;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= S_IDLE;
      i_idx      <= 5'd0;
      j_idx      <= 5'd1;
      lehmer_acc <= 5'd0;
      rst_timer  <= 3'd0;
      tx_data    <= {SPI_WIDTH{1'b0}};
      done       <= 1'b0;
      busy       <= 1'b0;
      ro_run     <= 1'b0;
      cnt_rst    <= 1'b1;
    end else begin
      case (state)
        // ---------------------------------------------------------
        S_IDLE: begin
          ro_run  <= 1'b0;
          cnt_rst <= 1'b1;
          if (cs_falling) begin
            done <= 1'b0;
          end
          if (start_falling) begin
            i_idx      <= 5'd0;
            j_idx      <= 5'd1;
            lehmer_acc <= 5'd0;
            tx_data    <= {SPI_WIDTH{1'b0}};
            done       <= 1'b0;
            busy       <= 1'b1;
            rst_timer  <= 3'd0;
            cnt_rst    <= 1'b0;
            state      <= S_RESET_C;
          end
        end

        // ---------------------------------------------------------
        // Hold counters in async reset for 4 clk cycles to ensure RO stability
        S_RESET_C: begin
          cnt_rst <= 1'b1;
          ro_run  <= 1'b0;
          if (&rst_timer) begin
            cnt_rst <= 1'b0;
            ro_run  <= 1'b1;
            state   <= S_COUNT;
          end else begin
            rst_timer <= rst_timer + 1'b1;
          end
        end

        // ---------------------------------------------------------
        // Race RO_i against RO_j
        // Count number of times RO_i is faster
        S_COUNT: begin
          ro_run <= 1'b1;
          if (cnt_stop_sync[1]) begin
            ro_run <= 1'b0;
            if (msb_i && !msb_j)
              lehmer_acc <= lehmer_acc + 1'b1;
            state <= S_NEXT;
          end
        end

        // ---------------------------------------------------------
        // Move to next pair, or pack and advance i.
        S_NEXT: begin
          if (j_idx == 5'd31) begin
            // All j>i compared. Shift lehmer_acc into tx_data,
            // packed at its variable bit-width.
            case (lehmer_shift)
              3'd5: tx_data <= {tx_data[SPI_WIDTH-6:0], lehmer_acc[4:0]};
              3'd4: tx_data <= {tx_data[SPI_WIDTH-5:0], lehmer_acc[3:0]};
              3'd3: tx_data <= {tx_data[SPI_WIDTH-4:0], lehmer_acc[2:0]};
              3'd2: tx_data <= {tx_data[SPI_WIDTH-3:0], lehmer_acc[1:0]};
              3'd1: tx_data <= {tx_data[SPI_WIDTH-2:0], lehmer_acc[0]};
              default: tx_data <= tx_data;
            endcase

            if (i_idx == 5'd30) begin
              state <= S_FINISH;
            end else begin
              i_idx      <= i_idx + 5'd1;
              j_idx      <= i_idx + 5'd2;   // (new_i)+1
              lehmer_acc <= 5'd0;
              rst_timer  <= 3'd0;
              cnt_rst    <= 1'b1;
              state      <= S_RESET_C;
            end
          end else begin
            j_idx     <= j_idx + 5'd1;
            rst_timer <= 3'd0;
            cnt_rst   <= 1'b1;
            state     <= S_RESET_C;
          end
        end

        // ---------------------------------------------------------
        S_FINISH: begin
          done    <= 1'b1;
          busy    <= 1'b0;
          ro_run  <= 1'b0;
          cnt_rst <= 1'b0;
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end



endmodule