`default_nettype none

module ro #(
  parameter RO_SIZE = 9
) (
  input  wire en,
  output wire ro_clk
);

  (* keep = "true", dont_touch = "true" *) wire [RO_SIZE-1:0] ro_wire;

  (* keep = "true", dont_touch = "true" *)
  sky130_fd_sc_hd__nand2_1 nand_gate (
    .A(ro_wire[RO_SIZE-1]),
    .B(en),
    .Y(ro_wire[0])
  );

  genvar i;
  generate
    for (i = 1; i < RO_SIZE; i = i + 1) begin : inv_chain
      (* keep = "true", dont_touch = "true" *)
      sky130_fd_sc_hd__inv_1 inv_stage (
        .A(ro_wire[i-1]),
        .Y(ro_wire[i])
      );
    end
  endgenerate

  assign ro_clk = ro_wire[RO_SIZE-1];

endmodule
