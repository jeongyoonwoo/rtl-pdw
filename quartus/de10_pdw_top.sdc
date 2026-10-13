# de10_pdw_top.sdc  (rtl-pdw 7단계)
# 50MHz 보드 클록 하나만 쓴다.
create_clock -name clk_50 -period 20.000 [get_ports {CLOCK_50}]
derive_clock_uncertainty

# 버튼·스위치는 비동기 입력이고 내부에서 2단 동기화한다. LED·7세그먼트는 사람이 보는 출력이다.
set_false_path -from [get_ports {KEY[*] SW[*]}]
set_false_path -to   [get_ports {LEDR[*] HEX0[*] HEX1[*] HEX2[*] HEX3[*] HEX4[*] HEX5[*]}]
