import sys

def parse_vcd_and_print_cycles(vcd_file):
    symbol_to_name = {}
    
    with open(vcd_file, 'r') as f:
        state = 0
        current_time = 0
        cycle = 0
        values = {}
        
        # signals to capture
        targets = {
            "clk": "clk",
            "imemreq0_msg_addr": "PC",
            "imemresp0_msg_data": "inst",
            "muldivresp_val": "mdiv_val",
            "wbA_mux_out_Whl": "wbA_out",
            "stall_X0hl": "s_X0",
            "stall_X1hl": "s_X1",
            "stall_X2hl": "s_X2",
            "stall_X3hl": "s_X3",
            "stall_Whl": "s_W",
            "opB0_byp_mux_sel_Dhl": "bypB0",
            "opB1_byp_mux_sel_Dhl": "bypB1",
            "opA0_byp_mux_sel_Dhl": "bypA0",
            "muldivresp_msg_result_X3hl": "mdiv_res",
            "wbB_mux_out_Whl": "wbB_out",
            "sb_stage_X0": "sb_stage_X0",
            "sb_valA_X0": "sbA_X0"
        }
        
        # Track previous clock to detect posedge
        last_clk = '0'
        
        def safe_hex(v):
            if v == 'x' or 'x' in v or 'z' in v: return v.rjust(8)
            return hex(int(v, 2))[2:].rjust(8)

        def safe_bin_int(v):
            if v == 'x' or 'x' in v or 'z' in v: return v
            return str(int(v, 2))

        print("Cycle |   Time |       PC |     Inst | MDivV | sX0 sX1 sX2 sX3 sW | bypA0 bypB0 bypB1 | wbA_out")
        print("-" * 100)

        for line in f:
            line = line.strip()
            if not line: continue
            if state == 0:
                if line.startswith('$var'):
                    parts = line.split()
                    name = parts[4]
                    symbol = parts[3]
                    if name in targets:
                        # Only take first mapping if multiple
                        if symbol not in symbol_to_name:
                            symbol_to_name[symbol] = targets[name]
                            values[targets[name]] = 'x'
                elif line.startswith('$enddefinitions'):
                    state = 1
            else:
                if line.startswith('#'):
                    current_time = int(line[1:])
                elif line.startswith('b') or line.startswith('B'):
                    if ' ' not in line:
                        val = line[1:-1]
                        symbol = line[-1]
                    else:
                        parts = line.split()
                        val = parts[0][1:]
                        symbol = parts[1]
                    if symbol in symbol_to_name:
                        values[symbol_to_name[symbol]] = val
                elif line[0] in '01xXzZ':
                    val = line[0]
                    symbol = line[1:]
                    if symbol in symbol_to_name:
                        name = symbol_to_name[symbol]
                        values[name] = val
                        
                        if name == "clk" and val == '1' and last_clk == '0':
                            cycle += 1
                            pc_str = values.get("PC", "x")
                            if pc_str != 'x' and 'x' not in pc_str:
                                p = int(pc_str, 2)
                                if 0x801d0 <= p <= 0x80230:
                                    print(f"{cycle:5d} | {current_time:6d} | {hex(p)[2:]:8s} | {safe_hex(values.get('inst', 'x'))} | "
                                          f"  {values.get('mdiv_val', 'x')}   |  {values.get('s_X0','x')}   {values.get('s_X1','x')}   {values.get('s_X2','x')}   {values.get('s_X3','x')}  {values.get('s_W','x')} | "
                                          f"  {safe_bin_int(values.get('bypA0', 'x')):3s}   {safe_bin_int(values.get('bypB0', 'x')):3s}   {safe_bin_int(values.get('bypB1', 'x')):3s} | "
                                          f"{safe_hex(values.get('wbA_out', 'x'))}")
                        if name == "clk":
                            last_clk = val

if __name__ == '__main__':
    vcd_file = sys.argv[1]
    parse_vcd_and_print_cycles(vcd_file)
