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
            "wbA_mux_out_Whl": "wbA_out",
        }
        
        last_clk = '0'

        for line in f:
            line = line.strip()
            if not line: continue
            if state == 0:
                if line.startswith('$var'):
                    parts = line.split()
                    name = parts[4]
                    symbol = parts[3]
                    if name in targets and symbol not in symbol_to_name:
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
                            if cycle > 100 and cycle < 120:
                                p_hex = hex(int(pc_str, 2)) if 'x' not in pc_str and 'z' not in pc_str else 'x'
                                print(f"Cycle {cycle} | PC: {p_hex} | wbA: {values.get('wbA_out')}")
                            if cycle > 120: sys.exit(0)
                        if name == "clk":
                            last_clk = val

if __name__ == '__main__':
    vcd_file = sys.argv[1]
    parse_vcd_and_print_cycles(vcd_file)
