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
            "imemreq0_msg_addr": "PC_pipe0",
            "dpath.imemresp0_msg_data": "inst0",
            "val0_X0hl": "val_X0",
            "valA_X1hl": "valA_X1",
            "valA_X2hl": "valA_X2",
            "valA_X3hl": "valA_X3",
            "valA_Whl": "valA_W",
            "stall_X0hl": "stall_X0",
            "stall_X1hl": "stall_X1",
            "stall_X2hl": "stall_X2",
            "stall_X3hl": "stall_X3",
            "stall_Whl": "stall_W",
            "muldivresp_val": "mdiv_val",
            "sb_valA_X0": "sbA_X0",
            "sb_val_X1": "sb_X1",
            "wbA_mux_out_Whl": "wbA_out",
            "wbA_en_Whl": "wbA_en",
            "executeA_mux_out_X3hl": "exA_out",
            "opB0_byp_mux_sel_Dhl": "B0_byp_sel",
            "opB1_byp_mux_sel_Dhl": "B1_byp_sel",
            "opB0_byp_mux_out_Dhl": "B0_byp_out",
            "sb_regB0_stage_X0": "B0_sb_stage",
        }
        
        for line in f:
            line = line.strip()
            if not line: continue
            if state == 0:
                if line.startswith('$var'):
                    parts = line.split()
                    name = parts[4]
                    symbol = parts[3]
                    if name in targets:
                        symbol_to_name[symbol] = targets[name]
                        values[targets[name]] = 'x'
                elif line.startswith('$enddefinitions'):
                    state = 1
            else:
                if line.startswith('#'):
                    # end of previous time step
                    if current_time % 10 == 5: # rising edge check usually
                        cycle += 1
                        # We print if PC happens to be near div loop
                        pc_str = values.get("PC_pipe0", "x")
                        if pc_str != 'x':
                            try:
                                pc = int(pc_str, 2)
                                if 0x801e0 <= pc <= 0x80230:
                                    print(f"Cycle {cycle:3d} | PC: {hex(pc)} | MDivV: {values.get('mdiv_val')} | WB_en: {values.get('wbA_en')} | WB_out: {values.get('wbA_out')} | B0_sel: {values.get('B0_byp_sel')} B0_stage: {values.get('B0_sb_stage')} | BypOut: {values.get('B0_byp_out')}")
                            except:
                                pass
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
                        values[symbol_to_name[symbol]] = val

if __name__ == '__main__':
    vcd_file = sys.argv[1]
    parse_vcd_and_print_cycles(vcd_file)
