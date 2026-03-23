import sys
import re

def vcd_parser(vcd_file, signal_names_to_track):
    symbol_to_name = {}
    name_to_symbol = {}
    
    with open(vcd_file, 'r') as f:
        # State: 0: defs, 1: dump
        state = 0
        current_time = 0
        
        # We will keep track of values for symbols we care about
        values = {}
        for line in f:
            line = line.strip()
            if not line: continue
            
            if state == 0:
                if line.startswith('$var'):
                    parts = line.split()
                    type = parts[1]
                    size = int(parts[2])
                    symbol = parts[3]
                    name = parts[4]
                    
                    if name in signal_names_to_track or True: # Capture all just in case, but prefer strict name matching
                        if parts[4] in signal_names_to_track:
                            symbol_to_name[symbol] = name
                            name_to_symbol[name] = symbol
                            values[symbol] = 'x'
                elif line.startswith('$enddefinitions'):
                    state = 1
            else:
                if line.startswith('#'):
                    # New timestamp
                    current_time = int(line[1:])
                    # if current_time > 150*10: break
                elif line.startswith('b') or line.startswith('B'):
                    parts = line.split()
                    val = parts[0][1:]
                    symbol = parts[1] if len(parts) > 1 else line.split()[0][-1:] # Handle 'b0 X' vs 'b0X' if no space, standard has space
                    if ' ' not in line:
                        # VCD standard: bVALUE symbol
                        val = line[1:-1]
                        symbol = line[-1]
                    else:
                        val = parts[0][1:]
                        symbol = parts[1]
                    if symbol in symbol_to_name:
                        values[symbol] = val
                        print(f"Time {current_time:5d} | {symbol_to_name[symbol]:20s} = {val}")
                elif line[0] in '01xXzZ':
                    val = line[0]
                    symbol = line[1:]
                    if symbol in symbol_to_name:
                        values[symbol] = val
                        print(f"Time {current_time:5d} | {symbol_to_name[symbol]:20s} = {val}")

if __name__ == '__main__':
    vcd_file = sys.argv[1]
    signals = ["clk", "imemreq0_msg_addr", "dpath.imemresp0_msg_data", "inst0_X0hl", "stall_Xhl", "stall_Mhl", "stall_X2hl", "stall_X3hl", "muldivresp_val", "result3_reg", "sb_stage_X0", "sb_stage_X1", "sb_val0"]
    vcd_parser(vcd_file, signals)

