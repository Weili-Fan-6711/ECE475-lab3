import sys

def parse():
    with open(sys.argv[1], 'r') as f:
        sym_clk = None
        for line in f:
            if '$var' in line and ' clk ' in line:
                sym_clk = line.split()[3]
                break
        print("Clk symbol:", sym_clk)

        state = 0
        cycle = 0
        f.seek(0)
        
        for line in f:
             if '$enddefinitions' in line:
                 state = 1
                 continue
             if state == 1:
                 if line.startswith('1'+sym_clk):
                     cycle += 1
                     if cycle < 10:
                         print("Cycle", cycle)

parse()

