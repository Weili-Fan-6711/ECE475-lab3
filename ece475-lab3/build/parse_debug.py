import sys
def parse():
    targets = { "clk": "clk", "pcA_Dhl": "pAD", "pcB_Dhl": "pBD", "pcA_X0hl": "pA0", "pcB_X0hl": "pB0", "pcA_X1hl": "pA1", "pcB_X1hl": "pB1" }
    s2n={} ; n2s={}
    with open(sys.argv[1], 'r') as f:
        for l in f:
            if l.startswith('$var '):
                p = l.split(); name=p[4]; sym=p[3]
                if name in targets and name not in n2s:
                    s2n[sym]=targets[name]; n2s[targets[name]]=sym
            elif l.startswith('$enddef'): break
    vals={v:'x' for v in targets.values()}; cyc=0; scl=n2s.get('clk')
    last_clk = '0'
    out_lines = []
    with open(sys.argv[1], 'r') as f:
        for l in f:
            if l.startswith('#'): continue
            if l.startswith('b') or l.startswith('B'):
                if ' ' in l: v,s=l.split(); v=v[1:]
                else: v=l[1:-1]; s=l[-1]
                if s in s2n: vals[s2n[s]]=v
            elif l and l[0] in '01xXzZ':
                v=l[0]; s=l[1:].strip()
                if s in s2n:
                    if s==scl:
                        if v=='1' and last_clk in '0xX':
                            cyc+=1
                            if cyc>=168 and cyc<=178:
                                pAD = hex(int(vals['pAD'],2))[2:] if 'x' not in vals['pAD'] else 'x'
                                pBD = hex(int(vals['pBD'],2))[2:] if 'x' not in vals['pBD'] else 'x'
                                pA0 = hex(int(vals['pA0'],2))[2:] if 'x' not in vals['pA0'] else 'x'
                                pB0 = hex(int(vals['pB0'],2))[2:] if 'x' not in vals['pB0'] else 'x'
                                out_lines.append(f"[C {cyc}] pAD:{pAD} pBD:{pBD} pA0:{pA0} pB0:{pB0}")
                        last_clk = v
                    else:
                        vals[s2n[s]]=v
    with open('output.txt', 'w') as f: f.write('\n'.join(out_lines))
parse()
