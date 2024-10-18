def units_to_mm(x):
    return x/1000000

def kcpt2pt(pt):
    return [units_to_mm(pt[0]), units_to_mm(pt[1])]
