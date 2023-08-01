# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn-v0_8")
import sympy
sympy.init_printing(pretty_print=False)

a, b, c, d, x = sympy.symbols('a, b, c, d, x', real=True)
cutoff, threshold = sympy.symbols('cutoff, threshold', real=True)

f = a*x**3 + b*x**2 + c*x + d
df = f.diff(x)

sol = sympy.solve([
    f.subs({x: cutoff}), 
    df.subs({x: cutoff}), 
    f.subs({x: threshold}) - 1, 
    df.subs({x: threshold})
    ], [a, b, c, d])

cse = sympy.cse(f.subs(sol))
for k, v in cse[0]:
    print("const double %s = %s" % (k, v))
print("const double f = %s" % cse[1][0])

def f_eval(x_, cutoff_, threshold_):
    if x_ < cutoff_:
        return 0.0
    elif x_ > threshold_:
        return 1.0
    else:
        sym = {}
        for k, s in sol.items():
            sym[k] = s.subs({cutoff: cutoff_, threshold: threshold_})
        sym[x] = x_
        return f.subs(sym)
    
    
x_axis = np.linspace(0.0, 1.0, 1000)
y_axis = np.array([f_eval(x, 0.25, 0.75) for x in x_axis])

plt.close()
plt.plot(x_axis, y_axis)


