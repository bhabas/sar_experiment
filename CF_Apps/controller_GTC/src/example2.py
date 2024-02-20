from sympy import symbols, cos, sin, Matrix, pprint, simplify
from sympy import *

dy, dx, C = symbols('dy dx C')

Gamma = Matrix([
    [ 1, 1, 1, 1],
    [-dy, -dy, dy, dy],
    [-dx, dx, dx, -dx],
    [-C, C, -C, C]
])

Result = simplify(Gamma.inv())
pprint(4*Result)
