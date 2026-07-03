import sympy as sp
from IPython.display import display, Math

x = sp.Symbol('x')
expr = sp.Integral(sp.sqrt(1/x), x)

# Generate the LaTeX string
latex_str = sp.latex(expr)

# Display the LaTeX string as a rendered mathematical expression
display(Math(latex_str))
