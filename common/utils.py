import matplotlib.pyplot as plt
import sympy as sp
from IPython.display import display, Math

def sympy_to_png(sympy, file_path="output.png", dpi=300, fontsize=16):
    latex_to_png(sp.latex(sympy), file_path, dpi, fontsize)

def latex_to_png(latex_str, file_path="output.png", dpi=300, fontsize=16):
    fig = plt.figure(figsize=(0.01, 0.01))
    formula = f"${latex_str}$" if not latex_str.startswith("$") else latex_str
    fig.text(0, 0, formula, fontsize=fontsize, usetex=False)
    fig.savefig(
        file_path,
        dpi=dpi,
        transparent=True,
        bbox_inches='tight',
        pad_inches=0.1
    )
    plt.close(fig)
    