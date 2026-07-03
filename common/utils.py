import matplotlib.pyplot as plt

def latex_to_png(latex_str, file_path="output.png", dpi=300, fontsize=16):
    """
    Renders a LaTeX string into a PNG image using Matplotlib's mathtext engine.
    """
    # Create a minimal figure
    fig = plt.figure(figsize=(0.01, 0.01))

    # Enclose the formula in $ signs if not already present
    formula = f"${latex_str}$" if not latex_str.startswith("$") else latex_str

    # Render the text into the figure
    fig.text(0, 0, formula, fontsize=fontsize, usetex=False)

    # Save with a transparent background and tight bounding box
    fig.savefig(
        file_path,
        dpi=dpi,
        transparent=True,
        bbox_inches='tight',
        pad_inches=0.1
    )
    plt.close(fig)
    