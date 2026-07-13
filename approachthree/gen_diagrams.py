"""Generate approach-three diagram artifacts."""

from pathlib import Path

from approachthree.continuous import analyze, save_bode_plot

OUTPUT = Path(__file__).with_name("assets") / "continuous_bode.png"


if __name__ == "__main__":
    save_bode_plot(analyze(), OUTPUT)
    print(OUTPUT)
