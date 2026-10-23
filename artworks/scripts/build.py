"""Builds uxsim/svg/uxsim-{hero,logo,icon}-{white,dark,transparent-light,transparent-dark,transparent-outline}.svg and exports uxsim/png/.

Usage (from any directory): python uxsim/scripts/build.py
Needs fontTools and uharfbuzz in tmp/pylib (see README.md) and Chrome for the PNGs.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [os.path.join(ROOT, "common"), os.path.join(ROOT, "tmp", "pylib")]
import glyphs  # noqa: E402
import export_png  # noqa: E402
import svgparts as sp  # noqa: E402

NAME = "uxsim"
TITLE = "UXsim"
HERO_TITLE = "UXsim: Python traffic simulator"
# Wordmark "UXsim": Poppins SemiBold with the X from Poppins SemiBold Italic, tracking -40; (font, character, x in font units), one path per group
WORDMARK = [[("SemiBold", "U", 0), ("SemiBoldItalic", "X", 657)], [("SemiBold", "s", 1303), ("SemiBold", "i", 1808), ("SemiBold", "m", 2046)]]
WM_COMMENT = "Wordmark: Poppins SemiBold with the X from Poppins SemiBold Italic (SIL OFL 1.1), tracking -40, converted to outlines; font units, baseline at y=0"
# Tagline of the hero: Poppins Regular; character ranges of the two paths ("Python" and the rest) and the placement (x, y, scale)
TAGLINE = "Python traffic simulator"
TAGLINE_GROUPS = [(0, 6), (6, 24)]
TAGLINE_TF = (582.714, 413.82, 0.0532499)


def main():
    wm_paths = [glyphs.glyphs(group) for group in WORDMARK]
    wm = [d for d, _ in wm_paths]
    wm_xmax = wm_paths[1][1][2]
    tag = [d for d, _ in glyphs.text("Regular", TAGLINE, TAGLINE_GROUPS)]
    svg_dir = os.path.join(ROOT, NAME, "svg")
    os.makedirs(svg_dir, exist_ok=True)
    for style in sp.STYLES:
        p = f"{NAME}-icon-{style}"
        sp.write(os.path.join(svg_dir, p + ".svg"), sp.icon_doc(p, style, TITLE))
        p = f"{NAME}-logo-{style}"
        sp.write(os.path.join(svg_dir, p + ".svg"), sp.logo_doc(p, style, TITLE, wm, wm_xmax, WM_COMMENT))
        p = f"{NAME}-hero-{style}"
        sp.write(os.path.join(svg_dir, p + ".svg"), sp.hero_doc(p, style, HERO_TITLE, wm, WM_COMMENT, tag, TAGLINE_TF))
    export_png.export(svg_dir, os.path.join(ROOT, NAME, "png"), os.path.join(ROOT, "tmp", "chrome-profile"))


if __name__ == "__main__":
    main()
