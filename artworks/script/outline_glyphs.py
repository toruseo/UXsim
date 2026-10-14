"""Print SVG path data for individually placed glyphs.

Usage: python outline_glyphs.py GROUP [GROUP ...]
GROUP is a comma-separated list of FONT.ttf:CHAR:X items; each GROUP becomes one path.
Coordinates are in font units with y flipped (baseline at y=0, y grows downwards).
"""
import sys

from fontTools.pens.boundsPen import BoundsPen
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen
from fontTools.ttLib import TTFont


def ntos(v):
    s = f"{v:.1f}"
    return s[:-2] if s.endswith(".0") else s


fonts = {}


def main():
    for group in sys.argv[1:]:
        pen = bounds = None
        for item in group.split(","):
            font_path, ch, x = item.rsplit(":", 2)
            if font_path not in fonts:
                fonts[font_path] = TTFont(font_path)
            tt = fonts[font_path]
            glyph_set = tt.getGlyphSet()
            if pen is None:
                pen, bounds = SVGPathPen(glyph_set, ntos=ntos), BoundsPen(glyph_set)
            glyph = glyph_set[tt.getBestCmap()[ord(ch)]]
            m = (1, 0, 0, -1, float(x), 0)
            glyph.draw(TransformPen(pen, m))
            glyph.draw(TransformPen(bounds, m))
        print(f"bbox={tuple(round(v, 1) for v in bounds.bounds)}")
        print(pen.getCommands())


if __name__ == "__main__":
    main()
