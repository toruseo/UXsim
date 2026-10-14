"""Print SVG path data for text set in a font (HarfBuzz shaping, fontTools outlines).

Coordinates are in font units with y flipped (baseline at y=0, y grows downwards).
Usage: python outline_text.py FONT.ttf "text" start:end [start:end ...]
Each start:end is a character range that becomes one output path.
"""
import sys

import uharfbuzz as hb
from fontTools.pens.boundsPen import BoundsPen
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen
from fontTools.ttLib import TTFont


def ntos(v):
    s = f"{v:.1f}"
    return s[:-2] if s.endswith(".0") else s


def main():
    font_path, text = sys.argv[1], sys.argv[2]
    groups = [tuple(int(v) for v in g.split(":")) for g in sys.argv[3:]]
    face = hb.Face(hb.Blob.from_file_path(font_path))
    font = hb.Font(face)
    buf = hb.Buffer()
    buf.add_str(text)
    buf.guess_segment_properties()
    hb.shape(font, buf, {})
    tt = TTFont(font_path)
    glyph_set = tt.getGlyphSet()
    order = tt.getGlyphOrder()
    pens = [SVGPathPen(glyph_set, ntos=ntos) for _ in groups]
    bounds = [BoundsPen(glyph_set) for _ in groups]
    x = 0
    for info, pos in zip(buf.glyph_infos, buf.glyph_positions):
        name = order[info.codepoint]
        k = next(i for i, (a, b) in enumerate(groups) if a <= info.cluster < b)
        m = (1, 0, 0, -1, x + pos.x_offset, -pos.y_offset)
        glyph_set[name].draw(TransformPen(pens[k], m))
        glyph_set[name].draw(TransformPen(bounds[k], m))
        x += pos.x_advance
    os2 = tt["OS/2"]
    print(f"upm={tt['head'].unitsPerEm} cap={os2.sCapHeight} xh={os2.sxHeight} advance={x}")
    for (a, b), p, bp in zip(groups, pens, bounds):
        print(f"[{text[a:b]!r}] bbox={tuple(round(v, 1) for v in bp.bounds)}")
        print(p.getCommands())


if __name__ == "__main__":
    main()
