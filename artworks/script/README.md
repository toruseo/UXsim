# UXsim ロゴ一式の生成スクリプト

プロジェクト直下の `uxsim-{hero,logo,icon}-{white,dark,transparent-light,transparent-dark,transparent-outline}.svg` と `png/` を作るためのスクリプトとフォントである。
SVG全体を組み立てるスクリプトはない。
各スクリプトは座標とパスデータを標準出力に出すだけで、SVGにはその出力を手で書き込んでいる。
以下のコマンドはすべてプロジェクト直下で実行する。

## 準備

文字のアウトライン化にはfontToolsとuharfbuzzが必要である。
ユーザー環境を変えないため、`tmp/pylib` に入れる（`tmp/` を消したら入れ直す）。

```bash
python -m pip install --target tmp/pylib --no-cache-dir --disable-pip-version-check --no-warn-script-location fonttools uharfbuzz
```

## ファイル

- `gen_mark.py`：マーク（ウロボロス）の黄・青のパス、噛み跡のclipPath、中央破線とdasharray、目の座標。512×512の座標系で、アイコン・ロゴ・ヒーローに共通。
- `outline_glyphs.py`：字形を指定位置に置いてアウトライン化する（ワードマーク「UXsim」用）。
- `outline_text.py`：文字列をHarfBuzzで組んでアウトライン化する（タグライン用）。
- `export_png.sh`：全SVGを `png/` に等倍と@2xで書き出す（headless Chrome）。
- `fonts/`：Poppins Regular・SemiBold・SemiBoldItalic（SIL OFL 1.1、ライセンスは `fonts/OFL.txt`）。

## 実行

```bash
python script/gen_mark.py
F=script/fonts; PYTHONPATH=tmp/pylib python script/outline_glyphs.py "$F/Poppins-SemiBold.ttf:U:0,$F/Poppins-SemiBoldItalic.ttf:X:657" "$F/Poppins-SemiBold.ttf:s:1303,$F/Poppins-SemiBold.ttf:i:1808,$F/Poppins-SemiBold.ttf:m:2046"
PYTHONPATH=tmp/pylib python script/outline_text.py script/fonts/Poppins-Regular.ttf "Python traffic flow simulator" 0:6 6:29
bash script/export_png.sh
```

文字の出力はフォント単位（UPM 1000、y軸下向き、ベースラインy=0）で、SVGでは下の `transform` で縮小して置いている。
ワードマークの2つのパスは「UX」と「sim」、タグラインの2つのパスは「Python」と「 traffic flow simulator」である。

## SVGでの配置

| ファイル | マーク | ワードマーク | タグライン |
|---|---|---|---|
| hero（1280×640） | translate(32.676 56.422) scale(1.0296) | translate(572.947 353.637) scale(0.206961) | translate(583.397 431.419) scale(0.0443788) |
| logo-transparent-*（610×148） | translate(-11.333 -11.333) scale(0.333333) | translate(173.442 118) scale(0.142653) | なし |
| logo-white・logo-dark（682×220） | 上の2つを translate(36 36) で包む | 同左 | なし |
| icon（512×512） | そのまま | なし | なし |

## 配色

| 要素 | 明るい背景用（white・transparent-light） | 暗い背景用（dark・transparent-dark） |
|---|---|---|
| UX | #306998 | #FFFFFF |
| sim | #3B3B3B | #FFD43B |
| タグライン | #6A6A6A（Pythonは#306998） | #A9BCD3（Pythonは#FFD43B） |

マークの青は #5A9FD4→#306998、黄は #FFE873→#FFD43B の線形グラデーション（userSpaceOnUse、(60,60)→(452,452)）で、全スタイル共通である。

## transparent-outline（あらゆる背景用）

transparent-lightに、マークと文字を囲む白の細い縁取りを加えたものである。
縁の太さは1倍表示で、heroのマークとワードマークが1.5px、heroのタグラインとlogoが1px、iconが4pxである。
マークは、白で塗って線幅2wの白線を付けた黄と青のパスを、元のマークの下に敷いている（wはマーク座標での縁の太さ）。
下敷きの黄は、噛み跡をwだけ小さくしたclipPathで切り抜き、噛まれた縁にも白が付くようにしている（`gen_mark.py` の BITE_OUTLINE）。
文字は、文字を包む `<g>` に `stroke="#fff"` と `paint-order="stroke"` を付け、塗りの外側にだけ白線が出るようにしている。
文字の線幅はフォント単位で、hero ワードマーク14.5、hero タグライン45、logo ワードマーク14である（縁の太さ×2÷縮尺）。
