#!/usr/bin/env python3
"""app.html + sql.js + 베이스 LUT → 최종 배포물 조립.
  webapp.html          : 독립 실행형(로컬/자체 호스팅용, 완전한 문서)
  webapp.fragment.html : claude.ai 아티팩트 게시용(본문 조각, doctype/head 없음)

sql.js는 bag_analyzer/webapp/vendor/를 그대로 재사용한다(심볼릭 링크).
"""
import base64
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", "..", ".."))

app = open(os.path.join(HERE, "app.html")).read()
glue = open(os.path.join(HERE, "vendor", "sql-wasm.js")).read()
wasm_b64 = base64.b64encode(open(os.path.join(HERE, "vendor", "sql-wasm.wasm"), "rb").read()).decode()
base_lut = open(os.path.join(REPO, "control_code", "LUT_calibrated.csv")).read().strip()
# 축 확장본(조향축 ~0.4320). UI에서 고를 수 있게 같이 심는다 — 기본은 표준이라
# CLI 기본값과의 바이트 동일성은 그대로 유지된다. 상세는 ../README.md 참고.
base_lut_ext = open(os.path.join(HERE, "..", "LUT_base_extended.csv")).read().strip()

# LUT는 <script type="text/plain">에 그대로 들어가므로 종료 태그만 없으면 안전
for name, txt in (("표준", base_lut), ("축 확장", base_lut_ext)):
    assert "</script" not in txt.lower(), f"베이스 LUT({name})에 </script>가 들어 있습니다"

full = (app.replace("/*__SQLJS_GLUE__*/", glue)
           .replace("__WASM_B64__", wasm_b64)
           .replace("__BASE_LUT_EXT__", base_lut_ext)   # ⚠️ __BASE_LUT__ 보다 먼저!
           .replace("__BASE_LUT__", base_lut))          #    (앞이 뒤의 접두사라 순서가 중요)

with open(os.path.join(HERE, "webapp.html"), "w") as f:
    f.write(full)

# 아티팩트용: <style> + <body> 내부만
style = re.search(r"<style>.*?</style>", full, re.S).group(0)
body = re.search(r"<body>(.*)</body>", full, re.S).group(1)
with open(os.path.join(HERE, "webapp.fragment.html"), "w") as f:
    f.write(style + "\n" + body)

for n in ("webapp.html", "webapp.fragment.html"):
    print(f"{n:22s} {os.path.getsize(os.path.join(HERE, n)) / 1e6:.2f} MB")
