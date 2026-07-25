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
base_lut = open(os.path.join(REPO, "control_code", "NUC6_glc_pacejka_lookup_table.csv")).read().strip()

# LUT는 <script type="text/plain">에 그대로 들어가므로 종료 태그만 없으면 안전
assert "</script" not in base_lut.lower(), "베이스 LUT에 </script>가 들어 있습니다"

full = (app.replace("/*__SQLJS_GLUE__*/", glue)
           .replace("__WASM_B64__", wasm_b64)
           .replace("__BASE_LUT__", base_lut))

with open(os.path.join(HERE, "webapp.html"), "w") as f:
    f.write(full)

# 아티팩트용: <style> + <body> 내부만
style = re.search(r"<style>.*?</style>", full, re.S).group(0)
body = re.search(r"<body>(.*)</body>", full, re.S).group(1)
with open(os.path.join(HERE, "webapp.fragment.html"), "w") as f:
    f.write(style + "\n" + body)

for n in ("webapp.html", "webapp.fragment.html"):
    print(f"{n:22s} {os.path.getsize(os.path.join(HERE, n)) / 1e6:.2f} MB")
