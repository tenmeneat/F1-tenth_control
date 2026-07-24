#!/usr/bin/env python3
"""app.html + vendored sql.js → 최종 배포물 조립.
  webapp.html          : 독립 실행형(로컬/자체 호스팅용, 완전한 문서)
  webapp.fragment.html : claude.ai 아티팩트 게시용(본문 조각, doctype/head 없음)
"""
import os, re, base64

HERE = os.path.dirname(os.path.abspath(__file__))
app = open(os.path.join(HERE, "app.html")).read()
glue = open(os.path.join(HERE, "vendor", "sql-wasm.js")).read()
wasm_b64 = base64.b64encode(open(os.path.join(HERE, "vendor", "sql-wasm.wasm"), "rb").read()).decode()

full = app.replace("/*__SQLJS_GLUE__*/", glue).replace("__WASM_B64__", wasm_b64)

with open(os.path.join(HERE, "webapp.html"), "w") as f:
    f.write(full)

# 아티팩트용: <style> + <body> 내부만
style = re.search(r"<style>.*?</style>", full, re.S).group(0)
body = re.search(r"<body>(.*)</body>", full, re.S).group(1)
frag = style + "\n" + body
with open(os.path.join(HERE, "webapp.fragment.html"), "w") as f:
    f.write(frag)

print(f"webapp.html          {os.path.getsize(os.path.join(HERE,'webapp.html'))/1e6:.2f} MB")
print(f"webapp.fragment.html {os.path.getsize(os.path.join(HERE,'webapp.fragment.html'))/1e6:.2f} MB")
