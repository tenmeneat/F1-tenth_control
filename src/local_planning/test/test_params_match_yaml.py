#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
stuck_case_harness의 하드코딩 파라미터가 운영 YAML과 같은지 검사한다.

하니스는 "정지 교착이 왜 생겼나"를 판정하는 데 쓰이고, 그 결과로
safe_stop_buffer_m 같은 운영값을 정한다. 하니스가 조용히 다른 마진을 쓰면
아무도 묻지 않은 질문에 답하게 된다.

2026-08-14 리뷰에서 실제로 5개가 어긋나 있었다: vehicle_length 0.58 vs 0.56,
localization_reserve 0.12 vs 0.06, maximum_target_offset 1.20 vs 1.50,
target_d_candidate_count 3 vs 5, safety_margin 반올림. 뒤의 둘은 후보를 60개가
아니라 36개만, 그것도 좁은 오프셋 범위에서만 생성하게 만들어 하니스를 운영보다
비관적으로 만들었다.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
HARNESS = HERE / 'stuck_case_harness.cpp'
YAML = HERE.parent / 'config' / 'local_planning.yaml'

# 하니스가 명시하는 스칼라 파라미터 전부. 새 항목을 하니스에 추가하면 여기도 늘어난다
# (이름이 같으므로 자동으로 잡힌다).
IGNORE = frozenset({
    # 하니스 전용 오버라이드 대상 — argv로 덮어쓰므로 기본값 일치는 요구하지 않는다.
})
TOLERANCE = 1e-9


def scalars_from_harness(text):
    found = {}
    for match in re.finditer(r'^\s*p\.([a-z_0-9]+)\s*=\s*(-?[0-9][0-9.eE+-]*)\s*;', text, re.M):
        found[match.group(1)] = float(match.group(2))
    return found


def scalars_from_yaml(text):
    found = {}
    for match in re.finditer(r'^\s{2,}([a-z_0-9]+):\s*(-?[0-9][0-9.eE+-]*)\s*$', text, re.M):
        found[match.group(1)] = float(match.group(2))
    return found


def test_harness_parameters_match_operational_yaml():
    """하니스 진단 결과로 운영값을 정하므로 둘은 반드시 같아야 한다."""
    assert main() == 0


def main():
    harness = scalars_from_harness(HARNESS.read_text())
    yaml = scalars_from_yaml(YAML.read_text())
    if not harness:
        print('FAIL: 하니스에서 파라미터를 하나도 못 읽었다 — 정규식이 깨졌다', file=sys.stderr)
        return 1

    mismatches = []
    checked = 0
    for name, harness_value in sorted(harness.items()):
        if name in IGNORE or name not in yaml:
            continue
        checked += 1
        if abs(harness_value - yaml[name]) > TOLERANCE:
            mismatches.append((name, harness_value, yaml[name]))

    print(f'검사한 파라미터 {checked}개 (하니스 {len(harness)} / YAML {len(yaml)})')
    if mismatches:
        print('\nstuck_case_harness.cpp가 config/local_planning.yaml과 다릅니다:', file=sys.stderr)
        for name, harness_value, yaml_value in mismatches:
            print(f'  {name}: 하니스 {harness_value!r} vs YAML {yaml_value!r}', file=sys.stderr)
        print('\n하니스 진단 결과로 운영값을 정하므로 둘은 반드시 같아야 합니다.', file=sys.stderr)
        return 1
    if checked < 15:
        print(f'FAIL: {checked}개만 대조됐다 — 이름 규칙이 바뀌어 검사가 무력해졌을 수 있다',
              file=sys.stderr)
        return 1
    print('OK: 하니스와 운영 YAML이 일치합니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
