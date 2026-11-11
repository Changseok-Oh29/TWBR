# 실행 및 결과 해석

모든 명령은 TWBR 저장소 루트에서 실행합니다. 설치와 기본 계산 명령은 [README](../README.md)를 참고하세요.

## 동역학 계산과 저장

```bash
# 부유 베이스 계산 및 결과 저장
.venv/bin/python scripts/compute_dynamics.py --output outputs/neutral_floating.npz

# 몸체를 고정한 모델과 비교
.venv/bin/python scripts/compute_dynamics.py --base fixed --output outputs/neutral_fixed.npz

# 다른 URDF를 지정할 때
.venv/bin/python scripts/compute_dynamics.py --urdf /path/to/robot.urdf
```

`outputs/`는 저장 명령을 실행할 때 생성됩니다. 저장된 행렬은 해당 자세·속도에서의 계산 결과이며,
다른 상태에서는 계산 함수를 다시 호출합니다.

## 3D 뷰어

연구 목표는 주행 바퀴 두 개로만 지면에 접촉하고 캐스터는 떠 있는 균형 상태입니다.
그 전에 URDF에 정의된 형상과 관절 방향부터 확인합니다.

시각화 의존성을 설치한 뒤 실행합니다.

```bash
.venv/bin/python -m pip install -e '.[visualization]'
.venv/bin/python scripts/view_robot.py --export outputs/robot_preview.html
```

터미널에 출력된 주소(예: `http://127.0.0.1:7000/static/`)를 같은 컴퓨터의 브라우저에서 엽니다.
원격 환경에서는 해당 포트를 전달하거나, 저장된 HTML을 로컬 브라우저에서 엽니다.
로컬 뷰어는 스크립트가 실행되는 동안 사용할 수 있으며, `quit` 또는 Ctrl+C로 종료합니다.

- URDF가 참조하는 STL 13개를 표시합니다.
- 기본 영점 자세에서 몸체를 `z = 0.35 m`에 띄웁니다. 이 높이는 형상 검토용이며 지면 접촉을 계산한 결과가 아닙니다.
- 빨간 점: 전체 무게중심. 몸체 내부에 있어도 확인할 수 있게 앞쪽에 겹쳐 표시합니다.
- 주황 점: 무게중심의 지면 수직 투영점.
- 좌표축: X 빨강, Y 초록, Z 파랑. 지면의 높이는 `z = 0`입니다.
- 마우스로 회전·확대/축소할 수 있습니다. 뷰어의 표시 제어 메뉴에서 링크의 표시 여부를 조정할 수 있습니다.

스크립트의 `twbr-viewer>` 프롬프트에 아래 명령을 입력하면 화면이 갱신됩니다.
이 명령들은 bash 명령이 아닙니다. 각도 단위는 도(degree), 관절 각도는 절대값입니다.

```text
joint left_shoulder_joint -10
joint left_wheel_joint 30
reset
save outputs/current_pose.html
quit
```

명령은 한 줄씩 입력하고 화면을 확인합니다. 줄 끝에 `\`를 붙이지 않습니다.
`reset`을 바로 입력하면 변경한 자세가 즉시 초기화됩니다.
현재 URDF의 shoulder 관절 범위는 약 -43.6°부터 0°까지입니다.
허용 범위를 벗어나거나 존재하지 않는 관절 이름을 입력하면 자세를 변경하지 않습니다.
실물에 폐루프 연결이 있다면 현재 독립 관절 조작은 그 연결을 만족하지 않을 수 있습니다.
이 기능은 URDF 연결 관계를 검사하기 위한 것이며 실물 관절 명령이 아닙니다.

서버를 계속 켜 두지 않고 HTML만 만들 수도 있습니다.

```bash
.venv/bin/python scripts/view_robot.py --export outputs/robot_preview.html --no-interactive
```

HTML은 현재 자세의 3D 장면을 포함하며 서버 없이 회전·확대할 수 있습니다.
저장 후 라이브 뷰어에서 관절을 바꿔도 기존 HTML은 갱신되지 않으므로 다시 저장해야 합니다.
원본 STL을 포함해 약 55 MB이며 처음 열 때 시간이 걸릴 수 있습니다.

이 단계에서는 중력에 따른 운동, 접촉력, 구름 조건, 균형 제어를 계산하지 않습니다.
다음 단계는 실제 기구 연결을 확인한 뒤 두 바퀴의 접촉과 캐스터 간격을 만족하는 기준 자세를 찾는 것입니다.

## 계산 결과 읽기

기본 실행은 `pin.neutral(model)`의 영점 자세와 `v = 0`을 사용합니다.
부유 베이스의 위치는 원점, 회전은 항등 회전이며 중력은 월드 좌표계의 `[0, 0, -9.81] m/s²`입니다.
이 자세는 계산 기준일 뿐, 지면에 서 있거나 균형을 유지하는 자세가 아닙니다.

| 항목 | 고정 베이스 | 부유 베이스 |
|---|---:|---:|
| 구성 벡터 크기 `nq` | 18 | 25 |
| 속도 벡터 크기 `nv` | 12 | 18 |
| 질량행렬 크기 | 12 × 12 | 18 × 18 |
| 전체 로봇 질량 | 4.949052425 kg | 4.949052425 kg |

- `q`는 구성 벡터, `v`는 일반화 속도입니다. 연속 회전 관절은 각도를 cos/sin 두 값으로, 부유 베이스 회전은 쿼터니언으로 표현하므로 `nq`와 `nv`가 다릅니다. `q`를 모두 0으로 초기화하지 마세요.
- 관절 순서는 URDF의 작성 순서와 다를 수 있습니다. 스크립트가 출력하는 이름 및 `q`, `v` 인덱스를 사용하세요. 슬라이스의 마지막 인덱스는 포함하지 않습니다.
- `M`은 `(nv, nv)`, `h`와 `g`는 `(nv,)` 배열입니다. 접촉 없는 이상적 강체 모델의 일반화 힘은 `M @ a + h`이며, `a`는 일반화 가속도입니다.
- `h`에는 코리올리·원심력 및 중력 효과가 포함됩니다. 현재 모델에서 `v = 0`이면 `h = g`입니다. 실제 마찰이나 모터 오차는 포함하지 않습니다.
- 부유 베이스의 `g[:6]`는 베이스 좌표계의 힘 3성분(N)과 모멘트 3성분(N·m)입니다. 나머지는 관절 일반화 토크이며, 수동 관절까지 포함하므로 전체 벡터를 모터 명령으로 사용할 수 없습니다.
- `com`은 월드 좌표계에서 표현한 **몸체를 포함한 전체 로봇**의 무게중심입니다. 영점 자세에서는 약 `[-0.045053, -0.000651, -0.044132] m`입니다.
- 고정 베이스에서 Pinocchio의 기본 무게중심 계산은 고정된 루트의 질량을 제외합니다. 이 코드에서는 해당 질량을 합산해 전체 로봇의 질량과 무게중심을 보고합니다. `moving_mass`는 고정 베이스에서 약 2.164593452 kg입니다.

저장된 `.npz`에는 `q`, `v`, `M`, `h`, `g`, `com`, 질량, 관절 이름·인덱스, 베이스 방식, URDF 경로 및 Pinocchio 버전이 들어갑니다.

```python
import numpy as np

result = np.load("outputs/neutral_floating.npz")
print(result["M"])
print(result["com"])
```

## 이후 코드에서 재사용

모델과 계산용 데이터는 한 번 만들고, 상태가 바뀔 때 계산 함수를 호출할 수 있습니다.

```python
import numpy as np
import pinocchio as pin
from twbr.modeling import load_model, compute_dynamics

model = load_model("2WL_V3_Assem_v2/urdf/2WL_V3_Assem_v2.urdf", base="floating")
data = model.createData()
q = pin.neutral(model)
v = np.zeros(model.nv)
result = compute_dynamics(model, data, q, v)
print(result.mass_matrix)
```

자세를 변경할 때는 연속 회전 관절·쿼터니언의 표현을 유지하는 `pin.integrate` 등을 사용합니다.
`data`는 계산 중 갱신되므로 동시에 실행하는 계산 사이에서 공유하지 않습니다.
현재 코드는 수치 모델 확인용으로, 실시간 제어 성능을 최적화한 구현은 아닙니다.

## 참고

- [Pinocchio 공식 설치 안내](https://stack-of-tasks.github.io/pinocchio/download.html): PyPI 패키지명은 `pin`, Python import 이름은 `pinocchio`입니다.
- [Pinocchio 시각화 예제](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/README.md)
- [현재 모델의 가정과 검증 범위](modeling_assumptions.md)

