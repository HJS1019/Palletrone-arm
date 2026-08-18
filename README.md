This first version is developed by <https://github.com/Ryung-coding>

## Logging And MATLAB Workflow

### 1. Build

```bash
source /opt/ros/humble/setup.bash
cd /home/parkjeongsu/ros2_project/Palletrone_sim
colcon build --executor sequential
source install/setup.bash
```

### 2. Record All ROS 2 Topics

Run the simulation, then record every topic:

```bash
source /opt/ros/humble/setup.bash
source /home/parkjeongsu/ros2_project/Palletrone_sim/install/setup.bash

ros2 bag record -a -o bags/bag_all_01 \
  --compression-mode file \
  --compression-format zstd
```

If you use `ros2 launch palletrone_cmd arm_launch.py`, bag recording starts automatically and writes to:

```text
/home/parkjeongsu/ros2_project/Palletrone_sim/bags/bag_all_YYYYmmdd_HHMMSS
```

This records all topics including:
- `/palletrone_state`
- `/cmd`
- `/att_cmd`
- `/wrench`
- `/input`
- `/actuator_debug/cmd_servo`
- `/actuator_debug/real_servo`
- `/actuator_debug/cmd_bldc`
- `/actuator_debug/real_bldc`
- `/external_wrench_cmd`
- `/external_wrench_hat`

### 3. Convert Bag To CSV

Use the helper script:

```bash
source /opt/ros/humble/setup.bash
source /home/parkjeongsu/ros2_project/Palletrone_sim/install/setup.bash

python3 scripts/bag_to_csv.py bags/bag_all_01 --out-dir csv_out
```

This generates per-topic CSV files such as:
- `csv_out/palletrone_state.csv`
- `csv_out/cmd.csv`
- `csv_out/att_cmd.csv`
- `csv_out/wrench.csv`
- `csv_out/input.csv`
- `csv_out/actuator_debug__real_servo.csv`

### 4. Plot In MATLAB

From the workspace root:

```matlab
run("matlab/plot_tracking.m")
```

The MATLAB script creates these figures when the corresponding CSV files exist:
- Position tracking: `/cmd` desired vs `/palletrone_state` real
- Attitude tracking: `/att_cmd` desired vs `/palletrone_state` real
- Servo tracking: cmd vs real servo angle
- BLDC cmd vs real thrust
- Wrench command history
- External wrench estimate: command ground truth vs `/external_wrench_hat`
- Allocator input command history

### 5. Notes

- `AttitudeCmd` is logged in degrees.
- `/palletrone_state.rpy` is logged in radians, and converted to degrees in MATLAB.
- `wrench` and `input` currently do not have a direct measured feedback topic pair, so they are plotted as command histories rather than desired-vs-real overlays.
- `external_wrench_cmd` publishes a body-frame ground-truth disturbance. Key map: `q/a` Mx, `w/s` My, `e/d` Mz, `i/k` Fx, `j/l` Fy, `u/o` Fz, `z` reset.

## Code Structure

```text
Sim_palletrone/
└── src/
	├── palletrone_cmd
	│   ├── CMakeLists.txt
	│   ├── launch
	│   │   ├── arm_launch.py
	│   │   ├── pt_launch.py
	│   │   └── __pycache__
	│   │       └── arm_launch.cpython-310.pyc
	│   ├── package.xml
	│   └── src
	│       ├── attitude_sweep_cmd.cpp
	│       └── position_cmd.cpp
	├── palletrone_controller
	│   ├── CMakeLists.txt
	│   ├── package.xml
	│   └── src
	│       ├── allocator_controller.cpp
	│       └── wrench_controller.cpp
	├── palletrone_interfaces
	│   ├── CMakeLists.txt
	│   ├── msg
	│   │   ├── AttitudeCmd.msg
	│   │   ├── Cmd.msg
	│   │   ├── Input.msg
	│   │   ├── PalletroneState.msg
	│   │   └── Wrench.msg
	│   └── package.xml
	└── plant
	    ├── package.xml
	    ├── plant
	    │   ├── __init__.py
	    │   ├── palm_teleop.py
	    │   ├── plant.py
	    │   └── __pycache__
	    │       ├── __init__.cpython-310.pyc
	    │       ├── palm_teleop.cpython-310.pyc
	    │       └── plant.cpython-310.pyc
	    ├── resource
	    │   └── plant
	    ├── setup.cfg
	    ├── setup.py
	    ├── test
	    │   ├── test_copyright.py
	    │   ├── test_flake8.py
	    │   └── test_pep257.py
	    └── xml
		├── BODY.stl
		├── Palletrone.xml
		├── palm.xml
		├── PROP.stl
		├── scene.xml
		└── STLchanger.py

---

## Arm-Mounted Simulation

### Requirements

```bash
pip install mujoco numpy matplotlib
```

### 1. Build The Model

```bash
python3 build_arm_drone.py \
  --drone src/plant/xml/Palletrone.xml \
  --arm ~/cone_harvester_sim_ws/src/cone_harvester_sim/models/Forest_Cone_Harvesting_Robot.xml \
  --rigid --j1 90 --a -0.16 \
  --base-mass 3.3 --arm-len 0.23 \
  --cw-mass 0.610 --cw-pos -0.251 --cw-z 0.165 \
  --top-height 0.404 \
  --out src/plant/xml/model_a16.xml
```

| 옵션 | 의미 | 기본값 |
| --- | --- | --- |
| `--drone` | Palletrone.xml 경로 | (필수) |
| `--arm` | 로봇팔 XML 경로 | (필수) |
| `--a` | 팔 장착점 x 오프셋 [m] | -0.014 |
| `--j1` ~ `--j4` | 팔 관절 각도 [deg] | 90 / 0 / 0 / 0 |
| `--rigid` | 관절을 지정 자세로 굳혀 강체화 | off |
| `--mount-z` | 팔 장착 높이 [m] | -0.125 |
| `--base-mass` | 드론 본체 질량 [kg] | 3.3 |
| `--arm-len` | 원점→프로펠러 암 길이 [m] | 0.23 |
| `--cw-mass` | 변압기 질량 [kg] | 0.610 |
| `--cw-pos` | 변압기 x 위치 [m] | -0.251 |
| `--cw-z` | 변압기 z 위치 [m] | 0.165 |
| `--top-height` | 지면~드론 윗면 [m] (다리 길이 자동 계산) | 0.404 |
| `--gear-span` | 다리 배치 반경 [m] | 0.25 |
| `--kp` | 팔 관절 위치 게인 (`--rigid` 아닐 때) | 80 |
| `--out` | 출력 XML 경로 | Palletrone_with_arm.xml |

모델 확인:

```bash
cd src/plant/xml && python3 -m mujoco.viewer --mjcf=model_a16.xml && cd -
```

### 2. Interactive Flight

```bash
python3 fly_sim.py --model src/plant/xml/model_a16.xml
```

| 옵션 | 의미 | 기본값 |
| --- | --- | --- |
| `--model` | 미리 만들어둔 XML 경로 | — |
| `--xml` | Palletrone.xml (모델 즉석 생성 시) | — |
| `--alt` | 호버 고도 [m] | 1.0 |
| `--ramp` | 이착륙 램프 시간 [s] | 4.0 |
| `--kf` | DoB 힘 게인 | 5.0 |
| `--km` | DoB 모멘트 게인 | 10.0 |
| `--no-viewer` | 창 없이 계산만 | off |
| `--script` | `"시각:명령;..."` 자동 실행 | — |

터미널 명령어:

| 명령 | 동작 |
| --- | --- |
| `takeoff` / `t` | 이륙 → 호버 유지 |
| `dob on` / `dob off` | 외란 관측기 켜기 / 끄기 |
| `dist X Y Z` | 외란 힘 인가 [N], world frame |
| `dist off` | 외란 해제 |
| `moment X Y Z` | 외란 모멘트 인가 [Nm], body frame |
| `goto X Y Z` | 목표 위치 이동 [m] |
| `status` / `s` | 현재 상태 출력 |
| `log <파일>` | CSV 기록 시작 |
| `land` / `l` | 착륙 → 종료 |
| `quit` / `q` | 즉시 종료 |
| `help` / `h` | 명령어 목록 |

뷰어 단축키:

| 키 | 동작 |
| --- | --- |
| `T` | 이륙 |
| `D` | DoB 토글 |
| `X` | 5 N 외란 토글 |
| `L` | 착륙 |
| `Q` | 종료 |
