# 3D-IK-Calculator

![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)  

🚀 **3D-IK-Calculator**는 VPython으로 구현된 인터랙티브 3D 역기구학(Inverse Kinematics) 시뮬레이터입니다.  
사용자는 키보드로 타겟 위치(빨간 공)를 이동시키고, 2-링크 로봇팔이 실시간으로 역기구학 계산을 통해 타겟을 추적합니다.

---

## 🎮 Features

- ✅ **3자유도 로봇팔 시뮬레이션** (Yaw + Shoulder + Elbow)
- ✅ **실시간 역기구학 계산** 및 시각화
- ✅ **키보드 기반 제어 (Q, E, W, A, S, D)**
- ✅ **조인트별 각도 표시 (θ₀, θ₁, θ₂)**
- ✅ **화면 고정 라벨 UI**
- ✅ **범위 초과 시 자동 복귀 처리**

---

## 🔧 Controls

| 키     | 동작                   |
|--------|------------------------|
| W / S  | 앞 / 뒤 이동 (z축)     |
| A / D  | 좌 / 우 이동 (x축)     |
| Q / E  | 아래 / 위 이동 (y축)   |
| X / ESC| 프로그램 종료          |

---

## 🛠 Requirements

- Python 3.7+
- VPython

설치:
```
pip install vpython
````

---

## ▶️ How to Run

```
cd 3D-IK-calculator
venv\Scripts\activate
python main.py

```

실행 시 링크 길이(l1, l2)를 입력한 뒤, 시뮬레이션 창이 열립니다.
빨간 공을 키보드로 이동시키면 로봇팔이 추적하며, 좌측 상단에 현재 좌표 및 조인트 각도가 표시됩니다.

---

## 📁 Files

| 파일명         | 설명               |
| ----------- | ---------------- |
| `main.py`   | 메인 실행 파일 (시뮬레이터) |
| `README.md` | 설명 파일            |


---

## 🖼️ Photo

<img width="503" alt="Image" src="https://github.com/user-attachments/assets/57882305-5456-4c66-bf46-7c4016b14913" />