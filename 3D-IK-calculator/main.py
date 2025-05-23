from vpython import *
import math

# 링크 길이 입력
l1 = float(input("첫 번째 링크 길이(cm): "))
l2 = float(input("두 번째 링크 길이(cm): "))

# VPython 설정
scene.title = "3DOF 로봇팔 IK 시뮬레이터"
scene.width = 800
scene.height = 600
scene.background = color.white
scene.camera.pos = vector(15, 15, 15)
scene.camera.axis = vector(-15, -15, -15)

# 바닥 격자 (xy 평면 기준으로 시각화)
for i in range(-10, 11):
    curve(pos=[vector(i, 0, -10), vector(i, 0, 10)], color=color.gray(0.7))
    curve(pos=[vector(-10, 0, i), vector(10, 0, i)], color=color.gray(0.7))

# 회전판 시각화 (벡터로 구현)
base = vector(0, 0, 0)
rotation_axis = arrow(pos=base, axis=vector(2, 0, 0), shaftwidth=0.1, color=color.green)
center_dot = sphere(pos=base, radius=0.05, color=color.black)

# 링크들 (arm1, arm2)
arm1 = cylinder(radius=0.15, color=color.blue)
arm2 = cylinder(radius=0.12, color=color.orange)

# 공 (이동할 대상)
ball = sphere(pos=vector(5, 5, 0), radius=0.3, color=color.red, make_trail=False)

# 라벨 (카메라 좌상단 고정 위치)
coord_label = label(pos=vector(-8, 12, 0), text='', xoffset=0, yoffset=0, space=30,
                    height=14, border=4, font='sans', box=False, opacity=0)
angle_label = label(pos=vector(-8, 10.5, 0), text='', xoffset=0, yoffset=0, space=30,
                    height=14, border=4, font='sans', box=False, opacity=0)

# IK 계산 함수
def inverse_kinematics_2d(x, y, l1, l2):
    dist = math.sqrt(x ** 2 + y ** 2)
    if dist > l1 + l2 or dist < abs(l1 - l2):
        return None
    cos_theta2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    theta2 = math.acos(cos_theta2)
    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(theta1), math.degrees(theta2)

# 회전판 각도 (yaw)
yaw_angle = 0.0  # 도 단위

# 공 위치에 따라 링크 갱신
def update_arm():
    global yaw_angle
    # 월드 좌표계 기준 공 위치
    world_target = ball.pos - base

    # 회전판 좌표계 기준으로 변환 (yaw 반대방향으로 회전)
    yaw_rad = math.radians(-yaw_angle)
    x_local = world_target.x * math.cos(yaw_rad) + world_target.z * math.sin(yaw_rad)
    y_local = world_target.y
    z_local = -world_target.x * math.sin(yaw_rad) + world_target.z * math.cos(yaw_rad)

    ik_result = inverse_kinematics_2d(x_local, y_local, l1, l2)

    if ik_result is None:
        coord_label.text = "범위 초과 → 원점 복귀"
        angle_label.text = ""
        arm1.visible = False
        arm2.visible = False
        return

    arm1.visible = True
    arm2.visible = True
    theta1, theta2 = ik_result

    # 회전판 기준 벡터 → 월드 기준 회전 적용
    dir1_local = vector(math.cos(math.radians(theta1)), math.sin(math.radians(theta1)), 0)
    dir1_world = vector(
        dir1_local.x * math.cos(math.radians(yaw_angle)) - dir1_local.z * math.sin(math.radians(yaw_angle)),
        dir1_local.y,
        dir1_local.x * math.sin(math.radians(yaw_angle)) + dir1_local.z * math.cos(math.radians(yaw_angle))
    )
    joint1 = base + l1 * dir1_world

    arm1.pos = base
    arm1.axis = joint1 - base

    dir2_local = vector(math.cos(math.radians(theta1 + theta2)), math.sin(math.radians(theta1 + theta2)), 0)
    dir2_world = vector(
        dir2_local.x * math.cos(math.radians(yaw_angle)) - dir2_local.z * math.sin(math.radians(yaw_angle)),
        dir2_local.y,
        dir2_local.x * math.sin(math.radians(yaw_angle)) + dir2_local.z * math.cos(math.radians(yaw_angle))
    )
    joint2 = joint1 + l2 * dir2_world

    arm2.pos = joint1
    arm2.axis = joint2 - joint1

    coord_label.text = (
        f"Position: x={ball.pos.x:.1f}, y={ball.pos.y:.1f}, z={ball.pos.z:.1f}"
    )
    angle_label.text = (
        f"θ₀ (yaw): {yaw_angle:.1f}°\n"
        f"θ₁ (shoulder): {theta1:.1f}°\n"
        f"θ₂ (elbow): {theta2:.1f}°"
    )
    rotation_axis.axis = vector(math.cos(math.radians(yaw_angle)), 0, math.sin(math.radians(yaw_angle))) * 2

# 초기화
update_arm()

# 키보드 제어
step = 0.3
running = True

# 키 입력 처리
def key_input(evt):
    global running
    s = evt.key
    if s == 'e': ball.pos.y += step
    elif s == 'q': ball.pos.y -= step
    elif s == 'a': ball.pos.x -= step
    elif s == 'd': ball.pos.x += step
    elif s == 'w': ball.pos.z -= step
    elif s == 's': ball.pos.z += step
    elif s == 'esc' or s == 'x':
        running = False
        scene.delete()
        return
    update_arm()

scene.bind('keydown', key_input)

while running:
    rate(60)
