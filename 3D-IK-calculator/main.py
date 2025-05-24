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
scene.camera.pos = vector(15, 15, 0)
scene.camera.axis = vector(-15, -15, 0)
scene.up = vector(0, 0, 1)

# 바닥 격자 (xy 평면 기준으로 시각화)
for i in range(-10, 11):
    curve(pos=[vector(i, -10, 0), vector(i, 10, 0)], color=color.gray(0.7))
    curve(pos=[vector(-10, i, 0), vector(10, i, 0)], color=color.gray(0.7))

# 회전판 시각화 (벡터로 구현)
base = vector(0, 0, 0)
rotation_axis = arrow(pos=base, axis=vector(2, 0, 0), shaftwidth=0.1, color=color.green)
center_dot = sphere(pos=base, radius=0.05, color=color.black)

# 링크들 (arm1, arm2)
arm1 = cylinder(radius=0.15, color=color.blue)
arm2 = cylinder(radius=0.12, color=color.orange)

# 공 (이동할 대상)
ball = sphere(pos=vector(5, 5, 0), radius=0.3, color=color.red, make_trail=False)

# 라벨 (고정된 좌표에 분리해서 표시)
coord_label = label(pos=vector(-8, 12, 10), text='', xoffset=0, yoffset=0, space=30,
                    height=14, border=4, font='sans', box=False, opacity=1, color=color.black)
angle_label = label(pos=vector(-8, 10.5, 10), text='', xoffset=0, yoffset=0, space=30,
                    height=14, border=4, font='sans', box=False, opacity=1, color=color.black)

# IK 계산 함수 (ㄱ자 형태 기본자세)
def inverse_kinematics_2d(x, y, l1, l2):
    dist = math.sqrt(x ** 2 + y ** 2)
    if dist > l1 + l2 or dist < abs(l1 - l2):
        return None
    cos_theta2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    theta2 = -math.acos(cos_theta2)  # elbow down (ㄱ 형태)
    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(theta1), math.degrees(theta2)

# 공 위치에 따라 링크 갱신
def update_arm():
    global yaw_angle
    world_target = ball.pos - base

    # 회전판 yaw 계산 (삼각측량)
    yaw_angle = math.degrees(math.atan2(world_target.y, world_target.x))
    yaw_rad = math.radians(-yaw_angle)

    planar_dist = math.sqrt(world_target.x**2 + world_target.y**2)  # 피타고라스
    x_local = planar_dist
    z_local = world_target.z

    ik_result = inverse_kinematics_2d(x_local, z_local, l1, l2)

    if ik_result is None:
        coord_label.text = "범위 초과 → 원점 복귀"
        angle_label.text = ""
        arm1.visible = False
        arm2.visible = False
        return

    arm1.visible = True
    arm2.visible = True
    theta1, theta2 = ik_result

    dir1_local = vector(math.cos(math.radians(theta1)), 0, math.sin(math.radians(theta1)))
    dir1_world = vector(
        dir1_local.x * math.cos(math.radians(yaw_angle)),
        dir1_local.x * math.sin(math.radians(yaw_angle)),
        dir1_local.z
    )
    joint1 = base + l1 * dir1_world

    arm1.pos = base
    arm1.axis = joint1 - base

    dir2_local = vector(math.cos(math.radians(theta1 + theta2)), 0, math.sin(math.radians(theta1 + theta2)))
    dir2_world = vector(
        dir2_local.x * math.cos(math.radians(yaw_angle)),
        dir2_local.x * math.sin(math.radians(yaw_angle)),
        dir2_local.z
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
    rotation_axis.axis = vector(math.cos(math.radians(yaw_angle)), math.sin(math.radians(yaw_angle)), 0) * 2

# 초기화
yaw_angle = 0.0
update_arm()

# 키보드 제어
step = 0.3
running = True

def key_input(evt):
    global running
    s = evt.key
    if s == 'e': ball.pos.z += step
    elif s == 'q': ball.pos.z -= step
    elif s == 'a': ball.pos.x -= step
    elif s == 'd': ball.pos.x += step
    elif s == 'w': ball.pos.y += step
    elif s == 's': ball.pos.y -= step
    elif s == 'esc' or s == 'x':
        running = False
        scene.delete()
        return
    update_arm()

scene.bind('keydown', key_input)

while running:
    rate(60)