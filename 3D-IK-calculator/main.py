from vpython import *
import math

# DOF 선택
while True:
    dof = int(input("로봇팔 자유도 선택 (2 또는 3): "))
    if dof in [2, 3]:
        break

# 링크 길이 입력
l1 = float(input("첫 번째 링크 길이(cm): "))
l2 = float(input("두 번째 링크 길이(cm): "))
l3 = float(input("세 번째 링크 길이(cm): ")) if dof == 3 else 0

# VPython 설정
scene.title = f"{dof}DOF 로봇팔 IK 시뮬레이터"
scene.width = 800
scene.height = 600
scene.background = color.white
scene.camera.pos = vector(15, 15, 15)
scene.camera.axis = vector(-15, -15, -15)
scene.up = vector(0, 0, 1)

# 바닥 격자
for i in range(-10, 11):
    curve(pos=[vector(i, -10, 0), vector(i, 10, 0)], color=color.gray(0.7))
    curve(pos=[vector(-10, i, 0), vector(10, i, 0)], color=color.gray(0.7))

# 회전판, 중심
base = vector(0, 0, 0)
rotation_axis = arrow(pos=base, axis=vector(2, 0, 0), shaftwidth=0.1, color=color.green)
center_dot = sphere(pos=base, radius=0.05, color=color.black)

# 링크들
arm1 = cylinder(radius=0.15, color=color.blue)
arm2 = cylinder(radius=0.12, color=color.orange)
arm3 = cylinder(radius=0.10, color=color.purple) if dof == 3 else None

# 공
ball = sphere(pos=vector(5, 5, 5), radius=0.3, color=color.red, make_trail=False)

# 라벨
coord_label = label(pos=vector(-8, 12, 10), height=14, border=4, font='sans', box=False, opacity=1, color=color.black)
angle_label = label(pos=vector(-8, 10.5, 10), height=14, border=4, font='sans', box=False, opacity=1, color=color.black)

# 이전 각도 초기값
prev_angles = [0, 0, 0, 0]

# 2DOF 역기구학 함수

def inverse_kinematics_2dof(x, y, l1, l2):
    dist = math.sqrt(x ** 2 + y ** 2)
    if dist > l1 + l2:
        return None
    theta2 = -math.acos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2))
    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(theta1), math.degrees(theta2)

# 3DOF 완전한 3D IK 계산 함수

def inverse_kinematics_3dof_xyz(x, y, z, l1, l2, l3, prev_angles):
    yaw = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    dx = r
    dz = z

    dist = math.sqrt(dx**2 + dz**2)
    if dist > l1 + l2 + l3:
        return None

    best_error = float('inf')
    best_cost = float('inf')
    best_solution = None
    for phi_deg in range(-180, 181, 2):
        phi = math.radians(phi_deg)
        x_p = dx - l3 * math.cos(phi)
        z_p = dz - l3 * math.sin(phi)

        d2 = math.sqrt(x_p**2 + z_p**2)
        if d2 > l1 + l2 or d2 < abs(l1 - l2):
            continue

        theta2 = -math.acos((x_p**2 + z_p**2 - l1**2 - l2**2) / (2 * l1 * l2))
        k1 = l1 + l2 * math.cos(theta2)
        k2 = l2 * math.sin(theta2)
        theta1 = math.atan2(z_p, x_p) - math.atan2(k2, k1)
        theta3 = phi - (theta1 + theta2)

        # 검증 (끝점 도달 정확도 측정)
        end_x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2) + l3 * math.cos(theta1 + theta2 + theta3)
        end_z = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2) + l3 * math.sin(theta1 + theta2 + theta3)
        error = math.hypot(end_x - dx, end_z - dz)

        # 이전 각도들과 차이 최소화
        cost = error + 0.1 * (abs(math.degrees(theta1) - prev_angles[1]) + abs(math.degrees(theta2) - prev_angles[2]) + abs(math.degrees(theta3) - prev_angles[3]))

        if cost < best_cost:
            best_error = error
            best_cost = cost
            best_solution = (math.degrees(yaw), math.degrees(theta1), math.degrees(theta2), math.degrees(theta3))

    return best_solution

# 업데이트 함수

def update_arm():
    global prev_angles
    world_target = ball.pos - base
    if dof == 2:
        planar_dist = math.sqrt(world_target.x ** 2 + world_target.y ** 2)
        ik = inverse_kinematics_2dof(planar_dist, world_target.z, l1, l2)
        if ik is None:
            coord_label.text = "범위 초과"
            angle_label.text = ""
            return
        t1, t2 = ik
        yaw = math.degrees(math.atan2(world_target.y, world_target.x))
    else:
        ik = inverse_kinematics_3dof_xyz(world_target.x, world_target.y, world_target.z, l1, l2, l3, prev_angles)
        if ik is None:
            coord_label.text = "범위 초과"
            angle_label.text = ""
            return
        yaw, t1, t2, t3 = ik
        prev_angles = [yaw, t1, t2, t3]

    dir1 = vector(math.cos(math.radians(t1)), 0, math.sin(math.radians(t1)))
    dir1_world = vector(dir1.x * math.cos(math.radians(yaw)), dir1.x * math.sin(math.radians(yaw)), dir1.z)
    joint1 = base + l1 * dir1_world
    arm1.pos = base
    arm1.axis = joint1 - base

    dir2 = vector(math.cos(math.radians(t1 + t2)), 0, math.sin(math.radians(t1 + t2)))
    dir2_world = vector(dir2.x * math.cos(math.radians(yaw)), dir2.x * math.sin(math.radians(yaw)), dir2.z)
    joint2 = joint1 + l2 * dir2_world
    arm2.pos = joint1
    arm2.axis = joint2 - joint1

    if dof == 3:
        dir3 = vector(math.cos(math.radians(t1 + t2 + t3)), 0, math.sin(math.radians(t1 + t2 + t3)))
        dir3_world = vector(dir3.x * math.cos(math.radians(yaw)), dir3.x * math.sin(math.radians(yaw)), dir3.z)
        joint3 = joint2 + l3 * dir3_world
        arm3.pos = joint2
        arm3.axis = joint3 - joint2
        angle_label.text = f"θ₀(yaw): {yaw:.1f}°\nθ₁: {t1:.1f}°\nθ₂: {t2:.1f}°\nθ₃: {t3:.1f}°"
    else:
        angle_label.text = f"θ₀(yaw): {yaw:.1f}°\nθ₁: {t1:.1f}°\nθ₂: {t2:.1f}°"

    coord_label.text = f"Position: x={ball.pos.x:.1f}, y={ball.pos.y:.1f}, z={ball.pos.z:.1f}"
    rotation_axis.axis = vector(math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0) * 2


# 키 입력 처리
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
update_arm()

# 실행 루프
while running:
    rate(60)
