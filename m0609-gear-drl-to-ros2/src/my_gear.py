# 티치 펜던트(Task Writer)로 만든 "m0609_homework_gear_1" 을 ROS 2 Python(DSR_ROBOT2)으로 옮긴 노드
# 원본: ../drl/m0609_homework_gear_1.drl + ../drl/System.drvar
# 노드 실행 뼈대(DR_init 설정, DSR_ROBOT2 import, 예외 처리)는 수업 제공 템플릿 패턴을 따름
#
# 동작 흐름
#   홈 → (기어 n 집기 → 놓기) × 3 (n = 1~3) → 기어4 집기 → 기어4 위치로 이동 후 30mm 하강
#   → 순응제어 + 힘제어(Z -5N, ABS)로 내려가며, 힘이 3~8N이면 Z축 회전(±10도) 흔들기로 맞물리기
#   → Z가 80mm 이하로 내려가면 그리퍼 열기 → 힘제어·순응제어 해제 → 상승 후 홈 복귀
#
# 2026-09-16 18:36 실로봇 실행에서 끝까지 동작한 버전 (실행 로그: ../logs/run_20260916_1836_full_success.log)
# 목표 힘 -5N / 판정 3~8N / atime 0.3, repeat 2 는 디버깅 중 낮춘 값이며,
# 원본 DRL 값(-10N / 7~12N / atime 1.0, repeat 3)으로 되돌린 재검증은 하지 않음

import rclpy
import DR_init
import time


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 원본 DRL 속도값. 처음 실행할 때는 안전을 위해 낮춰서(예: 30, 30) 테스트 권장
J_VEL, J_ACC = 150.0, 150.0          # movej (System_gj*, System_gtj*)
L_VEL, L_ACC = [1000.0, 142.5], [1500.0, 570.0]   # grasp/release 안의 movel
L_VEL2, L_ACC2 = [1000.0, 142.5], [1000.0, 570.0] # 메인 시퀀스의 movel

ON, OFF = 1, 0

# 티치펜던트(워크셀 매니저)에 등록된 이름과 글자 하나까지 같아야 함
TOOL_NAME = "Tool Weight"
TCP_NAME = "GripperDA_v1"


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_gear_pick_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
            set_singular_handling, set_velj, set_accj, set_velx, set_accx,
            movej, movel, amovel, wait, move_periodic,
            set_digital_output,
            task_compliance_ctrl, set_stiffnessx, release_compliance_ctrl,
            set_desired_force, release_force,
            check_force_condition, check_position_condition,
            get_tool_force, get_current_posx,mwait,
            DR_AVOID, DR_MV_MOD_REL, DR_MV_MOD_ABS, DR_FC_MOD_REL,
            DR_AXIS_Z, DR_BASE, DR_TOOL,DR_FC_MOD_ABS,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 컨트롤러 서비스가 보일 때까지 대기 (연결 전에 보낸 첫 명령이 유실되는 것을 방지)
    from dsr_msgs2.srv import SetSingularityHandling
    probe = node.create_client(SetSingularityHandling, "dsr_controller2/motion/set_singularity_handling")
    while not probe.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("컨트롤러 서비스 대기 중... (브링업이 떠 있는지 확인)")
    time.sleep(1.0)
    node.get_logger().info("컨트롤러 연결 확인")

    # ---------- System.drvar 에서 가져온 티칭 좌표 ----------
    home = posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00)

    gj1 = posj(-24.31, 1.93, 103.61, -0.71, 74.37, -24.66)   # 기어1 집는 위치
    gtj1 = posj(-13.37, 44.40, 42.75, -0.86, 92.88, -14.03)  # 기어1 놓는 위치
    gj2 = posj(-22.51, -13.56, 117.30, -1.21, 76.11, -24.76) # 기어2 집는 위치
    gtj2 = posj(-10.48, 27.25, 71.08, -1.36, 81.78, -12.83)  # 기어2 놓는 위치
    gj3 = posj(-39.76, -4.86, 110.21, -1.21, 74.20, -41.89)  # 기어3 집는 위치
    gtj3 = posj(-20.56, 32.31, 63.21, -1.41, 84.44, -22.91)  # 기어3 놓는 위치
    gj4 = posj(-28.71, -6.48, 111.65, -1.41, 74.03, -17.13)  # 기어4 집는 위치
    gtj4 = posj(-14.56, 33.24, 61.71, -1.68, 84.60, -3.25)   # 기어4 놓는 위치(맞물림)

    # ---------- 서브프로그램 (DRL 의 grasp / release) ----------
    def grasp():
        node.get_logger().info("grasp")
        movel(posx(0, 0, -50, 0, 0, 0), vel=L_VEL, acc=L_ACC, radius=0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2)
        movel(posx(0, 0, 50, 0, 0, 0), vel=L_VEL, acc=L_ACC, radius=0, ref=DR_BASE, mod=DR_MV_MOD_REL)

    def release():
        node.get_logger().info("release")
        movel(posx(0, 0, -50, 0, 0, 0), vel=L_VEL, acc=L_ACC, radius=0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(2)
        movel(posx(0, 0, 50, 0, 0, 0), vel=L_VEL, acc=L_ACC, radius=0, ref=DR_BASE, mod=DR_MV_MOD_REL)

    # ---------- 툴/TCP 설정 ----------
    # 브링업을 다시 켠 뒤 펜던트의 툴/TCP가 적용되지 않은 채 실행된 적이 있어 스크립트에서 매번 설정.
    # set_tool/set_tcp 는 수동 모드에서만 받으므로 잠깐 수동으로 바꿨다가 자동으로 되돌림.
    set_robot_mode(ROBOT_MODE_MANUAL)
    rt = set_tool(TOOL_NAME)
    rc = set_tcp(TCP_NAME)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    node.get_logger().info(f"set_tool={rt}, set_tcp={rc}  (0=성공, -1=실패: 펜던트 이름 확인)")
    if rt != 0 or rc != 0:
        node.get_logger().error("툴/TCP 설정 실패. 힘제어가 잘못 동작하므로 중단합니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # ---------- 초기 설정 ----------
    set_singular_handling(DR_AVOID)
    set_velj(60.0)
    set_accj(100.0)
    set_velx(500.0, 101.25)
    set_accx(1000.0, 405.0)

    try:
        # ---------- 메인 시퀀스 (DRL 의 while 루프 1회) ----------
        movej(home, vel=150, acc=100, radius=0)

        # 기어 1~3: 집어서 옮기기
        for i, (pick, place) in enumerate([(gj1, gtj1), (gj2, gtj2), (gj3, gtj3)], start=1):
            node.get_logger().info(f"=== 기어 {i} 이동 ===")
            movej(pick, vel=J_VEL, acc=J_ACC, radius=0)
            grasp()
            movej(place, vel=J_VEL, acc=J_ACC, radius=0)
            release()

        # 기어 4: 집어서 힘제어로 맞물리기
        node.get_logger().info("=== 기어 4 이동 (힘제어 삽입) ===")
        movej(gj4, vel=J_VEL, acc=J_ACC, radius=0)
        grasp()
        movej(gtj4, vel=J_VEL, acc=J_ACC, radius=0)
        movel(posx(0, 0, -30, 0, 0, 0), vel=L_VEL2, acc=L_ACC2, radius=0, ref=DR_BASE, mod=DR_MV_MOD_REL)

        # 순응제어 + 힘제어 시작 (Z축으로 -5N, 절대 기준 DR_FC_MOD_ABS. 원본 DRL은 -10N, REL)
        node.get_logger().info(f"접촉 전 힘: {get_tool_force()}")
        node.get_logger().info(f"현재 위치: {get_current_posx()[0]}")

        r1 = task_compliance_ctrl()
        r2 = set_stiffnessx([3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0], time=0.0)
        r3 = set_desired_force([0.0, 0.0, -5.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
        node.get_logger().info(f"compliance={r1}, stiffness={r2}, desired_force={r3}  (0=성공, -1=실패)")

        # amovel 삭제: 힘제어가 Z를 잡고 있으면 위치 이동 명령은 무시되면서
        #   DRCF 상태만 TASK_MOTION_w_FORCE_CONTROL 로 남아 move_periodic 이 거부됨.
        #   힘제어(-5N)만으로 닿을 때까지 내려간다.
        wait(0.5)

        def force_in_range(fz):
            return 3.0 <= abs(fz) <= 8.0

        def inserted():
            return check_position_condition(DR_AXIS_Z, max=80, ref=DR_BASE, mod=DR_MV_MOD_ABS, pos=None) == 0

        t0 = time.time()
        last_periodic = 0.0
        while time.time() - t0 < 15.0:
            fz = get_tool_force()[2]
            z = get_current_posx()[0][2]
            node.get_logger().info(f"Fz={fz:.1f}N  Z={z:.1f}mm")
            if inserted():
                node.get_logger().info("삽입 완료 → 그리퍼 열기")
                break
            if force_in_range(fz) and time.time() - last_periodic > 2.5:
                # 규칙: atime < (period × repeat) / 2   → 여기선 0.3 < 1.0
                node.get_logger().info("힘 조건 만족 → 흔들기 시작")
                move_periodic(amp=[0, 0, 0, 0, 0, 10], period=[0, 0, 0, 0, 0, 1], atime=0.3, repeat=2, ref=DR_TOOL)
                last_periodic = time.time()
                continue
            wait(0.1)
        else:
            node.get_logger().info("15초 시간 초과")

        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(2)

        # 힘제어 → 순응제어 순서로 해제해야 이후 위치 이동 명령이 정상 동작함
        release_force(time=0.0)
        release_compliance_ctrl()
        node.get_logger().info("힘제어/순응제어 해제 → 홈 복귀")

        movel(posx(0, 0, 50, 0, 0, 0), vel=L_VEL2, acc=L_ACC2, radius=0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movej(home, vel=90, acc=100, radius=0)
        node.get_logger().info("작업 완료")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except Exception as e:
        node.get_logger().error(f"Robot Error: {e}")
    finally:
        # 힘제어 중 중단됐을 때를 대비해 항상 해제
        try:
            release_force(time=0.0)
            release_compliance_ctrl()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
