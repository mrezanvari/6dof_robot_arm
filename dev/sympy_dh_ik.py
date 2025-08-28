from sympy import *


def frame(theta, alpha, radius, displacement) -> MutableDenseMatrix:
    ca = cos(alpha)
    sa = sin(alpha)
    ct = cos(theta)
    st = sin(theta)
    fr = MutableDenseMatrix(
        [
            [cos(theta), -sin(theta), 0, radius],
            [
                sin(theta) * cos(alpha),
                cos(theta) * cos(alpha),
                -sin(alpha),
                -displacement * sin(alpha),
            ],
            [
                sin(theta) * sin(alpha),
                cos(theta) * sin(alpha),
                cos(alpha),
                displacement * cos(alpha),
            ],
            [0, 0, 0, 1],
        ]
    )
    return fr


def frame_mult(
    frames: list[MutableDenseMatrix], first: int, last: int
) -> MutableDenseMatrix:
    prod: MutableDenseMatrix = eye(len(frames[first][:, 0]))
    for i in range(first, last):
        prod = prod.multiply(frames[i])
    return prod


def extract_rot(frame: MutableDenseMatrix) -> MutableDenseMatrix:
    return frame[:3, :3]


if __name__ == "__main__":
    init_printing()

    d = [d0, d1, d2, d3, d4, d5] = symbols("d1:7", real=True)
    t = [t0, t1, t2, t3, t4, t5] = symbols("t1:7", real=True)
    a = [a0, a1, a2, a3, a4, a5] = symbols("a1:7", real=True)
    r = [r0, r1, r2, r3, r4, r5] = symbols("r1:7", real=True)

    a_values = {a0: 0, a1: rad(90), a2: 0, a3: rad(90), a4: -rad(90), a5: rad(90)}
    r_values = {r0: 0, r1: 0, r2: 250.201, r3: 0, r4: 0, r5: 0}
    d_values = {d0: 167.719, d1: 19.898, d2: -0.39, d3: 231.03, d4: 2.785, d5: 140.54}
    # t_values = {t0: 0, t1: 0, t2: 0, t3: 0, t4: 0, t5: 0}
    vals = a_values | r_values | d_values  # | t_values

    # Create frames
    f: list[MutableDenseMatrix] = []
    for i in range(6):
        f.append(frame(t[i], a[i], r[i], d[i]))

    # T_0-6
    T_0_6 = frame_mult(f, 0, 6)

    # Position
    T_0_3 = frame_mult(f, 0, 3)

    # Orientation
    T_3_6 = frame_mult(f, 3, 6)

    T03e = T_0_3.subs(vals)[:3, :3]
    print("Position matrix:\n")
    pprint(
        simplify(T03e.evalf(subs={t0: rad(124.750), t1: rad(62.082), t2: rad(-24.685)}))
    )

    print("Orientation matrix: \n")
    T36e = T_3_6.subs(vals)[:3, :3]
    pprint(T36e)

    exit()
    t_values = {t0: 0, t1: 0, t2: 0, t3: pi / 3, t4: pi / 5, t5: pi / 2}
    print(f"\nSub {t_values.values()} for each theta:\n")
    T36 = T36e.subs(t_values)
    pprint(T36)

    print("\nExtract individual angles from rotation matrix:")
    print(f"\n{T36e[1,2]} = {T36[1, 2]}")
    T4 = acos(-T36[1, 2])
    print(f"So t4 = acos({T36[1,2]}0 = {T4}")

    print(f"\n{T36e[1,0]} = {T36[1,0]}")
    T5 = acos(T36[1, 0] / sin(T4))
    print(f"So t5 = acos({T36[1,0]}/sin({T4})) = {T5}")

    print(f"\n{T36e[0,2]} = {T36[0,2]}")
    T3 = acos(T36[0, 2] / sin(T4))
    print(f"So t3 = acos({T36[0,2]}/sin({T4})) = {T3}\n")

    # print("Note that -cos(t4) is isolated, so it can be extracted first then used to solve the rest")
