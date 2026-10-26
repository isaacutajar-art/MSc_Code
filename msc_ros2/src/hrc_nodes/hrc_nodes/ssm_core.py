import math

TR_S = 0.1865
TS_S = 0.100

C_M = 0.100
ZR_M = 0.0001
ZD_FRACTION = 0.14

VMAX_MPS = 2.0

WORKSPACE_MAX_DISTANCE_M = 2.0
DESIGN_VH_MPS = 1.6
SAFETY_FACTOR = 1.2

# Search resolution for maximum safe robot speed.
SPEED_STEP_MPS = 0.001

# If a detected human disappears while their last measured distance was
# below this threshold, hold the previous SSM command rather than assuming
# the workspace is clear.
NEAR_LOSS_HOLD_DISTANCE_M = 0.500


# -------------------- SSM MODEL --------------------

def stopping_distance_m(vr_mps: float) -> float:
    """
    Experimental piecewise UR3e stopping-distance model.

    Percentage is relative to VMAX_MPS = 2.0 m/s:
        <= 33%       -> 0.010 m
        >33 to <=66% -> 0.025 m
        >66 to 100%  -> 0.050 m
    """
    if vr_mps <= 0.0:
        return 0.0

    fraction = vr_mps / VMAX_MPS

    if fraction <= 0.33:
        return 0.010
    elif fraction <= 0.66:
        return 0.025
    else:
        return 0.050


def protective_distance(vr_mps: float, distance_m: float, vh_mps: float) -> dict:
    """
    Calculate the experimental protective separation distance.

        Sp = Sh + Sr + Ss + C + Zd + Zr

        Sh = vh * (Tr + Ts)
        Sr = vr * Tr
        Zd = 0.14 * measured distance

    vh_mps is signed:
        +ve -> human moving toward robot
        -ve -> human moving away

    Negative vh is intentionally retained so that a person moving away can
    permit the robot to accelerate in this experimental controller.
    """
    Sh = vh_mps * (TR_S + TS_S)
    Sr = vr_mps * TR_S
    Ss = stopping_distance_m(vr_mps)
    C = C_M
    Zd = ZD_FRACTION * distance_m
    Zr = ZR_M

    Sp = Sh + Sr + Ss + C + Zd + Zr
    Sp_design = SAFETY_FACTOR * Sp

    return {
        "Sh": Sh,
        "Sr": Sr,
        "Ss": Ss,
        "C": C,
        "Zd": Zd,
        "Zr": Zr,
        "Sp": Sp,
        "Sp_design": Sp_design,
    }


def is_safe(vr_mps: float, distance_m: float, vh_mps: float) -> bool:
    return protective_distance(vr_mps, distance_m, vh_mps)["Sp_design"] <= distance_m


def find_max_safe_speed(
    distance_m: float,
    vh_mps: float,
    upper_speed_mps: float,
    step_mps: float = SPEED_STEP_MPS,
) -> float:
    """
    Return the greatest candidate robot speed satisfying:

        SAFETY_FACTOR * Sp(vr) <= measured_distance

    Grid search is used because Ss is currently piecewise/discontinuous.
    """
    if upper_speed_mps <= 0.0:
        return 0.0

    best = 0.0
    steps = int(math.floor(upper_speed_mps / step_mps))

    for i in range(steps + 1):
        vr = i * step_mps
        if is_safe(vr, distance_m, vh_mps):
            best = vr

    if is_safe(upper_speed_mps, distance_m, vh_mps):
        best = upper_speed_mps

    return min(best, upper_speed_mps)


def calculate_vprog() -> float:
    """
    Fixed programmed/application velocity.

    Find the largest speed satisfying the design SSM condition at:
        distance = 2.0 m
        vh       = 1.6 m/s

    and cap it at VMAX_MPS.
    """
    return find_max_safe_speed(
        WORKSPACE_MAX_DISTANCE_M,
        DESIGN_VH_MPS,
        VMAX_MPS,
    )


VPROG_MPS = calculate_vprog()


def calculate_runtime(distance_mm: float, vh_mps: float) -> dict:
    distance_m = distance_mm / 1000.0

    vsafe = find_max_safe_speed(
        distance_m=distance_m,
        vh_mps=vh_mps,
        upper_speed_mps=VPROG_MPS,
    )

    speed_factor = 0.0 if VPROG_MPS <= 0.0 else vsafe / VPROG_MPS
    speed_factor = max(0.0, min(1.0, speed_factor))

    terms = protective_distance(vsafe, distance_m, vh_mps)

    return {
        "distance_m": distance_m,
        "vh_mps": vh_mps,
        "vsafe_mps": vsafe,
        "vprog_mps": VPROG_MPS,
        "speed_factor": speed_factor,
        **terms,
    }


