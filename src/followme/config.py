"""All tunable numbers for the Follow-Me MVP (plan Step 3). SI units: metres, seconds, radians."""
from dataclasses import dataclass, field

# --- simulation ------------------------------------------------------------------------------------------------
DT = 0.1  # s, control and simulation tick
DURATION = 60.0  # s, default run length
SEED = 42

# --- acceptance (plan section 1) -------------------------------------------------------------------------------
BAND = (1.0, 2.5)  # R3: centre distance cart <-> traveller, while behind the traveller
BAND_MIN_FRACTION = 0.85  # R3
MAX_DISTANCE = 4.0  # R3: never further than this
MIN_CLEARANCE = 0.3  # R4: cart footprint <-> any person / obstacle footprint


@dataclass(frozen=True)
class CartConfig:
    v_max: float = 1.5  # m/s (no reversing)
    w_max: float = 1.5  # rad/s
    a_acc: float = 1.2  # m/s^2, max acceleration (1.0 left too little recovery after a stop; see notes)
    a_brake: float = 2.5  # m/s^2, max deceleration (also used by the safety filter)
    alpha_max: float = 3.0  # rad/s^2, max angular acceleration
    follow_distance: float = 1.5  # m of traveller path-length kept between cart and traveller
    crumb_spacing: float = 0.2  # m, breadcrumb spacing
    k_gap: float = 0.9  # 1/s, speed correction per metre of gap error
    lookahead_min: float = 0.6  # m, pure-pursuit lookahead (grows with speed)
    lookahead_gain: float = 0.5  # s
    avoid_radius: float = 1.5  # m, obstacles / people closer than this push the heading
    avoid_gain: float = 0.35  # heading push weight per (1/clearance - 1/avoid_radius)
    avoid_predict: float = 0.8  # s - moving people are avoided at their position this far ahead
    cone_half_angle: float = 0.5236  # rad (30 deg) - safety stop cone ahead
    cone_stop_clearance: float = 0.5  # m - stop if anything is closer than this inside the cone
    hold_distance: float = 1.5  # m - traveller stopped and closer than this: hold position
    safety_margin: float = 0.4  # m - predictive filter keeps at least this clearance (> MIN_CLEARANCE)
    hard_clearance: float = 0.32  # m - inside the margin, moves that don't get closer are allowed down to this
    safety_horizon: float = 1.2  # s - pedestrians predicted at constant velocity this far ahead
    predict_uncertainty: float = 0.2  # m/s - predicted people grow by this much per second of look-ahead
    # footprint (from out/scene.json at load time; these are fallbacks)
    length: float = 0.93
    width: float = 0.62


@dataclass(frozen=True)
class PeopleConfig:
    radius: float = 0.3  # m, person footprint (H1 torso circle in scene.json is 0.6 m wide)
    traveller_speed: float = 1.2  # m/s
    traveller_stop_at: int = 3  # index into the route: the check-in stop (wp_4)
    traveller_stop_time: float = 3.0  # s
    crosser_speed: float = 1.2  # m/s
    walker_speed: tuple = (0.8, 1.2)  # m/s, random walkers (uniform)
    walker_pause: tuple = (0.0, 2.0)  # s, pause at each random goal
    tau: float = 0.5  # s, social-force relaxation time
    a_people: float = 2.0  # social-force strength between people
    b_people: float = 0.3  # m
    a_static: float = 3.0  # repulsion from walls / pillars / benches
    b_static: float = 0.2  # m
    a_cart: float = 4.0  # people keep away from the cart (they can see it)
    b_cart: float = 0.35  # m
    cart_personal_space: float = 0.35  # m - a person never steps closer than this to the cart
    blocked_patience: float = 1.0  # s - a person blocked this long walks around (new goal / side-step)
    # scripted crossers: (name, goal, x of the crossing line, seconds after the traveller passes that line)
    crossers: tuple = (("ped_1", (-6.0, -3.0), -6.0, 0.35), ("ped_2", (2.0, 4.5), 2.0, 0.35))
    walkers: tuple = ("ped_3", "ped_4")


@dataclass(frozen=True)
class Config:
    dt: float = DT
    duration: float = DURATION
    seed: int = SEED
    cart: CartConfig = field(default_factory=CartConfig)
    people: PeopleConfig = field(default_factory=PeopleConfig)
