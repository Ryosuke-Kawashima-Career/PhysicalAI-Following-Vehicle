"""Airport concourse layout for the Follow-Me MVP (plan Step 2).

Z-up, metres, concourse centred at the origin, open end at x = -15.
This is the *desired* layout: `cw_build_scene.py` pushes it to the environment, and `export_scene.py`
reads the environment back into `out/scene.json` (which is what the local world loads).

`pos` is the object's real geometric centre (x, y) and `size` its real extent (x, y, z) in metres; for actors
`pos` z is 0 (standing on the floor). `cw_build_scene.py` converts this to twin scale/position with GEOMETRY below.
Catalog robots keep scale 1, and `size` is their measured footprint for the local world.
"""

ASSETS = {
    "cube": "f69e1538-9a4a-458c-bbd0-cc28632adce9",  # cyberwave/generic_cube (primitive)
    "cylinder": "73c6a84b-4fb9-4406-94ea-52deb12bc81e",  # cyberwave/cylinder (primitive)
    "mir250": "7280ed31-308c-4312-b90d-0b9bbce6b127",  # mir/mir-250, indoor AMR (follow-me cart)
    "h1": "0824c184-2e1f-4eaa-894b-48614577d02c",  # unitree/unitree-h1, ~1.8 m humanoid (stand-in for people)
}
PRIMITIVES = {"cube", "cylinder"}

# Measured in the Cyberwave viewer (three.js world bounding boxes, 2026-11-14). The primitives are NOT unit-sized
# or centred: `native` = mesh size at scale 1, `center` = mesh centre in local coords per unit of scale.
# For robots, `origin_z` = height of the model origin above the floor when standing, and `center_xy` = footprint
# centre in the model frame (spec `pos` is the footprint centre). The H1 keeps (0, 0): its circle sits on the torso,
# and only its hands reach further forward.
GEOMETRY = {
    "cube": {"native": (1.0, 1.0, 1.0), "center": (0.0, 0.0478, 0.5)},  # mesh spans local z 0..1
    "cylinder": {"native": (2.0, 2.0, 2.0), "center": (0.0, 0.0, 1.0393)},
    "mir250": {"origin_z": 0.0, "center_xy": (0.038, 0.0)},  # base_footprint on the floor; bbox 0.93 x 0.62 x 0.31
    "h1": {"origin_z": 1.036},  # origin at the pelvis; feet 1.036 m below it; bbox 0.60 x 0.56 x 1.80 (hands reach 0.5 m forward of the origin)
}

H = 3.0  # wall / pillar height


def _obj(name, group, asset, pos, size, yaw=0.0, shape=None, static=True):
    if shape is None:
        shape = "circle" if asset in ("cylinder", "h1") else "box"
    return dict(name=name, group=group, asset=asset, pos=pos, size=size, yaw=yaw, shape=shape, static=static)


OBJECTS = [
    # --- concourse -----------------------------------------------------------------------------------------
    _obj("floor", "concourse", "cube", (0.0, 0.0, -0.025), (30.0, 12.0, 0.05)),
    _obj("wall_north", "concourse", "cube", (0.0, 6.1, H / 2), (30.0, 0.2, H)),
    _obj("wall_south", "concourse", "cube", (0.0, -6.1, H / 2), (30.0, 0.2, H)),
    _obj("wall_east", "concourse", "cube", (15.1, 0.0, H / 2), (0.2, 12.4, H)),
    # --- obstacles -----------------------------------------------------------------------------------------
    _obj("pillar_1", "obstacles", "cylinder", (-8.0, 2.0, H / 2), (0.6, 0.6, H)),
    _obj("pillar_2", "obstacles", "cylinder", (0.0, 2.0, H / 2), (0.6, 0.6, H)),
    _obj("pillar_3", "obstacles", "cylinder", (8.0, 2.0, H / 2), (0.6, 0.6, H)),
    _obj("bench_1", "obstacles", "cube", (-4.0, -4.0, 0.225), (2.0, 0.6, 0.45)),
    _obj("bench_2", "obstacles", "cube", (4.0, -4.0, 0.225), (2.0, 0.6, 0.45)),
    _obj("luggage_pile", "obstacles", "cube", (12.0, -1.0, 0.4), (1.0, 0.8, 0.8)),
    # --- actors (moved by the local loop in Steps 3-4) --------------------------------------------------------
    _obj("follow_cart", "actors", "mir250", (-13.5, 0.0, 0.0), (0.93, 0.62, 0.31), shape="box", static=False),
    _obj("traveller", "actors", "h1", (-12.0, 0.0, 0.0), (0.6, 0.56, 1.8), static=False),
    _obj("ped_1", "actors", "h1", (-6.0, 3.5, 0.0), (0.6, 0.56, 1.8), yaw=-90.0, static=False),
    _obj("ped_2", "actors", "h1", (2.0, -2.5, 0.0), (0.6, 0.56, 1.8), yaw=90.0, static=False),
    _obj("ped_3", "actors", "h1", (6.0, 4.0, 0.0), (0.6, 0.56, 1.8), yaw=180.0, static=False),
    _obj("ped_4", "actors", "h1", (10.0, -3.5, 0.0), (0.6, 0.56, 1.8), yaw=135.0, static=False),
]

# Traveller route (floor height). Clear of pillars (>= 1.6 m), benches and the luggage pile.
WAYPOINT_COLLECTION = "traveller_route"
WAYPOINTS = [(-12.0, 0.0), (-9.0, 0.0), (-5.0, -1.0), (-1.0, 0.2), (3.0, -1.2), (7.0, 0.0), (10.0, 1.0), (13.0, 2.5)]

# Old Step 1 test twin: reused as pillar_2 (same cylinder asset) instead of being deleted.
ADOPT = {"step1_test_cylinder": "pillar_2"}
