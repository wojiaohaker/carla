#!/usr/bin/env python3
"""
Convert a UE world coordinate (cm) to geographic (lat/lon/alt)
using the CARLA Python API.

Usage:
    python3 uegeo.py
"""

import carla

# ── UE coordinate in centimetres ──────────────────────────────────────────────
UE_X_CM = -11925.562500
UE_Y_CM =  8718.526367
UE_Z_CM =  15.239990

# ── Convert cm → metres (Python API uses metres) ──────────────────────────────
ue_loc_m = carla.Location(
    x=UE_X_CM / 100.0,
    y=UE_Y_CM / 100.0,
    z=UE_Z_CM / 100.0,
)

print(f"UE coordinate (cm): X={UE_X_CM:.6f}, Y={UE_Y_CM:.6f}, Z={UE_Z_CM:.6f}")
print(f"UE coordinate (m):  X={ue_loc_m.x:.6f}, Y={ue_loc_m.y:.6f}, Z={ue_loc_m.z:.6f}")

# ── Connect to CARLA ──────────────────────────────────────────────────────────
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
map = world.get_map()

# ── Show map geo-reference (origin) ───────────────────────────────────────────
origin = map.get_georeference()
print(f"\nMap origin (geo-reference):")
print(f"  lat={origin.latitude:.8f}, lon={origin.longitude:.8f}, alt={origin.altitude:.4f}")

# ── UE → Geo ──────────────────────────────────────────────────────────────────
geo_loc = map.transform_to_geolocation(ue_loc_m)
print(f"\nGeographic coordinate:")
print(f"  latitude  = {geo_loc.latitude:.8f}")
print(f"  longitude = {geo_loc.longitude:.8f}")
print(f"  altitude  = {geo_loc.altitude:.4f} m")

# ── Round-trip verification ───────────────────────────────────────────────────
back = map.geolocation_to_transform(geo_loc)
print(f"\nRound-trip check (should match UE m):")
print(f"  X={back.x:.6f}, Y={back.y:.6f}, Z={back.z:.6f}")
print(f"\nRound-trip (cm):")
print(f"  X={back.x * 100.0:.6f}, Y={back.y * 100.0:.6f}, Z={back.z * 100.0:.6f}")
