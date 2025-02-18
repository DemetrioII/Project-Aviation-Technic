import time
import krpc
import json
import matplotlib.pyplot as plt
import numpy as np

conn = krpc.connect(address="127.0.0.1")
vessel = conn.space_center.active_vessel

st_time = conn.space_center.ut

data = [
    {
        "pastime": [],
        "height": [],
        "velocity": [],
        "ox_velocity": [],
        "oy_velocity": [],
        "acceleration": [],
        "a_x": [],
        "a_y": []
    }
]

prev_speed = 0
prev_OXs = 0
prev_OYs = 0

initial_position = vessel.position(vessel.orbit.body.reference_frame)
initial_position_vec_length = np.linalg.norm(initial_position)

#vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

while 1:
    cur_time = conn.space_center.ut
    past_time = cur_time - st_time

    altitude = vessel.flight().mean_altitude
    if altitude >= 85000:
        vessel.control.throttle = 0.0
        break
    speed = vessel.flight(vessel.orbit.body.reference_frame).speed
    acceleration = (speed - prev_speed) / 0.1
    prev_speed = speed

    Ox_speed = vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
    ox_acc = (Ox_speed - prev_OXs) / 0.1
    prev_OXs = Ox_speed

    Oy_speed = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
    oy_acc = (Oy_speed - prev_OYs) / 0.1
    prev_OYs = Oy_speed

    engines = [part.engine for part in vessel.parts.all if part.engine and part.engine.active]

    if len(engines) > 5:
        target_thrust = 3_800_800
    else:
        # print(past_time)
        target_thrust = 770_000 #688_800

    data[0]["pastime"] += [past_time]
    data[0]["height"] += [altitude]
    data[0]["velocity"] += [speed]
    data[0]["ox_velocity"] += [Ox_speed]
    data[0]["oy_velocity"] += [Oy_speed]
    data[0]["acceleration"] += [acceleration]
    data[0]["a_x"] += [ox_acc]
    data[0]["a_y"] += [oy_acc]

    vessel.auto_pilot.target_roll = 0
    vessel.auto_pilot.engage()
    if altitude < 80000:
        target_pitch = 90 * (1 - altitude / 80000)
        vessel.auto_pilot.target_pitch_and_heading(target_pitch, 90)
    else:
        vessel.auto_pilot.target_pitch_and_heading(0, 90)

    current_thrust = sum([engine.thrust for engine in engines if engine.active])

    if current_thrust > 0:
        vessel.control.throttle = target_thrust / current_thrust

    # print(current_thrust)
    time.sleep(0.1)

with open("data_for_ksp.json", 'w', encoding="UTF-8") as file:  # запись данных в файл
    json.dump(data, file, ensure_ascii=False, indent=2)
