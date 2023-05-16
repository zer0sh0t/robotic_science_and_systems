action = (0, 63, 0)
px, py, rot_id = action
H = 64
W = 64

for rotation_deg in [0, 90, 180, 270]:
    # Compute the new pixel location and gripper rotation
    if rotation_deg == 90:
        # rot_action = (py, H - 1 - px, 1 - rot_id)
        rot_action = (W - 1 - py, px, 1 - rot_id)
    elif rotation_deg == 180:
        rot_action = (H - 1 - px, W - 1 - py, rot_id)
    elif rotation_deg == 270:
        # rot_action = (W - 1 - py, px, 1 - rot_id)
        rot_action = (py, H - 1 - px, 1 - rot_id)
    else:
        rot_action = action
    print(f"{rotation_deg}: {rot_action}")
