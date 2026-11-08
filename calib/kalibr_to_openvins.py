#!/usr/bin/env python3
"""Turn Kalibr calibration results into an OpenVINS config folder.

Inputs
  --camchain  Kalibr camera-IMU result, e.g. calib-camchain-imucam.yaml (from kalibr_calibrate_imu_camera)
  --imu       Kalibr IMU noise file used for that calibration (calib/imu.yaml)
  --template  OpenVINS estimator_config.yaml to start from (default: calib/openvins/estimator_config.yaml)

Output folder (--out), ready for `roslaunch ov_msckf serial.launch config_path:=<out>/estimator_config.yaml`
  estimator_config.yaml      copy of the template, max_cameras set to the number of cameras found
  kalibr_imucam_chain.yaml   per camera: T_imu_cam (camera->IMU, the inverse of Kalibr's T_cam_imu),
                             intrinsics, distortion, resolution, topic, timeshift_cam_imu
  kalibr_imu_chain.yaml      IMU noise densities / random walks, rate, topic; ideal IMU intrinsics

Conventions
  Kalibr  T_cam_imu   maps IMU-frame points into the camera frame.
  OpenVINS T_imu_cam  is [R_CtoI | p_CinI], i.e. maps camera-frame points into the IMU frame.
  Both use t_imu = t_cam + timeshift_cam_imu, so the time shift is copied unchanged.

Usage
  python calib/kalibr_to_openvins.py --camchain calib-camchain-imucam.yaml --imu calib/imu.yaml \
      --out calib/openvins/crazyflie [--meta recordings/<stamp>/meta.json]
"""
import argparse
import json
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TEMPLATE = os.path.join(HERE, "openvins", "estimator_config.yaml")
DIST_MODELS = {"radtan": "radtan", "equidistant": "equidistant", "equi": "equidistant"}
IMU_KEYS = ("accelerometer_noise_density", "accelerometer_random_walk",
            "gyroscope_noise_density", "gyroscope_random_walk")


def die(msg):
    sys.exit(f"error: {msg}")


def warn(msg):
    print(f"warning: {msg}", file=sys.stderr)


def load_yaml(path):
    with open(path) as f:
        text = f.read()
    # OpenCV-style "%YAML:1.0" headers are not valid for PyYAML
    text = re.sub(r"^%YAML:1\.0.*\n", "", text)
    return yaml.safe_load(text)


def invert_se3(T):
    """Inverse of a 4x4 rigid transform given as nested lists."""
    R = [row[:3] for row in T[:3]]
    t = [row[3] for row in T[:3]]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    ti = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [Rt[i] + [ti[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def check_rotation(T, name):
    R = [row[:3] for row in T[:3]]
    err = max(abs(sum(R[k][i] * R[k][j] for k in range(3)) - (1.0 if i == j else 0.0))
              for i in range(3) for j in range(3))
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    if err > 1e-4 or abs(det - 1.0) > 1e-4:
        die(f"{name} is not a rotation (orthogonality error {err:.2e}, det {det:.4f})")
    if [len(r) for r in T] != [4, 4, 4, 4] or [float(x) for x in T[3]] != [0.0, 0.0, 0.0, 1.0]:
        die(f"{name} is not a 4x4 homogeneous transform")


def fmt_list(v):
    return "[" + ", ".join(repr(float(x)) for x in v) + "]"


def fmt_matrix(M, indent="    "):
    return "\n".join(f"{indent}- [{', '.join(f'{float(x) + 0.0: .12f}' for x in row)}]" for row in M)


def camera_block(name, cam):
    for key in ("T_cam_imu", "camera_model", "intrinsics", "distortion_model",
                "distortion_coeffs", "resolution", "rostopic"):
        if key not in cam:
            die(f"{name}: missing '{key}' (is this the *-camchain-imucam.yaml from "
                f"kalibr_calibrate_imu_camera, not the camera-only camchain?)")
    if cam["camera_model"] != "pinhole":
        die(f"{name}: camera_model '{cam['camera_model']}' is not supported by OpenVINS (use pinhole)")
    dist = DIST_MODELS.get(cam["distortion_model"])
    if dist is None:
        die(f"{name}: distortion_model '{cam['distortion_model']}' is not supported by OpenVINS "
            f"(radtan or equidistant)")
    if len(cam["intrinsics"]) != 4 or len(cam["distortion_coeffs"]) != 4:
        die(f"{name}: expected 4 intrinsics and 4 distortion coefficients")

    T_cam_imu = cam["T_cam_imu"]
    check_rotation(T_cam_imu, f"{name}.T_cam_imu")
    T_imu_cam = invert_se3(T_cam_imu)
    shift = float(cam.get("timeshift_cam_imu", 0.0))
    p = [row[3] for row in T_imu_cam[:3]]
    overlaps = cam.get("cam_overlaps", []) or []

    lines = [
        f"{name}:",
        "  T_imu_cam: # rotation from camera to IMU R_CtoI, position of camera in IMU p_CinI",
        fmt_matrix(T_imu_cam),
        f"  cam_overlaps: [{', '.join(str(int(i)) for i in overlaps)}]",
        "  camera_model: pinhole",
        f"  distortion_coeffs: {fmt_list(cam['distortion_coeffs'])}",
        f"  distortion_model: {dist}",
        f"  intrinsics: {fmt_list(cam['intrinsics'])} # fu, fv, cu, cv",
        f"  resolution: [{int(cam['resolution'][0])}, {int(cam['resolution'][1])}]",
        f"  rostopic: {cam['rostopic']}",
        f"  timeshift_cam_imu: {shift!r} # t_imu = t_cam + shift [s]",
    ]
    summary = (f"{name}: {cam['resolution'][0]}x{cam['resolution'][1]} {dist}, "
               f"f=({cam['intrinsics'][0]:.1f}, {cam['intrinsics'][1]:.1f}), "
               f"|p_CinI|={math.sqrt(sum(x * x for x in p)) * 100:.1f} cm, "
               f"timeshift={shift * 1000:+.2f} ms")
    return "\n".join(lines), summary, cam


def imu_block(imu):
    missing = [k for k in IMU_KEYS + ("update_rate", "rostopic") if k not in imu]
    if missing:
        die(f"IMU file is missing {', '.join(missing)}")
    eye3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    zero3 = [[0.0] * 3 for _ in range(3)]
    lines = [
        "imu0:",
        "  T_i_b:",
        fmt_matrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
        f"  accelerometer_noise_density: {float(imu['accelerometer_noise_density']):.4e} # [m/s^2/sqrt(Hz)]",
        f"  accelerometer_random_walk: {float(imu['accelerometer_random_walk']):.4e} # [m/s^3/sqrt(Hz)]",
        f"  gyroscope_noise_density: {float(imu['gyroscope_noise_density']):.4e} # [rad/s/sqrt(Hz)]",
        f"  gyroscope_random_walk: {float(imu['gyroscope_random_walk']):.4e} # [rad/s^2/sqrt(Hz)]",
        f"  rostopic: {imu['rostopic']}",
        "  time_offset: 0.0",
        f"  update_rate: {float(imu['update_rate'])}",
        '  model: "kalibr"',
    ]
    for key, M in (("Tw", eye3), ("R_IMUtoGYRO", eye3), ("Ta", eye3), ("R_IMUtoACC", eye3), ("Tg", zero3)):
        lines += [f"  {key}:", fmt_matrix(M)]
    return "\n".join(lines)


def patch_template(text, n_cams):
    def setval(text, key, value):
        new, n = re.subn(rf"^({key}:\s*)[^#\n]*", rf"\g<1>{value} ", text, flags=re.M)
        if n == 0:
            new = text.rstrip("\n") + f"\n{key}: {value}\n"
        return new
    text = setval(text, "max_cameras", str(n_cams))
    if n_cams == 1:
        text = setval(text, "use_stereo", "false")
    text = setval(text, "relative_config_imu", '"kalibr_imu_chain.yaml"')
    text = setval(text, "relative_config_imucam", '"kalibr_imucam_chain.yaml"')
    if not text.startswith("%YAML:1.0"):
        text = "%YAML:1.0\n" + text
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camchain", required=True, help="Kalibr *-camchain-imucam.yaml")
    ap.add_argument("--imu", required=True, help="Kalibr IMU yaml (calib/imu.yaml)")
    ap.add_argument("--out", required=True, help="output config folder")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE, help="estimator_config.yaml to start from")
    ap.add_argument("--euroc-config", help="(legacy) OpenVINS config folder whose estimator_config.yaml "
                                           "is used as the template instead")
    ap.add_argument("--meta", help="recording meta.json, to check the image size matches the calibration")
    args = ap.parse_args()

    template = args.template
    if args.euroc_config:
        template = os.path.join(args.euroc_config, "estimator_config.yaml")
    if not os.path.isfile(template):
        die(f"template not found: {template}")

    chain = load_yaml(args.camchain) or {}
    cams = sorted((k for k in chain if re.fullmatch(r"cam\d+", k)), key=lambda k: int(k[3:]))
    if not cams:
        die(f"no cam0/cam1/... entries in {args.camchain}")
    if cams != [f"cam{i}" for i in range(len(cams))]:
        die(f"camera ids must be consecutive from cam0, got {cams}")
    imu = load_yaml(args.imu) or {}

    blocks, summaries = [], []
    for name in cams:
        block, summary, cam = camera_block(name, chain[name])
        blocks.append(block)
        summaries.append(summary)
        shift_ms = abs(float(cam.get("timeshift_cam_imu", 0.0))) * 1000
        if shift_ms > 50:
            warn(f"{name}: |timeshift_cam_imu| = {shift_ms:.1f} ms is large; check the clock fit in convert.py")
        if cam["rostopic"] != "/cam0/image_raw" and name == "cam0":
            warn(f"cam0 topic is {cam['rostopic']}, convert.py writes /cam0/image_raw")

    if imu.get("rostopic") != "/imu0":
        warn(f"IMU topic is {imu.get('rostopic')}, convert.py writes /imu0")

    if args.meta:
        with open(args.meta) as f:
            meta = json.load(f)
        res = [int(x) for x in chain["cam0"]["resolution"]]
        if res != [int(meta["width"]), int(meta["height"])]:
            die(f"cam0 was calibrated at {res[0]}x{res[1]} but the recording is "
                f"{meta['width']}x{meta['height']}; recalibrate at the flight resolution")

    os.makedirs(args.out, exist_ok=True)
    header = (f"%YAML:1.0\n# Generated by calib/kalibr_to_openvins.py from {os.path.basename(args.camchain)}"
              f" and {os.path.basename(args.imu)}\n\n")
    with open(os.path.join(args.out, "kalibr_imucam_chain.yaml"), "w") as f:
        f.write(header + "\n".join(blocks) + "\n")
    with open(os.path.join(args.out, "kalibr_imu_chain.yaml"), "w") as f:
        f.write(header + imu_block(imu) + "\n")
    with open(template) as f:
        text = patch_template(f.read(), len(cams))
    with open(os.path.join(args.out, "estimator_config.yaml"), "w") as f:
        f.write(text)

    for s in summaries:
        print(s)
    print(f"imu0: {imu['rostopic']} @ {float(imu['update_rate']):g} Hz")
    print(f"wrote {args.out}/{{estimator_config,kalibr_imucam_chain,kalibr_imu_chain}}.yaml")


if __name__ == "__main__":
    main()
