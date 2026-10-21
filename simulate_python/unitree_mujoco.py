import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading
import numpy as np
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(str((Path(__file__).parent / config.ROBOT_SCENE).resolve()))
mj_data = mujoco.MjData(mj_model)


if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

parkour_observations = None
if getattr(config, "PARKOUR_OBSERVATIONS", False):
    if config.ROBOT != "g1" or mj_model.nu != 29:
        raise ValueError("Parkour observations require G1 29DOF")
    from parkour_observation import ParkourObservationPublisher
    parkour_observations = ParkourObservationPublisher(
        mj_model, mj_data, config.PARKOUR_OBSERVATION_PORT)
    print(f"Parkour observations -> 127.0.0.1:{config.PARKOUR_OBSERVATION_PORT}; inference runs in C++")

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        with locker:
            motor_active = unitree.UpdateMotorControl()
            if parkour_observations is not None:
                parkour_observations.adapter.set_motor_control_active(motor_active)
            # Refresh kinematics and COM velocities for the actual attachment point.
            mujoco.mj_forward(mj_model, mj_data)
            if config.ENABLE_ELASTIC_BAND:
                elastic_band.Apply(mj_model, mj_data, band_attached_link)
            if parkour_observations is not None:
                parkour_observations.update()
            if parkour_observations is not None:
                # Apply the tighter of training and model torque limits, in motor order.
                adapter = parkour_observations.adapter
                ids = adapter.actuators
                mj_data.ctrl[ids] = np.clip(mj_data.ctrl[ids], -adapter.limits, adapter.limits)
            mujoco.mj_step(mj_model, mj_data)

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    depth_preview = None
    if parkour_observations is not None and getattr(config, "PARKOUR_DEPTH_PREVIEW", False):
        try:
            from depth_preview import DepthPreview
            depth_preview = DepthPreview()
        except (ImportError, RuntimeError) as error:
            print(f"Depth preview unavailable: {error}")
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()

    # Keep OpenCV GUI calls on the main thread; physics only copies its latest frame.
    try:
        while sim_thread.is_alive() and viewer.is_running():
            if depth_preview is not None:
                depth_preview.update(*parkour_observations.preview_frame())
            time.sleep(config.VIEWER_DT)
    except KeyboardInterrupt:
        viewer.close()
    finally:
        if depth_preview is not None:
            depth_preview.close()
    sim_thread.join()
    viewer_thread.join()
    if parkour_observations is not None:
        parkour_observations.close()
