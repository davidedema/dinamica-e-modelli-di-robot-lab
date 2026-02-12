import mujoco
import mujoco.viewer
import numpy as np
class MujocoSim:
    def __init__(self, robot_name, dt_sim=0.002, open_viz=True):
        self.robot_name = robot_name
        self.model = mujoco.MjModel.from_xml_path(robot_name)
        self.data = mujoco.MjData(self.model)
        self.open_viz = open_viz
        if open_viz:
            self.viz = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False
            )
            self.viz.cam.distance = 3.0
        
        self.dt_sim = dt_sim
        self.model.opt.timestep = dt_sim
        self.paused = False

    def set_state(self, q, dq):
        """Set joint positions and velocities and update kinematics."""
        self.data.qpos[:] = q
        self.data.qvel[:] = dq
        self.data.qacc[:] = np.zeros_like(dq) 
        if self.open_viz:
            self.render()

    def step(self, ctrl, dt=None):
        if self.paused:
            if self.open_viz:
                self.render()
            return

        self.data.ctrl[:] = ctrl

        step_iter = 1 if dt is None else max(1, int(dt / self.model.opt.timestep))

        for _ in range(step_iter):
            mujoco.mj_step(self.model, self.data)

        if self.open_viz:
            self.render()


    def get_state(self):
        """Return joint positions, velocities, and accelerations (post-step)."""
        return self.data.qpos.copy(), self.data.qvel.copy(), self.data.qacc.copy()

    def render(self):
        if self.open_viz:
            self.viz.sync()

    def get_force(self):
        return self.data.qfrc_passive + self.data.qfrc_actuator + self.data.qfrc_applied

    def set_frame_vis(self, frame_type="body"):
        if not self.open_viz:
            return
        
        if frame_type == "body":
            self.viz.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
        elif frame_type == "geom":
            self.viz.opt.frame = mujoco.mjtFrame.mjFRAME_GEOM
        elif frame_type == "site":
            self.viz.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        else:
            self.viz.opt.frame = mujoco.mjtFrame.mjFRAME_NONE

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def toggle_pause(self):
        self.paused = not self.paused


    def close(self):
        if self.open_viz:
            self.viz.close()
            del self.viz