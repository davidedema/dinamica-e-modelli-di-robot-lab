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
        self.sphere_name_to_id = {}

    def set_state(self, q, dq):
        """Set joint positions and velocities and update kinematics."""
        self.data.qpos[:] = q
        self.data.qvel[:] = dq
        self.data.qacc[:] = np.zeros_like(dq) 
        if self.open_viz:
            self.render()

    def step(self, ctrl, dt=None):
        """Step simulation forward by dt using ctrl torques, returns updated state."""
        self.data.ctrl[:] = ctrl

        step_iter = 1 if dt is None else max(1, int(dt / self.model.opt.timestep))

        for _ in range(step_iter):
            mujoco.mj_step(self.model, self.data)

        # mujoco.mj_forward(self.model, self.data)
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

    def close(self):
        if self.open_viz:
            self.viz.close()
            del self.viz
    def add_visual_sphere(self, name, center, radius, rgba):
        """Adds a visual sphere to the scene."""
        if self.open_viz:
            scene = self.viz.user_scn
            if scene.ngeom >= scene.maxgeom:
                print("ERROR: Max number of geom in scene has been reached!")
                return
            self.sphere_name_to_id[name] = scene.ngeom
            scene.ngeom += 1  # increment ngeom
            # initialise a new sphere and add it to the scene
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom-1],
                                mujoco.mjtGeom.mjGEOM_SPHERE, radius*np.ones(3),
                                center.astype(np.float32), np.eye(3).flatten(), rgba.astype(np.float32))

    def move_visual_sphere(self, name, pos):
        if self.open_viz:
            geom_id = self.sphere_name_to_id[name]
            scene = self.viz.user_scn
            scene.geoms[geom_id].pos = pos