import optas
import numpy as np


class AdmittanceController(object):
    def __init__(
        self,
        node,
        robot_description,
        end_effector_link,
    ) -> None:
        # Setup class attributes
        self.node = node

        # Setup robot
        self.robot_model = optas.RobotModel(
            urdf_string=robot_description, time_derivs=[1]
        )
        self.name = self.robot_model.get_name()

        # Setup optimization builder
        T = 1
        builder = optas.OptimizationBuilder(
            T, robots=self.robot_model, derivs_align=True
        )

        # Setup parameters
        f_ext = builder.add_parameter("f_ext", 6)
        qc = builder.add_parameter("qc", self.robot_model.ndof)
        dt = builder.add_parameter("dt")

        # Get model state
        dq = builder.get_model_state(self.name, 0, time_deriv=1)
        q = qc + dt * dq

        # Compute jacobian at current state
        Jc = self.robot_model.get_global_link_geometric_jacobian(end_effector_link, qc)

        # Compute current eff position
        pc = self.robot_model.get_global_link_position(end_effector_link, qc)

        # Compute goal end-effector velocity
        gain = optas.DM([0.01, 0.01, 0.01, 0.25, 0.25, 0.25])
        dx_lim = optas.DM([0.04, 0.04, 0.04, 0.2, 0.2, 0.2])
        dxg = optas.clip(gain * f_ext, -dx_lim, dx_lim)
        self.goal_eff_velocity = optas.Function("dxg", [f_ext], [dxg])

        # Cost: goal end-effector velocity
        dx = Jc @ dq
        builder.add_cost_term("goal_eff_velocity", optas.sumsqr(dx - dxg))

        # Compute next eff position
        p = pc + dt * dx[:3]

        # Constraint: y limit
        py = p[1]
        builder.add_bound_inequality_constraint("y_bound", -0.3, py, 0.3)

        # Cost: minimize joint velocity
        w = 0.01
        builder.add_cost_term("min_dq", w * optas.sumsqr(dq))

        # Constraint: joint position limits
        builder.add_bound_inequality_constraint(
            "joint_position_limits",
            self.robot_model.lower_actuated_joint_limits,
            q,
            self.robot_model.upper_actuated_joint_limits,
        )

        # Setup solver
        self.solver = optas.CasADiSolver(builder.build()).setup("qpoases")
        self.solution = None

    def __call__(self, q, f_ext, dt):
        # Compute/report goal end-effector velocity
        # dxg = self.goal_eff_velocity(f_ext).toarray().flatten()
        # self.node.get_logger().info("dxg=" + repr(dxg))

        # Reset optimization problem
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)

        self.solver.reset_parameters({"f_ext": f_ext, "dt": dt, "qc": q})

        # Solve problem
        self.solution = self.solver.solve()

        # Compute goal state
        dq = self.solution[f"{self.name}/dq"].toarray().flatten()
        qg = q + dt * dq

        return qg
