from google.protobuf.text_format import Parse
from src.lie_ilqr_binding import LieILQR
import src.trajectory_pb2 as traj
import src.ilqr_options_pb2 as opts
import numpy as np

opts_str = """
line_search_params {
  step_update: 0.5
  desired_reduction_frac: 0.5
  max_iters: 10
}
convergence_criteria {
  rtol: 1e-12
  atol: 1e-12
  max_iters: 100.0
}
"""


def main():
    desired_traj = traj.LieTrajectory(
        points=[
            traj.LieTrajectoryPoint(
                time_s=0.0,
                state=traj.SE3(
                    rot=traj.Quaternion(
                        w=1.0,
                    )
                ),
                control=traj.SE3(
                    rot=traj.Quaternion(
                        w=1.0,
                    )
                ),
            ),
            traj.LieTrajectoryPoint(
                time_s=1.0,
                state=traj.SE3(
                    translation=traj.Vec3(
                        c0=1.0,
                    ),
                    rot=traj.Quaternion(w=1.0),
                ),
                control=traj.SE3(
                    rot=traj.Quaternion(
                        w=1.0,
                    )
                ),
            ),
            traj.LieTrajectoryPoint(
                time_s=2.0,
                state=traj.SE3(
                    rot=traj.Quaternion(
                        w=1.0,
                    )
                ),
                control=traj.SE3(
                    rot=traj.Quaternion(
                        w=1.0,
                    )
                ),
            ),
        ]
    )

    options = opts.ILQROptions(
        line_search_params=opts.LineSearchParams(
            step_update=0.5,
            desired_reduction_frac=0.5,
            max_iters=10,
        ),
        convergence_criteria=opts.ConvergenceCriteria(
            rtol=1e-12,
            atol=1e-12,
            max_iters=100,
        ),
    )

    Q = np.eye(6)
    R = np.eye(6)

    ilqr = LieILQR(Q, R, desired_traj, options)


if __name__ == "__main__":
    main()
data = []
