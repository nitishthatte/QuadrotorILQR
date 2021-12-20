from google.protobuf.text_format import Parse
from src.lie_ilqr_binding import LieILQR
import src.trajectory_pb2 as traj
import src.ilqr_options_pb2 as opts
import numpy as np
import matplotlib.pyplot as plt


def make_se3(x=0):
    return traj.SE3(
        translation=traj.Vec3(
            c0=x,
            c1=0.0,
            c2=0.0,
        ),
        rot=traj.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
    )


def extract_series(trajectory: traj.LieTrajectory):
    time_s = [pt.time_s for pt in trajectory.points]
    state_x = [pt.state.translation.c0 for pt in trajectory.points]
    control_x = [pt.control.translation.c0 for pt in trajectory.points]
    return time_s, state_x, control_x


def main():
    num_pts = 10
    time_s = np.linspace(0, 2 * np.pi, num_pts)
    initial_traj = traj.LieTrajectory(
        points=[
            traj.LieTrajectoryPoint(
                time_s=time_s[i],
                state=make_se3(),
                control=make_se3(),
            )
            for i in range(num_pts)
        ]
    )
    desired_traj = traj.LieTrajectory(
        points=[
            traj.LieTrajectoryPoint(
                time_s=time_s[i],
                state=make_se3(x=np.sin(time_s[i])),
                control=make_se3(),
            )
            for i in range(num_pts)
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
    opt_traj = ilqr.solve(initial_traj)

    init_time_s, init_state_x, init_control_x = extract_series(initial_traj)
    desired_time_s, desired_state_x, desired_control_x = extract_series(desired_traj)
    opt_time_s, opt_state_x, opt_control_x = extract_series(opt_traj)

    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(init_time_s, init_state_x, label="initial")
    ax[0].plot(desired_time_s, desired_state_x, label="desired")
    ax[0].plot(opt_time_s, opt_state_x, label="optimal")
    ax[0].legend()
    ax[0].set_ylabel("state x")

    ax[1].plot(init_time_s, init_control_x, label="initial")
    ax[1].plot(desired_time_s, desired_control_x, label="desired")
    ax[1].plot(opt_time_s, opt_control_x, label="optimal")
    ax[1].legend()
    ax[1].set_ylabel("control x")

    plt.show()


if __name__ == "__main__":
    main()
data = []
