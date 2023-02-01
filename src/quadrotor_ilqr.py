import sys
from src.quadrotor_ilqr_binding import QuadrotorILQR
from scipy.spatial.transform import Rotation as R
import src.trajectory_pb2 as traj
import src.ilqr_options_pb2 as opts
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
import argparse
from stl import mesh
from enum import IntEnum
from copy import deepcopy


class IDX(IntEnum):
    time_s = 0
    translation_x_m = 1
    translation_y_m = 2
    translation_z_m = 3
    quaternion_w = 4
    quaternion_x = 5
    quaternion_y = 6
    quaternion_z = 7
    vel_translational_x_mps = 8
    vel_translational_y_mps = 9
    vel_translational_z_mps = 10
    vel_rotational_x_radps = 11
    vel_rotational_y_radps = 12
    vel_rotational_z_radps = 13
    control_0 = 14
    control_1 = 15
    control_2 = 16
    control_3 = 17


def extract_traj_array(trajectory: traj.QuadrotorTrajectory):
    out = np.zeros((len(trajectory.points), len(IDX)))
    extract_fcns = {
        IDX.time_s: lambda pt: pt.time_s,
        IDX.translation_x_m: lambda pt: pt.state.inertial_from_body.translation.c0,
        IDX.translation_y_m: lambda pt: pt.state.inertial_from_body.translation.c1,
        IDX.translation_z_m: lambda pt: pt.state.inertial_from_body.translation.c2,
        IDX.quaternion_w: lambda pt: pt.state.inertial_from_body.rotation.quaternion.c0,
        IDX.quaternion_x: lambda pt: pt.state.inertial_from_body.rotation.quaternion.c1,
        IDX.quaternion_y: lambda pt: pt.state.inertial_from_body.rotation.quaternion.c2,
        IDX.quaternion_z: lambda pt: pt.state.inertial_from_body.rotation.quaternion.c3,
        IDX.vel_translational_x_mps: lambda pt: pt.state.body_velocity.c0,
        IDX.vel_translational_y_mps: lambda pt: pt.state.body_velocity.c1,
        IDX.vel_translational_z_mps: lambda pt: pt.state.body_velocity.c2,
        IDX.vel_rotational_x_radps: lambda pt: pt.state.body_velocity.c3,
        IDX.vel_rotational_y_radps: lambda pt: pt.state.body_velocity.c4,
        IDX.vel_rotational_z_radps: lambda pt: pt.state.body_velocity.c5,
        IDX.control_0: lambda pt: pt.control.c0,
        IDX.control_1: lambda pt: pt.control.c1,
        IDX.control_2: lambda pt: pt.control.c2,
        IDX.control_3: lambda pt: pt.control.c3,
    }
    for field in IDX:
        out[:, field] = [extract_fcns[field](pt) for pt in trajectory.points]

    return out


def make_state(x_m=0.0, y_m=0.0, z_m=0.0, roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0):
    # quaternion in x, y, z, w format from euler angles
    quat = R.from_euler("xyz", [roll_rad, pitch_rad, yaw_rad]).as_quat()

    return traj.QuadrotorState(
        inertial_from_body=traj.SE3(
            translation=traj.Vec3(c0=x_m, c1=y_m, c2=z_m),
            rotation=traj.SO3(
                quaternion=traj.Vec4(c0=quat[3], c1=quat[0], c2=quat[1], c3=quat[2])
            ),
        ),
        body_velocity=traj.Vec6(),
    )


def make_traj_pt(t_s, vel_mps, horizon_s):
    quarter_horizon_s = horizon_s / 4.0
    if t_s < quarter_horizon_s:
        return make_state(x_m=vel_mps * t_s, y_m=0.0, z_m=0.0, roll_rad=0.0)
    if t_s < 2.0 * quarter_horizon_s:
        return make_state(
            x_m=vel_mps * quarter_horizon_s,
            y_m=vel_mps * (t_s - quarter_horizon_s),
            z_m=10.0 / 3.0,
            roll_rad=1 * np.pi / 3.0,
        )
    if t_s < 3.0 * quarter_horizon_s:
        return make_state(
            x_m=vel_mps * (3.0 * quarter_horizon_s - t_s),
            y_m=vel_mps * quarter_horizon_s,
            z_m=20.0 / 3.0,
            roll_rad=2.0 * np.pi / 3.0,
        )
    return make_state(
        x_m=0.0,
        y_m=vel_mps * (4.0 * quarter_horizon_s - t_s),
        z_m=10.0,
        roll_rad=np.pi,
    )


def plot_temporal_trajectories(traj_dict):
    fig, ax = plt.subplots(7, 1, figsize=(9, 12), sharex=True)

    for label, traj in traj_dict.items():
        traj_array = extract_traj_array(traj)

        ax[0].plot(
            traj_array[:, IDX.time_s],
            traj_array[:, IDX.translation_x_m],
            label=label,
        )
        ax[1].plot(
            traj_array[:, IDX.time_s],
            traj_array[:, IDX.translation_y_m],
            label=label,
        )
        ax[2].plot(
            traj_array[:, IDX.time_s],
            traj_array[:, IDX.translation_z_m],
            label=label,
        )

        xyz_euler_rad = np.array(
            [
                R.from_quat(
                    [
                        x[IDX.quaternion_x],
                        x[IDX.quaternion_y],
                        x[IDX.quaternion_z],
                        x[IDX.quaternion_w],
                    ]
                ).as_euler("xyz")
                for x in traj_array
            ]
        )
        ax[3].plot(
            traj_array[:, IDX.time_s],
            np.unwrap(xyz_euler_rad[:, 0]),
            label=label,
        )
        ax[4].plot(
            traj_array[:, IDX.time_s],
            xyz_euler_rad[:, 1],
            label=label,
        )
        ax[5].plot(
            traj_array[:, IDX.time_s],
            xyz_euler_rad[:, 2],
            label=label,
        )

        ax[6].plot(
            traj_array[:, IDX.time_s],
            traj_array[:, IDX.control_0 : IDX.control_3 + 1],
            label=label,
        )
    ax[0].set_ylabel("x translation [m]")
    ax[1].set_ylabel("y translation [m]")
    ax[2].set_ylabel("z translation [m]")
    ax[3].set_ylabel("roll [rad]")
    ax[4].set_ylabel("pitch [rad]")
    ax[5].set_ylabel("yaw [rad]")
    ax[6].set_ylabel("control")

    [axis.legend() for axis in ax]

    fig.align_ylabels()
    ax[-1].set_xlabel("time [s]")


def plot_costs(costs):
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    ax.semilogy(costs)
    ax.set_xlabel("cost")
    ax.set_ylabel("iteration")


def animate_trajectories(traj_dict, plot_3d_key):
    fig, ax = plt.subplots(1, 1, figsize=(9, 9), subplot_kw={"projection": "3d"})
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    for label, traj in traj_dict.items():
        traj_array = extract_traj_array(traj)
        ax.plot3D(
            traj_array[:, IDX.translation_x_m],
            traj_array[:, IDX.translation_y_m],
            traj_array[:, IDX.translation_z_m],
            label=label,
        )

        # set equal aspect ratio
        scale = traj_array[:, IDX.translation_x_m : IDX.translation_z_m + 1].flatten()
        ax.auto_scale_xyz(scale, scale, scale)

    orig_quad_mesh = mesh.Mesh.from_file("quad_simple_scaled.stl")
    orig_quad_mesh.rotate([1.0, 0.0, 0.0], np.pi / 2.0)
    orig_quad_mesh.rotate([0.0, 0.0, 1.0], np.pi)
    collection = ax.add_collection3d(
        mplot3d.art3d.Poly3DCollection(orig_quad_mesh.vectors)
    )

    def anim_init():
        return (collection,)

    def anim_update(traj_pt):
        quaternion_scalar_first = traj_pt.state.inertial_from_body.rotation.quaternion
        quaternion_scalar_last = np.array(
            [
                quaternion_scalar_first.c1,
                quaternion_scalar_first.c2,
                quaternion_scalar_first.c3,
                quaternion_scalar_first.c0,
            ]
        )
        rot = Rotation.from_quat(quaternion_scalar_last)
        translation = np.array(
            [
                traj_pt.state.inertial_from_body.translation.c0,
                traj_pt.state.inertial_from_body.translation.c1,
                traj_pt.state.inertial_from_body.translation.c2,
            ]
        )

        transform = np.eye(4)
        transform[0:3, 0:3] = rot.as_matrix()
        transform[0:3, 3] = translation

        quad_mesh = deepcopy(orig_quad_mesh)
        quad_mesh.transform(transform)
        collection.set_verts(quad_mesh.vectors)
        return (collection,)

    ax.legend(bbox_to_anchor=(1.5, 0.5), loc="center right", ncol=2)
    fig.tight_layout()

    anim = animation.FuncAnimation(
        fig,
        anim_update,
        frames=traj_dict[plot_3d_key].points,
        init_func=anim_init,
        blit=False,
    )

    return anim


def main(show_plots: bool = True, plot_iters: bool = False, save_anim_path: str = None):
    dt_s = 0.1
    horizon_s = 4.0
    time_s = np.arange(0, horizon_s, dt_s)
    vel_mps = 10
    desired_traj = traj.QuadrotorTrajectory(
        points=[
            traj.QuadrotorTrajectoryPoint(
                time_s=t_s,
                state=make_traj_pt(t_s, vel_mps, horizon_s),
                control=traj.Vec4(),
            )
            for t_s in time_s
        ]
    )

    options = opts.ILQROptions(
        line_search_params=opts.LineSearchParams(
            step_update=0.5,
            desired_reduction_frac=0.5,
            max_iters=100,
        ),
        convergence_criteria=opts.ConvergenceCriteria(
            rtol=1e-12,
            atol=1e-12,
            max_iters=100,
        ),
        populate_debug=True,
    )

    mass_kg = 1.0
    inertia = np.eye(3)
    arm_length_m = 1.0
    torque_to_thrust_ratio_m = 0.0
    g_mpss = 9.81
    Q = np.diag(np.concatenate((100 * np.ones(6), 1 * np.ones(6))))
    R = np.eye(4)

    ilqr = QuadrotorILQR(
        mass_kg,
        inertia,
        arm_length_m,
        torque_to_thrust_ratio_m,
        g_mpss,
        Q,
        R,
        desired_traj,
        dt_s,
        options,
    )
    opt_traj, debug = ilqr.solve(desired_traj)

    traj_dict = {"desired": desired_traj, "optimized": opt_traj}
    if plot_iters:
        for i, iter_debug in enumerate(debug.iter_debugs):
            traj_dict[f"iter {i}"] = iter_debug.trajectory
    costs = [d.cost for d in debug.iter_debugs]

    if show_plots:
        plot_temporal_trajectories(traj_dict)
        plot_costs(costs)
        anim = animate_trajectories(traj_dict, plot_3d_key="optimized")
        plt.show()

        if save_anim_path:
            print(f"Saving animation to {save_anim_path}...", end=" ", flush=True)
            anim.save(save_anim_path, "ffmpeg", f"{1/dt_s}")
            print("Done!")


def parse_args(args):
    parser = argparse.ArgumentParser(
        description="Run the Quadrotor iLQR Trajectory Generator."
    )
    parser.add_argument(
        "--show_plots",
        action="store_true",
        help="Show the plots after generating the trajectory",
    )
    parser.add_argument(
        "--plot_iters",
        action="store_true",
        help="Plot the intermediate trajectories generated during optimzation.",
    )
    parser.add_argument(
        "--save_anim_path",
        type=str,
        default=None,
        help="The path to save the result animation. --show_plots must be specified"
        "for this option to have an affect. If the path is not provided, "
        "the animation will not be saved.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    main(args.show_plots, args.plot_iters, args.save_anim_path)
