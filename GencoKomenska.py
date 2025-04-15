import pybullet as p
import pybullet_data
import numpy as np
import time
import random

# Generate a random target position (upper workspace, farther from base)
def generate_random_pose():
    x = random.uniform(0.7, 1.0)       # forward
    y = random.uniform(-0.3, 0.3)      # side
    z = random.uniform(0.6, 0.9)       # upper space
    return [x, y, z]

# Solve inverse kinematics using PyBullet's built-in function
def solve_inverse_kinematics(robot_id, target_pos, target_ori, ee_index,
                             lower_limits, upper_limits, joint_ranges, rest_poses):
    joint_angles = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ee_index,
        targetPosition=target_pos,
        targetOrientation=target_ori,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses,
        maxNumIterations=100,
        residualThreshold=1e-4
    )
    return joint_angles

if __name__ == "__main__":
    # Connect to simulation
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load ground and robot
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        basePosition=[0, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]),
        useFixedBase=True
    )

    # Robot setup
    num_joints = p.getNumJoints(robot_id)
    end_effector_index = 6
    print("Number of joints:", num_joints)

    for joint in range(num_joints):
        p.resetJointState(robot_id, joint, 0)

    # Define joint constraints and rest pose
    lower_limits = [-1.0, -1.0] + [-2.9] * (num_joints - 2)
    upper_limits = [1.0, 1.0] + [2.9] * (num_joints - 2)
    joint_ranges = [2.0, 2.0] + [5.8] * (num_joints - 2)
    rest_poses = [0, -0.5, 0, 1.0, 0, 0.5, 0]

    last_pos = None  # for drawing trajectory

    # Move to 15 random target positions
    for i in range(15):
        target_position = generate_random_pose()
        target_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])  # flip end-effector

        # Calculate joint angles using IK
        joint_angles = solve_inverse_kinematics(
            robot_id,
            target_position,
            target_orientation,
            end_effector_index,
            lower_limits,
            upper_limits,
            joint_ranges,
            rest_poses
        )

        # Smooth motion simulation
        for step in range(240):
            for j in range(num_joints):
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_angles[j],
                    force=500
                )
            p.stepSimulation()
            time.sleep(1 / 240)

            # Track and draw trajectory
            ee_state = p.getLinkState(robot_id, end_effector_index)
            ee_pos = ee_state[0]

            if last_pos is not None:
                p.addUserDebugLine(last_pos, ee_pos, [0.1, 0.8, 0.3], lineWidth=2, lifeTime=0)

            last_pos = ee_pos

        time.sleep(0.5)

    input("Simulation finished. Press ENTER to exit.")
    p.disconnect()
