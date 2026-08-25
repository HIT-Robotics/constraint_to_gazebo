from robot_model import *
from lxml import etree
import numpy as np
from scipy.spatial.transform import Rotation as R


# ToDo: add helper functions to make the whole function more readable
def close_loop(tree:etree.ElementTree, robot:RobotModel)->etree.ElementTree:
    """
    close the loop of a closed loop robot defined with a tree structure and an urdf using a numerical solver to calculate a valid state
    Args:
        tree: The urdf tree
        robot: The robot model
    Returns:
        The urdf tree with the loop closed
        violations: A dictionary of constraint violations
    """
    #import robot_model
    # parse tree
    root = tree.getroot()
    # calculate a valid zero configuration for the constrained robot
    zero_state, success = calcValidMotionState(robot, tol=1e-11, num_iterations=100)
    # check if the solver returned a valid solution
    if not success:
        raise ValueError("Failed to calculate valid motion state")
    
    # update the joint values in the tree
    for joint, valid_value in zip(root.findall('joint'), zero_state):
        # change nothing for fixed joints
        if joint.get('type') == 'fixed':
            continue
        # revolute joints
        if joint.get('type') == 'revolute':
            # check for origin and set default if not exists
            if joint.find('origin') is None:
                origin = etree.Element('origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
                joint.append(origin)
            # get xyz and rpy from origin
            xyz = joint.find('origin').get('xyz')
            rpy = joint.find('origin').get('rpy')
            # check xyz and rpy values and set default if not exists
            if xyz is not None:
                origin_xyz = np.array(xyz.split(), dtype=float)
            if rpy is not None:
                origin_rpy = np.array(rpy.split(), dtype=float)
            # get axis from the joint
            axis = np.array(joint.find('axis').get('xyz').split(), dtype=float)
            # calculate zero state for the joint and set it in the tree
            rotation_joint = R.from_rotvec(valid_value * axis)
            rotation_origin = R.from_euler('xyz', origin_rpy)
            valid_rotation = rotation_origin * rotation_joint
            valid_rpy = ' '.join(map(str, valid_rotation.as_euler('xyz')))
            joint.find('origin').set('rpy', valid_rpy)
            # check for limits and update them
            if joint.find('limit') is not None:
                joint.find('limit').set('lower', str(float(joint.find('limit').get('lower'))-valid_value))
                joint.find('limit').set('upper', str(float(joint.find('limit').get('upper'))-valid_value))
        # prismatic joints
        if joint.get('type') == 'prismatic':
            # get origin or create if none exists
            origin = joint.find('origin')
            if joint.find('origin') is None:
                origin = etree.Element('origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
                joint.append(origin)
            # get xyz and rpy from origin
            xyz = joint.find('origin').get('xyz')
            rpy = joint.find('origin').get('rpy')
            # check xyz and rpy values and set default if not exists
            if xyz is not None:
                origin_xyz = np.array(xyz.split(), dtype=float)
            if rpy is not None:
                origin_rpy = np.array(rpy.split(), dtype=float)
            # get axis from the joint
            axis = np.array(joint.find('axis').get('xyz').split(), dtype=float)
            # calculate zero state for the joint and set it in the tree
            R_origin = R.from_euler('xyz', origin_rpy)
            translation_joint = R_origin.apply(valid_value*axis)
            translation_origin = np.array(origin_xyz)
            valid_translation = translation_origin + translation_joint
            valid_xyz = ' '.join(map(str, valid_translation))
            joint.find('origin').set('xyz', valid_xyz)
            # update joint limits
            if joint.find('limit') is not None:
                joint.find('limit').set('lower', str(float(joint.find('limit').get('lower'))-valid_value))
                joint.find('limit').set('upper', str(float(joint.find('limit').get('upper'))-valid_value))
    return tree , getConstraintTransforms(robot, zero_state, tree)


def getConstraintTransforms(robot:RobotModel, state:np.ndarray, tree:etree.ElementTree)->dict[str, list[R, np.ndarray]]:
    """
    Get the constraint transforms from the robot model.
    Args:
        robot: The robot model
        state: The robot state
        tree: The urdf tree
    Returns:
        A dictionary of constraint transforms
    """
    # update the robot state
    robot.updateData(state) 
    # preset violations dictionary
    violations = {}
    root = tree.getroot()
    # loop through all constraints
    for constraint in root.findall('constraint'):
        # get data from the constraint for later calculations
        name = constraint.get('name')
        parent = f'{name}_parent_frame'
        child = f'{name}_child_frame'
    
        # get poses of the parent and child frames
        parent_pose = robot.getFrameTransform(parent)
        child_pose = robot.getFrameTransform(child)
        # get the rotation matrices and translation vectors
        parent_rotation = R.from_matrix(np.array(parent_pose[0:3, 0:3]))
        child_rotation = R.from_matrix(np.array(child_pose[0:3, 0:3]))
        parent_translation = np.array(parent_pose[0:3, 3])
        child_translation = np.array(child_pose[0:3, 3])
        # calculate violations
        rotation_violation = parent_rotation.inv() * child_rotation
        translation_violation = parent_translation - child_translation
        # add to dictionary
        violations[name] = [rotation_violation, translation_violation]
    return violations
    


if __name__ == "__main__":
    """
    Example usage of close_loop function
    """
    tree = etree.parse("/home/robot/test_ws/install/descriptions/share/descriptions/urdf/A_00036-02.ardf")
    robot_model = RobotModel("/home/robot/test_ws/install/descriptions/share/descriptions/urdf/A_00036-02.ardf")
    tree, violations = close_loop(tree, robot_model)