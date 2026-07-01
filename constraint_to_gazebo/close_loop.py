from robot_model import *
from lxml import etree
import numpy as np
from scipy.spatial.transform import Rotation as R

def close_loop(tree:etree.ElementTree, robot:RobotModel)->etree.ElementTree:
    import robot_model
    root = tree.getroot()
    zero_state, success = calcValidMotionState(robot, tol=1e-11, num_iterations=100)
    if not success:
        raise ValueError("Failed to calculate valid motion state")

    for joint, valid_value in zip(root.findall('joint'), zero_state):
        if joint.get('type') == 'fixed':
            continue
        if joint.get('type') == 'revolute':
            if joint.find('origin') is None:
                origin = etree.Element('origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
                joint.append(origin)
            xyz = joint.find('origin').get('xyz')
            rpy = joint.find('origin').get('rpy')
            if xyz is not None:
                origin_xyz = np.array(xyz.split(), dtype=float)
            if rpy is not None:
                origin_rpy = np.array(rpy.split(), dtype=float)
            axis = np.array(joint.find('axis').get('xyz').split(), dtype=float)
            rotation_joint = R.from_rotvec(valid_value*axis)
            rotation_origin = R.from_euler('xyz', origin_rpy)
            valid_rotation = rotation_origin * rotation_joint
            valid_rpy = ' '.join(map(str, valid_rotation.as_euler('xyz')))
            joint.find('origin').set('rpy', valid_rpy)
            if joint.find('limit') is not None:
                joint.find('limit').set('lower', str(float(joint.find('limit').get('lower'))-valid_value))
                joint.find('limit').set('upper', str(float(joint.find('limit').get('upper'))-valid_value))
        if joint.get('type') == 'prismatic':
            origin = joint.find('origin')
            if joint.find('origin') is None:
                origin = etree.Element('origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
                joint.append(origin)
            xyz = joint.find('origin').get('xyz')
            rpy = joint.find('origin').get('rpy')
            if xyz is not None:
                origin_xyz = np.array(xyz.split(), dtype=float)
            if rpy is not None:
                origin_rpy = np.array(rpy.split(), dtype=float)
            axis = np.array(joint.find('axis').get('xyz').split(), dtype=float)
            R_origin = R.from_euler('xyz', origin_rpy)
            translation_joint = R_origin.apply(valid_value*axis)
            translation_origin = np.array(origin_xyz)
            valid_translation = translation_origin + translation_joint
            valid_xyz = ' '.join(map(str, valid_translation))
            joint.find('origin').set('xyz', valid_xyz)
            if joint.find('limit') is not None:
                joint.find('limit').set('lower', str(float(joint.find('limit').get('lower'))-valid_value))
                joint.find('limit').set('upper', str(float(joint.find('limit').get('upper'))-valid_value))
    return tree , getConstraintTransforms(robot, zero_state, tree)


def getConstraintTransforms(robot:RobotModel, state:np.ndarray, tree:etree.ElementTree)->dict[str, list[R, np.ndarray]]:
    robot.updateData(state)  
    violations = {}
    root = tree.getroot()
    for constraint in root.findall('constraint'):
        name = constraint.get('name')
        parent = f'{name}_parent_frame'
        child = f'{name}_child_frame'
        
        parent_pose = robot.getFrameTransform(parent)
        child_pose = robot.getFrameTransform(child)
        parent_rotation = R.from_matrix(np.array(parent_pose[0:3, 0:3]))
        child_rotation = R.from_matrix(np.array(child_pose[0:3, 0:3]))
        parent_translation = np.array(parent_pose[0:3, 3])
        child_translation = np.array(child_pose[0:3, 3])
        
        rotation_violation = parent_rotation.inv() * child_rotation
        translation_violation = parent_translation - child_translation

        violations[name] = [rotation_violation, translation_violation]
    return violations
    


if __name__ == "__main__":
    tree = etree.parse("/home/robot/test_ws/install/descriptions/share/descriptions/urdf/A_00036-02.ardf")
    robot_model = RobotModel("/home/robot/test_ws/install/descriptions/share/descriptions/urdf/A_00036-02.ardf")
    tree, violations = close_loop(tree, robot_model)