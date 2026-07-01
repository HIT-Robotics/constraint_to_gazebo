from lxml import etree
from robot_model import *
import numpy as np
from scipy.spatial.transform import Rotation as R

if __name__ != "__main__":
    from constraint_to_gazebo.close_loop import close_loop

def createDeattachableJoint(constraint:etree.Element, model_name:str)->etree.Element:
    gazebo = etree.Element("gazebo")
    deattachable_joint = etree.Element("plugin")
    deattachable_joint.set('name', 'gz::sim::systems::DetachableJoint')
    deattachable_joint.set('filename', 'libgz-sim-detachable-joint-system.so')
    parent_element = etree.Element("parent_link")
    parent_element.text = f'{constraint.get("name")}_parent'
    child_element = etree.Element("child_link")
    child_element.text = f'{constraint.get("name")}_child'
    child_model_element = etree.Element("child_model")
    child_model_element.text = model_name
    deattachable_joint.append(parent_element)
    deattachable_joint.append(child_element)
    deattachable_joint.append(child_model_element)
    gazebo.append(deattachable_joint)
    return gazebo


def createLink(name:str)->etree.Element:
    link = etree.Element('link')
    link.set('name', name)
    return link

def createParentLink(name:str)->etree.Element:
    link = etree.Element('parent')
    link.set('link', name)
    return link

def createChildLink(name:str)->etree.Element:
    link = etree.Element('child')
    link.set('link', name)
    return link

def createOrigin(xyz:str, rpy:str)->etree.Element:
    origin = etree.Element('origin')
    origin.set('rpy', rpy)
    origin.set('xyz', xyz)
    return origin

def createAxis(xyz:str)->etree.Element:
    axis = etree.Element('axis')
    axis.set('xyz', xyz)
    return axis    

def createPreserveFixedJoints(links:tuple)->tuple:
    preserve_links = []
    for link in links:
        preserve_link = etree.Element('gazebo')
        preserve_link.set('reference', link)
        preserve_fixed_joint = etree.Element('preserveFixedJoint')
        preserve_fixed_joint.text = 'true'
        preserve_link.append(preserve_fixed_joint)
        preserve_links.append(preserve_link)
    return tuple(preserve_links)


def createFixedJoint(name:str, parent:str, child:str, xyz:str, rpy:str)->etree.Element:
    if rpy is None:
        rpy = "0 0 0"
    if xyz is None:
        xyz = "0 0 0"
    joint = etree.Element('joint')
    joint.set('name', name)
    joint.set('type', 'fixed')
    joint.append(createParentLink(parent))
    joint.append(createChildLink(child))
    joint.append(createOrigin(xyz, rpy))
    return joint


def createContinuousJoint(name:str, parent:str, child:str, xyz:str|None, rpy:str|None, axis:str|None)->etree.Element:
    if rpy is None:
        rpy = "0 0 0"
    if xyz is None:
        xyz = "0 0 0"
    if axis is None:
        axis = "0 0 1"
    joint = etree.Element('joint')
    joint.set('name', name)
    joint.set('type', 'continuous')
    joint.append(createParentLink(parent))
    joint.append(createChildLink(child))
    joint.append(createOrigin(xyz, rpy))
    joint.append(createAxis(axis))
    return joint


def createPrismaticJoint(name:str, parent:str, child:str, xyz:str|None, rpy:str|None, axis:str|None)->etree.Element:
    if rpy is None:
        rpy = "0 0 0"
    if xyz is None:
        xyz = "0 0 0"
    if axis is None:
        axis = "0 0 1"
    joint = etree.Element('joint')
    joint.set('name', name)
    joint.set('type', 'prismatic')
    joint.append(createParentLink(parent))
    joint.append(createChildLink(child))
    joint.append(createOrigin(rpy, xyz))
    joint.append(createAxis(axis))
    return joint


def createSphericalJoint(name:str, parent:str, child:str, xyz:str, rpy:str)->tuple:
    dummy_1 = createLink(f'{name}_dummy_1')
    dummy_2 = createLink(f'{name}_dummy_2')
    joint1 = createContinuousJoint(f'{name}_x', parent, f'{name}_dummy_1', xyz, rpy, "1 0 0")
    joint2 = createContinuousJoint(f'{name}_y', f'{name}_dummy_1', f'{name}_dummy_2', None, None, "0 1 0")
    joint3 = createContinuousJoint(f'{name}_z', f'{name}_dummy_2', child, None, None, "0 0 1")
    return dummy_1, dummy_2, joint1, joint2, joint3


def createFixedConstraint(constraint:etree.Element, model_name:str, violation:dict=None)->tuple:
    parent_link = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')

    parent_joint = createFixedJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'))
    child_joint = createFixedJoint(f'{constraint.get("name")}_child_joint',
                                    constraint.find('child').get('link'),
                                    child_link.get('name'),
                                    constraint.find('child_origin').get('xyz'),
                                    constraint.find('child_origin').get('rpy'))
    detachable_joint = createDeattachableJoint(constraint, model_name)
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createSphericalConstraint(constraint:etree.Element, model_name:str, violation:dict)->tuple:
    parent_link: etree.Element = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')

    if violation is None:
        violation_rotation = R.from_matrix(np.eye(3))
        violation_translation = np.zeros(3)
    else:
        violation_rotation = violation[0]
        violation_translation = violation[1]
    child_rotation = np.array([float(x) for x in constraint.find('child_origin').get('rpy').split()])
    child_rotation = R.from_euler('xyz', child_rotation)
    child_rotation = (child_rotation * violation_rotation.inv()).inv() 
    child_translation = np.array([float(x) for x in constraint.find('child_origin').get('xyz').split()])
    child_translation = violation_translation + child_translation
    child_rotation = ' '.join(map(str, child_rotation.as_euler('xyz')))
    child_translation = ' '.join(map(str, child_translation))

    parent_joint = createSphericalJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'))
    child_joint = createFixedJoint(f'{constraint.get("name")}_child_joint',
                                        constraint.find('child').get('link'),
                                        child_link.get('name'),
                                        child_translation,
                                        child_rotation)
    detachable_joint = createDeattachableJoint(constraint, model_name)
    preserve_parent = createPreserveFixedJoints((child_joint.get('name'),))
    return *parent_joint, parent_link, child_joint, child_link, detachable_joint, *preserve_parent


def createRevoluteConstraint(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    parent_link = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')
    parent_joint = createContinuousJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'),
                                    constraint.find('parent_axis').get('xyz'))
    child_joint = createFixedJoint(f'{constraint.get("name")}_child_joint',
                                    constraint.find('child').get('link'),
                                    child_link.get('name'),
                                    constraint.find('child_origin').get('xyz'),
                                    constraint.find('child_origin').get('rpy'))
    detachable_joint = createDeattachableJoint(constraint, model_name)
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createPrismaticConstraint(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    parent_link = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')
    parent_joint = createPrismaticJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'),
                                    constraint.find('parent_axis').get('xyz'))
    child_joint = createFixedJoint(f'{constraint.get("name")}_child_joint',
                                    constraint.find('child').get('link'),
                                    child_link.get('name'),
                                    constraint.find('child_origin').get('xyz'),
                                    constraint.find('child_origin').get('rpy'))
    detachable_joint = createDeattachableJoint(constraint, model_name)
    return parent_joint, parent_link, child_joint, child_link, detachable_joint

def createUniversalConstraint(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    parent_link: etree.Element = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')
    parent_joint = createContinuousJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'),
                                    constraint.find('parent_axis').get('xyz'))
    child_joint = createContinuousJoint(f'{constraint.get("name")}_child_joint',
                                        constraint.find('child').get('link'),
                                        child_link.get('name'),
                                        constraint.find('child_origin').get('xyz'),
                                        constraint.find('child_origin').get('rpy'),
                                        constraint.find('child_axis').get('xyz'))
    detachable_joint = createDeattachableJoint(constraint, model_name)
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createDetachableJointElements(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    match constraint.get('type'):
        case 'fixed':
            return createFixedConstraint(constraint, model_name)
        case 'spherical' | 'spherical_joint':
            return createSphericalConstraint(constraint, model_name, violation)
        case 'universal' | 'universal_joint':
            return createUniversalConstraint(constraint, model_name, violation)
        case 'revolute' | 'revolute_joint':
            return createRevoluteConstraint(constraint, model_name, violation)
        case 'prismatic' | 'prismatic_joint':
            return createPrismaticConstraint(constraint, model_name, violation)
    return tuple()
    
def addDetachableJoints(tree:etree.ElementTree, violations:dict={})->etree.ElementTree:
    root = tree.getroot()
    for constraint in root.findall('constraint'):
        violation = violations[constraint.get('name')] if constraint.get('name') in violations else None
        for element in createDetachableJointElements(constraint, root.get('name'), violation):
            root.append(element)
    return tree

if __name__ =="__main__":
    from close_loop import close_loop
    path = "/home/robot/ws/src/constraint_to_detachable_joint/example.urdf"
    tree = etree.parse(path)
    robot = RobotModel(path)
    close_loop_tree, violations = close_loop(tree,robot)
    close_loop_tree = addDetachableJoints(close_loop_tree, violations)
    #print(etree.tostring(close_loop_tree,pretty_print=True))
    