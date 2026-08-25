from lxml import etree
from robot_model import *
import numpy as np
from scipy.spatial.transform import Rotation as R

def createDeattachableJoint(constraint:etree.Element, model_name:str)->etree.Element:
    """
    Create a detachable joint for a constraint.
    Args:
        constraint: The constraint to create a detachable joint for
        model_name: The name of the model
    Returns:
        The detachable joint
    """
    # add gazebo element
    gazebo = etree.Element("gazebo")
    # create detachable joint plugin element and set values
    deattachable_joint = etree.Element("plugin")
    deattachable_joint.set('name', 'gz::sim::systems::DetachableJoint')
    deattachable_joint.set('filename', 'libgz-sim-detachable-joint-system.so')
    # add parent link, child link and child model elements and set values
    parent_element = etree.Element("parent_link")
    parent_element.text = f'{constraint.get("name")}_parent'
    child_element = etree.Element("child_link")
    child_element.text = f'{constraint.get("name")}_child'
    child_model_element = etree.Element("child_model")
    child_model_element.text = model_name
    # append parent link, child link and child model elements to detachable joint
    deattachable_joint.append(parent_element)
    deattachable_joint.append(child_element)
    deattachable_joint.append(child_model_element)
    # append detachable joint to gazebo
    gazebo.append(deattachable_joint)
    return gazebo


def createLink(name:str)->etree.Element:
    """
    create a link with the given name
    Args:
        name: The name of the link
    Returns:
        The link element
    """
    link = etree.Element('link')
    link.set('name', name)
    return link

def createParentLink(name:str)->etree.Element:
    """
    create a parent link with the given name
    Args:
        name: The name of the parent link
    Returns:
        The parent link element
    """
    link = etree.Element('parent')
    link.set('link', name)
    return link

def createChildLink(name:str)->etree.Element:
    """
    Create a child link with the given name
    Args:
        name: The name of the child link
    Returns:
        The child link element
    """
    link = etree.Element('child')
    link.set('link', name)
    return link

def createOrigin(xyz:str, rpy:str)->etree.Element:
    """
    Create an origin with the given xyz and rpy
    Args:
        xyz: The xyz coordinates
        rpy: The rpy coordinates
    Returns:
        The origin element
    """
    origin = etree.Element('origin')
    origin.set('rpy', rpy)
    origin.set('xyz', xyz)
    return origin

def createAxis(xyz:str)->etree.Element:
    """
    Create an axis with the given xyz
    Args:
        xyz: The xyz coordinates
    Returns:
        The axis element
    """
    axis = etree.Element('axis')
    axis.set('xyz', xyz)
    return axis    

def createPreserveFixedJoints(links:tuple)->tuple:
    """
    Create preserve fixed joints to prevent them from being removed by gazebo optimization.
    Args:
        links: A tuple of links
    Returns:
        A tuple of preserve fixed joints
    """
    # create empty list for all preserved links
    preserve_links = []
    for link in links:
        # create a gazebo element and set reference to a link
        preserve_link = etree.Element('gazebo')
        preserve_link.set('reference', link)
        # create a preserve fixed joint element and set value to true
        preserve_fixed_joint = etree.Element('preserveFixedJoint')
        preserve_fixed_joint.text = 'true'
        # append preserve fixed joint element to gazebo element
        preserve_link.append(preserve_fixed_joint)
        # append gazebo element to preserve links
        preserve_links.append(preserve_link)
    return tuple(preserve_links)


def createFixedJoint(name:str, parent:str, child:str, xyz:str|None, rpy:str|None)->etree.Element:
    """
    Create a fixed joint with the given name, parent, child, xyz and rpy
    Args:
        name: The name of the joint
        parent: The parent link
        child: The child link
        xyz: The xyz coordinates
        rpy: The rpy coordinates
    Returns:
        The fixed joint element
    """
    # set origin if none is given
    if rpy is None:
        rpy = "0 0 0"
    if xyz is None:
        xyz = "0 0 0"
    # create joint and set values
    joint = etree.Element('joint')
    joint.set('name', name)
    joint.set('type', 'fixed')
    # append parent link, child link and origin elements to joint
    joint.append(createParentLink(parent))
    joint.append(createChildLink(child))
    joint.append(createOrigin(xyz, rpy))
    return joint


def createContinuousJoint(name:str, parent:str, child:str, xyz:str|None, rpy:str|None, axis:str|None)->etree.Element:
    """
    Create a continuous joint with the given name, parent, child, xyz, rpy and axis
    Args:
        name: The name of the joint
        parent: The parent link
        child: The child link
        xyz: The xyz coordinates
        rpy: The rpy coordinates
        axis: The axis of rotation
    Returns:
        The continuous joint element
    """
    # set origin and axis if none is given
    if rpy is None:
        rpy = "0 0 0"
    if xyz is None:
        xyz = "0 0 0"
    if axis is None:
        axis = "0 0 1"
    #create joint element and set values
    joint = etree.Element('joint')
    joint.set('name', name)
    joint.set('type', 'continuous')
    # append parent link, child link, origin and axis elements to joint
    joint.append(createParentLink(parent))
    joint.append(createChildLink(child))
    joint.append(createOrigin(xyz, rpy))
    joint.append(createAxis(axis))
    return joint


def createPrismaticJoint(name:str, parent:str, child:str, xyz:str|None, rpy:str|None, axis:str|None)->etree.Element:
    """
    Create a prismatic joint with the given name, parent, child, xyz, rpy and axis
    Args:
        name: The name of the joint
        parent: The parent link
        child: The child link
        xyz: The xyz coordinates
        rpy: The rpy coordinates
        axis: The axis of rotation
    Returns:
        The prismatic joint element
    """
    # set origin and axis if none is given
    if rpy is None:
        rpy = "0 0 0"
    if xyz is None:
        xyz = "0 0 0"
    if axis is None:
        axis = "0 0 1"
    #create joint element and set values
    joint = etree.Element('joint')
    joint.set('name', name)
    joint.set('type', 'prismatic')
    # append elements to joint
    joint.append(createParentLink(parent))
    joint.append(createChildLink(child))
    joint.append(createOrigin(xyz, rpy))
    joint.append(createAxis(axis))
    return joint


def createSphericalJoint(name:str, parent:str, child:str, xyz:str, rpy:str)->tuple:
    """
    Create a spherical joint with the given name, parent, child, xyz and rpy
    Args:
        name: The name of the joint
        parent: The parent link
        child: The child link
        xyz: The xyz coordinates
        rpy: The rpy coordinates
    Returns:
        A tuple of dummy links and joints
    """
    # create 2 dummy links
    dummy_1 = createLink(f'{name}_dummy_1')
    dummy_2 = createLink(f'{name}_dummy_2')
    # create 3 continuous joints
    joint1 = createContinuousJoint(f'{name}_x', parent, f'{name}_dummy_1', xyz, rpy, "1 0 0")
    joint2 = createContinuousJoint(f'{name}_y', f'{name}_dummy_1', f'{name}_dummy_2', None, None, "0 1 0")
    joint3 = createContinuousJoint(f'{name}_z', f'{name}_dummy_2', child, None, None, "0 0 1")
    # return dummy links and joints
    return dummy_1, dummy_2, joint1, joint2, joint3


def createFixedConstraint(constraint:etree.Element, model_name:str, violation:dict=None)->tuple:
    """
    Create a fixed constraint with the given constraint, model_name and violation
    Args:
        constraint: The constraint element
        model_name: The name of the model
        violation: The violation dictionary
    Returns:
        A tuple of dummy links and joints
    """
    # create parent and child links
    parent_link = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')

    # create fixed joints
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
    # create deattachable joint
    detachable_joint = createDeattachableJoint(constraint, model_name)
    # return parent joint, parent link, child joint, child link and detachable joint
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createSphericalConstraint(constraint:etree.Element, model_name:str, violation:dict)->tuple:
    """
    Create a spherical constraint with the given constraint, model_name and violation
    Args:
        constraint: The constraint element
        model_name: The name of the model
        violation: The violation dictionary
    Returns:
        A tuple of dummy links and joints
    """
    # create parent and child links
    parent_link: etree.Element = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')

    # create violation rotation and translation
    if violation is None:
        violation_rotation = R.from_matrix(np.eye(3))
        violation_translation = np.zeros(3)
    else:
        violation_rotation = violation[0]
        violation_translation = violation[1]
    # clac rotation and translation from violation
    child_rotation = np.array([float(x) for x in constraint.find('child_origin').get('rpy').split()])
    child_rotation = R.from_euler('xyz', child_rotation)
    child_rotation = (child_rotation * violation_rotation.inv()).inv() 
    child_translation = np.array([float(x) for x in constraint.find('child_origin').get('xyz').split()])
    child_translation = violation_translation + child_translation
    child_rotation = ' '.join(map(str, child_rotation.as_euler('xyz')))
    child_translation = ' '.join(map(str, child_translation))
    
    # create parent joint
    parent_joint = createSphericalJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'))
    # create child joint
    child_joint = createFixedJoint(f'{constraint.get("name")}_child_joint',
                                        constraint.find('child').get('link'),
                                        child_link.get('name'),
                                        child_translation,
                                        child_rotation)
    # create detachable joint
    detachable_joint = createDeattachableJoint(constraint, model_name)
    # create preserve fixed joints
    preserve_parent = createPreserveFixedJoints((child_joint.get('name'),))
    # return parent joint, parent link, child joint, child link and detachable joint
    return *parent_joint, parent_link, child_joint, child_link, detachable_joint, *preserve_parent


def createRevoluteConstraint(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    """
    Create a revolute constraint with the given constraint, model_name and violation
    Args:
        constraint: The constraint element
        model_name: The name of the model
        violation: The violation dictionary
    Returns:
        A tuple of dummy links and joints
    """
    # create parent and child links
    parent_link = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')
    # create parent joint
    parent_joint = createContinuousJoint(f'{constraint.get("name")}_parent_joint',
                                    constraint.find('parent').get('link'),
                                    parent_link.get('name'),
                                    constraint.find('parent_origin').get('xyz'),
                                    constraint.find('parent_origin').get('rpy'),
                                    constraint.find('parent_axis').get('xyz'))
    # create child joint
    child_joint = createFixedJoint(f'{constraint.get("name")}_child_joint',
                                    constraint.find('child').get('link'),
                                    child_link.get('name'),
                                    constraint.find('child_origin').get('xyz'),
                                    constraint.find('child_origin').get('rpy'))
    # create detachable joint
    detachable_joint = createDeattachableJoint(constraint, model_name)
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createPrismaticConstraint(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    """
    Create a prismatic constraint with the given constraint, model_name and violation
    Args:
        constraint: The constraint element
        model_name: The name of the model
        violation: The violation dictionary
    Returns:
        A tuple of dummy links and joints
    """
    # create parent and child links
    parent_link = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')
    #create parent and child joints
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
    # create detachable joint
    detachable_joint = createDeattachableJoint(constraint, model_name)
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createUniversalConstraint(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    """
    Create a universal constraint with the given constraint, model_name and violation
    Args:
        constraint: The constraint element
        model_name: The name of the model
        violation: The violation dictionary
    Returns:
        A tuple of dummy links and joints
    """
    # create parent and child links
    parent_link: etree.Element = createLink(f'{constraint.get("name")}_parent')
    child_link = createLink(f'{constraint.get("name")}_child')
    #create parent and child joints
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
    # create detachable joint
    detachable_joint = createDeattachableJoint(constraint, model_name)
    # return parent joint, parent link, child joint, child link and detachable joint
    return parent_joint, parent_link, child_joint, child_link, detachable_joint


def createDetachableJointElements(constraint:etree.Element, model_name:str, violation:tuple=None)->tuple:
    """
    Create detachable joint elements with the given constraint, model_name and violation
    Args:
        constraint: The constraint element
        model_name: The name of the model
        violation: The violation dictionary
    Returns:
        A tuple of dummy links and joints
    """
    # choose the type of constraint and create elements
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
    """
    Add detachable joints to the urdf tree
    Args:
        tree: The urdf tree
        violations: The violation dictionary
    Returns:
        The urdf tree with detachable joints added
    """
    root = tree.getroot()
   # Add detachable joints for each constraint
    for constraint in root.findall('constraint'):
        violation = violations[constraint.get('name')] if constraint.get('name') in violations else None
        for element in createDetachableJointElements(constraint, root.get('name'), violation):
            root.append(element)
    return tree

if __name__ =="__main__":
    # import here to prevent circular import error
    from close_loop import close_loop
    path = "/home/robot/ws/src/constraint_to_detachable_joint/example.urdf"
    tree = etree.parse(path)
    robot = RobotModel(path)
    close_loop_tree, violations = close_loop(tree,robot)
    close_loop_tree = addDetachableJoints(close_loop_tree, violations)
    #print(etree.tostring(close_loop_tree,pretty_print=True))
    