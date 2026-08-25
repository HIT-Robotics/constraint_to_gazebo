from lxml import etree

def addWorldLink(tree, base_link = 'base_link', offset = [0, 0, 0], rpy = [0, 0, 0]):
    """
    Add a world link to the urdf tree.
    Args:
        tree: The urdf tree
        base_link: The base link of the robot
        offset: The offset of the world link from the base link
        rpy: The rotation of the world link from the base link
    Returns:
        The urdf tree with the world link added
    """
    # get the root element
    root = tree.getroot()
    # create the world link
    world_link = etree.Element('link', name='world')
    # append the world link to the root
    root.append(world_link)
    # create the world joint
    joint = etree.Element('joint', name='world_joint')
    joint.set('type', 'fixed')
    # append the parent and child links to the joint
    joint.append(etree.Element('parent', link='world'))
    joint.append(etree.Element('child', link=base_link))
    # create the origin
    origin = etree.Element('origin')
    origin.set('xyz', f'{offset[0]} {offset[1]} {offset[2]}')
    origin.set('rpy', f'{rpy[0]} {rpy[1]} {rpy[2]}')
    # append the origin to the joint
    joint.append(origin)
    # append the joint to the root
    root.append(joint)
    # return the urdf tree
    return tree