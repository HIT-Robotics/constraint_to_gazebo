from lxml import etree

def addWorldLink(tree, base_link = 'base_link', offset = [0, 0, 0], rpy = [0, 0, 0]):
    root = tree.getroot()
    world_link = etree.Element('link', name='world')
    root.append(world_link)
    joint = etree.Element('joint', name='world_joint')
    joint.set('type', 'fixed')
    joint.append(etree.Element('parent', link='world'))
    joint.append(etree.Element('child', link=base_link))
    origin = etree.Element('origin')
    origin.set('xyz', f'{offset[0]} {offset[1]} {offset[2]}')
    origin.set('rpy', f'{rpy[0]} {rpy[1]} {rpy[2]}')
    joint.append(origin)
    root.append(joint)
    return tree