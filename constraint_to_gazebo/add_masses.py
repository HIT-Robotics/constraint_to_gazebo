from lxml import etree


def createMass() -> etree.Element:
    """Create a <mass> element with a reasonable non‑zero value.
    The value 0.1 kg works well for dummy links and satisfies the
    positive‑mass requirement of the URDF parser.
    """
    mass = etree.Element('mass')
    mass.set('value', '0.001')
    return mass


def createInertia() -> etree.Element:
    """Create a positive‑definite <inertia> element.
    Diagonal entries are set to 0.01 kg·m² and off‑diagonal entries are 0.0.
    """
    inertia = etree.Element('inertia')
    inertia.set('ixx', '1e-6')
    inertia.set('iyy', '1e-6')
    inertia.set('izz', '1e-6')
    inertia.set('ixy', '0.0')
    inertia.set('ixz', '0.0')
    inertia.set('iyz', '0.0')
    return inertia


def createInertial() -> etree.Element:
    """Combine <mass> and <inertia> under an <inertial> tag."""
    inertial = etree.Element('inertial')
    inertial.append(createMass())
    inertial.append(createInertia())
    return inertial

def addMassesToDummyLinks(tree:etree.ElementTree)->etree.ElementTree:
    """
    Add masses to dummy links in the urdf tree.
    Args:
        tree: The urdf tree
    Returns:
        The urdf tree with masses added to dummy links
    """
    root = tree.getroot()
    for link in root.findall('link'):
        if link.find('inertial') is None:
            link.append(createInertial())
    return tree