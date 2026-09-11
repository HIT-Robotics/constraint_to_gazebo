from lxml import etree

def add_ros2_control(urdf_tree, active_joints):
    """Dynamically adds <ros2_control> and gz_ros2_control plugin to the URDF tree."""
    # Add <ros2_control> element for active joints only

    ros2_control_elem = etree.SubElement(
        urdf_tree.getroot(),
        "ros2_control",
        name="GazeboSimSystem",
        type="system"
    )

    hardware_elem = etree.SubElement(ros2_control_elem, "hardware")
    plugin_hw = etree.SubElement(hardware_elem, "plugin")
    plugin_hw.text = "gz_ros2_control/GazeboSimSystem"

    # Add hardware interfaces for active driving joints (skip loop-closure passive joints)
    for joint_name in active_joints:
        joint_elem = etree.SubElement(ros2_control_elem, "joint", name=joint_name)
        
        # Command interfaces (e.g., position, effort)
        cmd_interface = etree.SubElement(joint_elem, "command_interface", name="position")
        
        # State interfaces
        etree.SubElement(joint_elem, "state_interface", name="position")
        etree.SubElement(joint_elem, "state_interface", name="velocity")

    return urdf_tree

def add_gz_ros2_control(urdf_tree, temp_yaml_path):
    """Adds gz_ros2_control system plugin pointing to the generated temp YAML."""
    gazebo_elem = etree.SubElement(urdf_tree.getroot(), "gazebo")
    
    plugin_elem = etree.SubElement(
        gazebo_elem,
        "plugin",
        filename="gz_ros2_control-system",
        name="gz_ros2_control::GazeboSimROS2ControlPlugin"
    )

    # Point directly to the generated temp YAML
    param_elem = etree.SubElement(plugin_elem, "parameters")
    param_elem.text = temp_yaml_path

    return urdf_tree

def add_gz_joint_state_publisher(urdf_tree, active_joints=None, topic_name='joint_states'):
    """
    Appends the gz-sim-joint-state-publisher-system plugin tag to the URDF root.
    If active_joints is provided, only those joints will be published.
    """
    root = urdf_tree.getroot()
    
    gazebo_elem = etree.SubElement(root, 'gazebo')
    plugin_elem = etree.SubElement(gazebo_elem, 'plugin', {
        'filename': 'gz-sim-joint-state-publisher-system',
        'name': 'gz::sim::systems::JointStatePublisher'
    })
    
    # Set topic name
    topic_elem = etree.SubElement(plugin_elem, 'topic')
    topic_elem.text = topic_name
    
    # Add joint_name elements only if active_joints is provided
    if active_joints is not None:
        for joint in active_joints:
            j_elem = etree.SubElement(plugin_elem, 'joint_name')
            j_elem.text = str(joint)
            
    return urdf_tree
