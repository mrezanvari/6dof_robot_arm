import os
from rclpy.utilities import remove_ros_args


def parse_custom_args(args) -> dict:
    """Converts all non-ros args into a dict for custom user input"""
    args_raw = remove_ros_args(args)
    args_list = args_raw[1:]
    args_dict = {}

    current_key = ""
    for item in args_list:
        if item.startswith("--"):
            current_key = item.replace("--", "")
        else:
            args_dict[current_key] = int(item) if item.isnumeric() else item

    return args_dict


def set_ros_domain_id(domain_id: int = 0):
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)


def init_ros_domain_from_args(args, domain_id_arg_key="domain-id"):
    args_dict = parse_custom_args(args)
    print("ROS_DOMAIN_ID set to: ", end="")
    if "domain-id" in list(args_dict.keys()):
        os.environ["ROS_DOMAIN_ID"] = str(args_dict[domain_id_arg_key])
        print(args_dict["domain-id"])
    else:
        print("DEFAULT:0 -> domain-id argument not found")
