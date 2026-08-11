def main() -> None:
    # Importing this console module has already imported the history package and
    # registered every config class used by its YAML files.  Running LeRobot's
    # global third-party discovery again tries to import the current Piper
    # package as well; both packages intentionally expose the same uf::* config
    # names, so the second registration produces a harmless duplicate-plugin
    # error.  Skip only that redundant discovery in the isolated history CLI.
    from lerobot_robot_ufactory.scripts import uf_robot_teleop as parent_teleop

    parent_teleop.register_third_party_plugins = lambda: None
    parent_teleop.main()


if __name__ == "__main__":
    main()

