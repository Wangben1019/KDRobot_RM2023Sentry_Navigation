import robot_upstart

j = robot_upstart.Job()
j.add(package="user_package",filename="launch/Start_Robot.launch")
j.uninstall()