import robot_upstart

j = robot_upstart.Job()
j.add(package="user_package",filename="launch/robot_distributed.launch")
j.install()