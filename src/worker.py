from .DobotExpress import DobotExpress


def worker_main(port, cmd_q, res_q):

    robot = DobotExpress(port)

    # Initialize defaults if needed

    while True:
        name, args = cmd_q.get()

        if name == "__EXIT__":
            break

        try:
            result = getattr(robot, name)(*args)
            res_q.put(result)
        except Exception as e:
            res_q.put(e)
