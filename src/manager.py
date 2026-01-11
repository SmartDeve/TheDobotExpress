# manager.py
from multiprocessing import Process, Queue
from .worker import worker_main
from .proxy import DobotProxy


class DobotManager:
    def __init__(self):
        self.processes = []

    def create(self, port):
        cmd_q = Queue()
        res_q = Queue()

        p = Process(target=worker_main, args=(port, cmd_q, res_q))
        p.start()

        self.processes.append(p)
        return DobotProxy(cmd_q, res_q)

    def shutdown(self):
        for p in self.processes:
            p.terminate()
