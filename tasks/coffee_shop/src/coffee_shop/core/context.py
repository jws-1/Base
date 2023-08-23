import yaml
from enum import Enum
from collections import defaultdict

class Table:

    class Status(Enum):
        NEEDS_SERVING = 0
        SERVED = 1
        READY = 2
        NEEDS_CLEANING = 3
        UNVISITED = 4

    def __init__(self, idx):
        self.idx = idx
        self.status = Table.Status.UNVISITED

    def __repr__(self):
        return f"{self.idx} {self.statusString()}"

    def statusString(self):
        return {
            Table.Status.NEEDS_SERVING: "needs serving",
            Table.Status.SERVED: "served",
            Table.Status.READY: "ready",
            Table.Status.NEEDS_CLEANING: "needs cleaning",
            Table.Status.UNVISITED: "unvisited"
        }[self.status]

class Context:

    def __init__(self, config):
        with open(config) as fp:
            data = yaml.load(fp, Loader=yaml.SafeLoader)
        self._tables = [ Table(int(k.replace("table", ""))) for k in data["tables"].keys() if k.startswith("table") ]
        self._currentTable = self.tables[0]

    def allVisited(self):
        return not any([t.status == Table.Status.UNVISITED for t in self.tables])
    
    def unvisited(self):
        return [table for table in self._tables if table.status == Table.Status.UNVISITED]

    def needsServing(self):
        return [table for table in self._tables if table.status == Table.Status.NEEDS_SERVING]

    def ready(self):
        return [table for table in self._tables if table.status == Table.Status.READY]

    def current(self):
        return self._currentTable

    def visit(self, table, status):
        table.status = status