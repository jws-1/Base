import yaml
from enum import Enum
import numpy as np
class Table:

    class Status(Enum):
        NEEDS_SERVING = 0
        SERVED = 1
        READY = 2
        NEEDS_CLEANING = 3
        UNVISITED = 4
        VISITING = 5
        SERVING = 6

    def __init__(self, idx, position, orientation, objects_cuboid, persons_cuboid):
        self.idx = idx
        self.position = position
        self.orientation = orientation
        self.objects_cuboid = objects_cuboid
        self.persons_cuboid = persons_cuboid
        self.status = Table.Status.UNVISITED
        self.order = []

    def __repr__(self):
        return f"{self.idx} {self.status_string()}"

    def status_string(self):
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
        self._tables = [ Table(k, v["location"]["position"], v["location"]["orientation"], v["objects_cuboid"], v["persons_cuboid"]) for k, v in data["tables"].items() if k.startswith("table") ]
        self._current = self.tables[0]

    def all_visited(self):
        return not any([t.status == Table.Status.UNVISITED for t in self.tables])
    
    def unvisited(self):
        return [table for table in self._tables if table.status == Table.Status.UNVISITED]

    def needs_serving(self):
        return [table for table in self._tables if table.status == Table.Status.NEEDS_SERVING]

    def ready(self):
        return [table for table in self._tables if table.status == Table.Status.READY]

    def current(self):
        return self._currentTable

    def visit(self, table, status):
        self._current = table
        table.status = status

    def closest(self, x, y, status=None):
        return min([t for t in self._tables if status is None or t.status == status], key=lambda t: np.linalg.norm(t.position["x"] - x, t.position["y"] - y))

