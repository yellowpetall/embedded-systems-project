"""
The lot as a graph.

Lane points ("nodes") are joined by edges you can drive along. Every slot hangs
off one node. At start-up we run Dijkstra once from the entrance, so we know:
  - how far every slot is from the entrance (real driving distance), and
  - the exact route to it (used to animate the guidance LEDs).
Slots are then sorted by distance ONCE, so finding the nearest free slot is a
simple walk down that list - fine for thousands of slots.
"""
import heapq
from dataclasses import dataclass, field


@dataclass
class SlotSpec:
    id: str
    node: str
    side: str
    x: float
    y: float
    sensor: str
    led: dict
    distance: float = 0.0
    route: list = field(default_factory=list)   # node ids, entrance -> slot's node


class Layout:
    def __init__(self, cfg):
        lane = cfg["lane"]
        self.nodes = {n["id"]: n for n in lane["nodes"]}
        self.entrance = lane["entrance"]
        if self.entrance not in self.nodes:
            raise ValueError(f"lane.entrance {self.entrance!r} is not a lane node")

        self.adj = {nid: [] for nid in self.nodes}
        for edge in lane["edges"]:
            a, b = edge[0], edge[1]
            length = edge[2] if len(edge) > 2 else 1.0
            for n in (a, b):
                if n not in self.nodes:
                    raise ValueError(f"edge {edge} uses unknown node {n!r}")
            self.adj[a].append((b, length))
            self.adj[b].append((a, length))

        self.dist, self.prev = self._dijkstra(self.entrance)

        self.slots = []
        seen = set()
        for s in cfg["slots"]:
            if s["id"] in seen:
                raise ValueError(f"duplicate slot id {s['id']!r}")
            seen.add(s["id"])
            if s["node"] not in self.nodes:
                raise ValueError(f"slot {s['id']} uses unknown node {s['node']!r}")
            if s["node"] not in self.dist:
                raise ValueError(f"slot {s['id']} can't be reached from the entrance")
            spec = SlotSpec(s["id"], s["node"], s.get("side", ""), s.get("x", 0),
                            s.get("y", 0), s["sensor"], s["led"])
            spec.distance = self.dist[spec.node]
            spec.route = self._route(spec.node)
            self.slots.append(spec)

        # nearest first; ties broken by side then id so the choice is predictable
        self.slots.sort(key=lambda s: (s.distance, s.side, s.id))

    def _dijkstra(self, start):
        dist, prev = {start: 0.0}, {}
        heap = [(0.0, start)]
        while heap:
            d, n = heapq.heappop(heap)
            if d > dist[n]:
                continue
            for nxt, length in self.adj[n]:
                nd = d + length
                if nd < dist.get(nxt, float("inf")):
                    dist[nxt], prev[nxt] = nd, n
                    heapq.heappush(heap, (nd, nxt))
        return dist, prev

    def _route(self, node):
        path = [node]
        while path[-1] != self.entrance:
            path.append(self.prev[path[-1]])
        return path[::-1]
