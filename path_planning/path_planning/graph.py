from collections import deque
from utils.yaml_editor import YamlEditor


class Graph:
    """
    This Class encapsulates methods for Graph Creation and Manipulation.
    """

    _singleton = None

    def __init__(self):
        if Graph._singleton is not None:
            raise Exception("Graph is a singleton class")
        else:
            try:
                Graph._singleton = self
                self.trajectory_map = {}
            except Exception as err_msg:
                raise Exception(str(err_msg), "Unknown")

    @staticmethod
    def get():
        """To get the singleton instance of this class.

        Returns:
            Graph: Returns the singleton instance of the class.
        """
        if Graph._singleton is None:
            Graph()
        return Graph._singleton

    def __bfs_path(self, graph, start, goal):
        queue = deque([(start, [], [])])
        visited = set()
        while queue:
            current_node, path, directions = queue.popleft()
            if current_node == goal:
                return path + [current_node], directions
            if current_node in visited:
                continue
            visited.add(current_node)
            for direction, neighbor in graph[current_node].items():
                if neighbor and neighbor not in visited:
                    queue.append(
                        (neighbor, path + [current_node], directions + [direction])
                    )
        return None, None

    def __generate_segment_path(self, map_dict, start, goal):

        path, directions = self.__bfs_path(map_dict, start, goal)

        if path is None:
            return None, None, None

        left_tags = [path[i] for i in range(len(directions)) if directions[i] == "left"]
        right_tags = [path[i] for i in range(len(directions)) if directions[i] == "right"]

        return path, left_tags, right_tags

    def plan_path(self, start, goal):
        """Generates a path using the stored trajectory map."""
        if not self.trajectory_map:
            return None, None, None

        return self.__generate_segment_path(self.trajectory_map, start, goal)

    def from_yaml(self, file_path) -> None:
        """Reads YAML file and stores trajectory map and global path separately."""
        try:
            yaml_data = YamlEditor().read(file_path)

            self.trajectory_map = yaml_data.get("trajectory_map", {})

        except Exception as e:
            raise Exception(str(e), "Unknown")
