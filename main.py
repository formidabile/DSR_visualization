import heapq
import networkx as nx
import matplotlib.pyplot as plt
from threading import Thread, Lock, Event
import random

nodes = 10
start, goal = '1', str(nodes)
priority_queue = [(0, start)]
distances = {}
came_from = {}
distances[start] = 0
came_from[start] = None
lock = Lock()
is_goal = Event()
edges = []

def build_graph(nodes):
    graph = {}
    all_nodes = [str(i) for i in range(1, nodes + 1)]
    for i in range(len(all_nodes)):
        j = 0
        neighbors = []
        while j < 3:
            n = random.choice(all_nodes)
            if n == all_nodes[i] or n in neighbors:
                continue
            neighbors.append(n)
            j += 1
        distances = {}
        for n in neighbors:
            distances[n] = 1
        graph[all_nodes[i]] = distances

    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            if neighbor not in graph:
                graph[neighbor] = {}
            if node not in graph[neighbor]:
                graph[neighbor][node] = weight
    return graph

def plot_graph(graph):
    G = nx.Graph()
    for vertex, neighbors in graph.items():
        G.add_node(vertex)
        for v, distance in neighbors.items():
            G.add_edge(vertex, v, weight=distance, length=distance)
    return G

def go_next(G, graph, vertex, dist, goal):
    global is_goal
    for neighbor, weight in graph[vertex].items():
        distance = dist + weight
        if neighbor not in distances or distance < distances[neighbor]:
            lock.acquire()
            distances[neighbor] = distance
            heapq.heappush(priority_queue, (distance, neighbor))
            came_from[neighbor] = vertex
            print(f"Sending RREP from {neighbor} to {vertex}")
            edges.append((vertex, neighbor))
            if neighbor == goal:
                is_goal.set()
            lock.release()
    fig, axs = plt.subplots(1, 1)
    axs.set_title(f'Sending RREQ from {vertex}')
    pos = nx.shell_layout(G)
    nx.draw(G, pos, with_labels=True)
    nx.draw_networkx_edges(G, pos=pos, edgelist=edges, edge_color="r", width=3)
    fig.align_titles()
    plt.show()

def dijkstra(G, graph, start, goal):
    global is_goal
    visited = set()
    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)
        if is_goal.is_set():
            while current_vertex != goal:
                current_distance, current_vertex = heapq.heappop(priority_queue)
        if current_vertex in visited:
            continue
        visited.add(current_vertex)
        if current_vertex == goal:
            return came_from, distances
        print(f'Sending RREQ from {current_vertex}')
        t = Thread(target=go_next, args=[G, graph, current_vertex, current_distance, goal])
        t.run()
        #go_next(G, graph, current_vertex, current_distance, goal)
    return came_from, distances

def reconstruct_path(G, came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    edges = []
    for i in range(len(path) - 1):
        edges.append((path[i], path[i + 1]))
    print(f"Shortest path from {start} to {goal} is {path}")
    fig, axs = plt.subplots(1, 1)
    axs.set_title(f"Shortest path from {start} to {goal}")
    pos = nx.spring_layout(G)
    nx.draw(G, pos, with_labels=True)
    nx.draw_networkx_edges(G, pos=pos, edgelist=edges, edge_color="r", width=3)
    plt.axis('off')
    plt.show()
    return path

graph = build_graph(nodes)
G = plot_graph(graph)
dijkstra(G, graph, start, goal)
reconstruct_path(G, came_from, start, goal)