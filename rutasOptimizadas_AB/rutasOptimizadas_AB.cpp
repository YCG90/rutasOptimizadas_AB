#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <stack>
#include <string>
#include <set>
#include <limits>
#include <algorithm>

// Definición de la clase Graph.
class Graph {
private:
    // El grafo se representa como un mapa de una ciudad a una lista de pares,
    // donde cada par contiene una ciudad vecina y la distancia a ella.
    std::map<std::string, std::vector<std::pair<std::string, int>>> adjList;

public:
    // Método para agregar una arista con distancia al grafo.
    // Dado que el grafo es no dirigido, agregamos la arista en ambas direcciones.
    void addEdge(const std::string& from, const std::string& to, int distance) {
        adjList[from].emplace_back(to, distance);
        adjList[to].emplace_back(from, distance);
    }

    // Método BFS para encontrar la ruta más corta y la distancia entre dos ciudades.
    void BFS(const std::string& start, const std::string& end) {
        // Verificar que las ciudades de inicio y fin existan en el grafo.
        if (adjList.find(start) == adjList.end() || adjList.find(end) == adjList.end()) {
            std::cout << "One of the cities is not in the graph." << std::endl;
            return;
        }

        // Inicialización de estructuras para rastrear nodos visitados, distancias y predecesores.
        std::map<std::string, bool> visited;
        std::map<std::string, int> distances;
        std::map<std::string, std::string> predecessors;
        std::queue<std::string> queue;

        // Inicializar todas las distancias a infinito y visitado a falso.
        for (const auto& node : adjList) {
            distances[node.first] = std::numeric_limits<int>::max();
            visited[node.first] = false;
        }

        // Configurar el nodo de inicio.
        visited[start] = true;
        distances[start] = 0;
        queue.push(start);

        // Bucle principal de BFS.
        while (!queue.empty()) {
            // Sacar el primer elemento de la cola.
            std::string current = queue.front();
            queue.pop();

            // Iterar sobre todos los vecinos del nodo actual.
            for (const auto& edge : adjList[current]) {
                std::string neighbor = edge.first;
                int weight = edge.second;

                // Si encontramos una ruta más corta al vecino, actualizamos la distancia y el predecesor.
                if (!visited[neighbor] || distances[current] + weight < distances[neighbor]) {
                    visited[neighbor] = true;
                    distances[neighbor] = distances[current] + weight;
                    predecessors[neighbor] = current;
                    queue.push(neighbor);
                }
            }
        }

        // Si el nodo final no fue visitado, no hay camino.
        if (!visited[end]) {
            std::cout << "No path found from " << start << " to " << end << std::endl;
            return;
        }

        // Reconstruir la ruta más corta utilizando la información de predecesores.
        std::vector<std::string> path;
        std::string at = end;
        while (at != start) {
            path.push_back(at);
            at = predecessors[at];
        }
        path.push_back(start);

        // Invertir la ruta para que vaya desde el inicio hasta el final.
        std::reverse(path.begin(), path.end());

        // Imprimir la ruta y la distancia total.
        std::cout << "BFS - Shortest path from " << start << " to " << end << ": ";
        for (const auto& node : path) {
            std::cout << node << " ";
        }
        std::cout << "\nTotal distance: " << distances[end] << " km" << std::endl;
    }

    // Método auxiliar DFS que realiza la búsqueda recursiva.
    bool DFSUtil(const std::string& node, const std::string& end,
        std::set<std::string>& visited, std::vector<std::string>& path,
        int& pathDistance) {
        // Marcar el nodo actual como visitado y añadirlo al camino.
        visited.insert(node);
        path.push_back(node);

        // Si hemos llegado al nodo de destino, retornar verdadero.
        if (node == end) {
            return true;
        }

        // Iterar sobre todos los vecinos del nodo actual.
        for (const auto& neighbor : adjList[node]) {
            // Si el vecino no ha sido visitado, continuar la búsqueda.
            if (visited.find(neighbor.first) == visited.end()) {
                // Agregar la distancia al vecino a la distancia total del camino.
                pathDistance += neighbor.second;
                // Si la búsqueda desde el vecino es exitosa, retornar verdadero.
                if (DFSUtil(neighbor.first, end, visited, path, pathDistance)) {
                    return true;
                }
                // Si el vecino no lleva al destino, restar la distancia y continuar.
                pathDistance -= neighbor.second;
            }
        }

        // Si no se encontró el destino desde este nodo, retirarlo del camino y retornar falso.
        path.pop_back();
        return false;
    }

    // Método DFS para encontrar una ruta entre dos ciudades y calcular la distancia.
    void DFS(const std::string& start, const std::string& end) {
        // Conjunto para rastrear nodos visitados y vector para almacenar el camino.
        std::set<std::string> visited;
        std::vector<std::string> path;
        // Variable para llevar la cuenta de la distancia total del camino.
        int pathDistance = 0;

        // Iniciar la búsqueda DFS.
        if (!DFSUtil(start, end, visited, path, pathDistance)) {
            std::cout << "DFS - No path found from " << start << " to " << end << std::endl;
        }
        else {
            // Si se encuentra un camino, imprimirlo junto con la distancia total.
            std::cout << "DFS - Path from " << start << " to " << end << ": ";
            for (const auto& node : path) {
                std::cout << node << " ";
            }
            std::cout << "\nTotal distance: " << pathDistance << " km" << std::endl;
        }
    }
};

// Función principal que interactúa con el usuario y ejecuta los métodos de búsqueda.
int main() {
    // Crear un objeto de la clase Graph.
    Graph g;

    // Añadir aristas con distancias al grafo.
    g.addEdge("MADRID", "CIUDAD REAL", 100);
    g.addEdge("CIUDAD REAL", "SALAMANCA", 100);
    g.addEdge("MADRID", "TOLEDO", 80);
    g.addEdge("MADRID", "SALAMANCA", 190);
    g.addEdge("TOLEDO", "ALBACETE", 150);
    g.addEdge("TOLEDO", "JAEN", 180);
    g.addEdge("TOLEDO", "CIUDAD REAL", 50);
    g.addEdge("SALAMANCA", "GUADALAJARA", 100);
    g.addEdge("SALAMANCA", "CACERES", 150);
    g.addEdge("CACERES", "JAEN", 150);
    g.addEdge("SALAMANCA", "JAEN", 100);

    // Variables para almacenar las ciudades de inicio y fin ingresadas por el usuario.
    std::string startNode, endNode;
    std::cout << "Enter the starting city: ";
    std::cin >> startNode;
    std::cout << "Enter the destination city: ";
    std::cin >> endNode;

    // Realizar la búsqueda en anchura (BFS) y en profundidad (DFS).
    std::cout << "Performing BFS..." << std::endl;
    g.BFS(startNode, endNode);
    std::cout << "Performing DFS..." << std::endl;
    g.DFS(startNode, endNode);

    return 0;
}
