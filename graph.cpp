//
// Created by marcus on 29/03/24.
//
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

using namespace std;

// Estrutura para representar uma aresta
struct Edge {
    int src, dest;
};

class Graph {
private:
    vector<vector<int>> adjacencyMatrix;
    vector<int> adjacencyList[10001];
    int numVertices;
    bool directed;

public:
    // Construtor
    Graph(int numVertices, bool directed) {
        this->numVertices = numVertices;
        this->directed = directed;

        // Inicialização da matriz de adjacência
        adjacencyMatrix.resize(numVertices + 1, vector<int>(numVertices + 1, 0));
    }

    // Função para adicionar uma aresta na matriz de adjacência
    void addEdgeMatrix(int src, int dest) {
        adjacencyMatrix[src][dest] = 1;
        if (!directed) {
            adjacencyMatrix[dest][src] = 1;
        }
    }

    // Função para adicionar uma aresta na lista de adjacência
    void addEdgeList(int src, int dest) {
        adjacencyList[src].push_back(dest);
        if (!directed) {
            adjacencyList[dest].push_back(src);
        }
    }

    // Função para remover uma aresta da matriz de adjacência
    void removeEdgeMatrix(int src, int dest) {
        adjacencyMatrix[src][dest] = 0;
        if (!directed) {
            adjacencyMatrix[dest][src] = 0;
        }
    }

    // Função para remover uma aresta da lista de adjacência
    void removeEdgeList(int src, int dest) {
        for (auto it = adjacencyList[src].begin(); it != adjacencyList[src].end(); ++it) {
            if (*it == dest) {
                adjacencyList[src].erase(it);
                break;
            }
        }
        if (!directed) {
            for (auto it = adjacencyList[dest].begin(); it != adjacencyList[dest].end(); ++it) {
                if (*it == src) {
                    adjacencyList[dest].erase(it);
                    break;
                }
            }
        }
    }

    // Função para verificar a vizinhança de um vértice na matriz de adjacência
    vector<int> getNeighborsMatrix(int vertex) {
        vector<int> neighbors;
        for (int i = 1; i <= numVertices; ++i) {
            if (adjacencyMatrix[vertex][i] == 1) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    // Função para verificar a vizinhança de um vértice na lista de adjacência
    vector<int> getNeighborsList(int vertex) {
        return adjacencyList[vertex];
    }

    // Função para verificar os sucessores de um vértice na matriz de adjacência (para grafo direcionado)
    vector<int> getSuccessorsMatrix(int vertex) {
        vector<int> successors;
        for (int i = 1; i <= numVertices; ++i) {
            if (adjacencyMatrix[vertex][i] == 1) {
                successors.push_back(i);
            }
        }
        return successors;
    }

    // Função para verificar os sucessores de um vértice na lista de adjacência (para grafo direcionado)
    vector<int> getSuccessorsList(int vertex) {
        return adjacencyList[vertex];
    }

    // Função para verificar os predecessores de um vértice na matriz de adjacência (para grafo direcionado)
    vector<int> getPredecessorsMatrix(int vertex) {
        vector<int> predecessors;
        for (int i = 1; i <= numVertices; ++i) {
            if (adjacencyMatrix[i][vertex] == 1) {
                predecessors.push_back(i);
            }
        }
        return predecessors;
    }

    // Função para verificar os predecessores de um vértice na lista de adjacência (para grafo direcionado)
    vector<int> getPredecessorsList(int vertex) {
        vector<int> predecessors;
        for (int i = 1; i <= numVertices; ++i) {
            for (int j : adjacencyList[i]) {
                if (j == vertex) {
                    predecessors.push_back(i);
                    break;
                }
            }
        }
        return predecessors;
    }

    // Função para verificar o grau de um vértice na matriz de adjacência
    int getDegreeMatrix(int vertex) {
        int degree = 0;
        for (int i = 1; i <= numVertices; ++i) {
            if (adjacencyMatrix[vertex][i] == 1) {
                degree++;
            }
        }
        return degree;
    }

    // Função para verificar o grau de um vértice na lista de adjacência
    int getDegreeList(int vertex) {
        return adjacencyList[vertex].size();
    }

    // Função para verificar se o grafo é simples
    bool isSimpleMatrix() {
        for (int i = 1; i <= numVertices; ++i) {
            for (int j = 1; j <= numVertices; ++j) {
                if (adjacencyMatrix[i][j] == 1 && i == j) {
                    return false; // Grafo não é simples, pois há laços
                } else if (adjacencyMatrix[i][j] > 1 && adjacencyMatrix[j][i] > 1) {
                    return false; // Grafo não é simples, pois há arestas paralelas
                }
            }
        }
        return true; // Grafo é simples
    }

    // Função para verificar se o grafo é simples usando lista de adjacência
    bool isSimpleList() {
        for (int i = 1; i <= numVertices; ++i) {
            for (int j : adjacencyList[i]) {
                if (j == i || count(adjacencyList[j].begin(), adjacencyList[j].end(), i) > 0) {
                    return false; // Se houver laços ou arestas paralelas, o grafo não é simples
                }
            }
        }
        return true; // Grafo é simples
    }


    // Função para verificar se o grafo é regular
    bool isRegularMatrix() {
        int degree = getDegreeMatrix(1); // Grau do primeiro vértice
        for (int i = 2; i <= numVertices; ++i) {
            if (getDegreeMatrix(i) != degree) {
                return false; // Grafo não é regular
            }
        }
        return true; // Grafo é regular
    }

    // Função para verificar se o grafo é regular usando lista de adjacência
    bool isRegularList() {
        int degree = adjacencyList[1].size(); // Grau do primeiro vértice
        for (int i = 2; i <= numVertices; ++i) {
            if (adjacencyList[i].size() != degree) {
                return false; // Se o grau de qualquer vértice for diferente do grau do primeiro vértice, o grafo não é regular
            }
        }
        return true; // Grafo é regular
    }


    // Função para verificar se o grafo é completo
    bool isCompleteMatrix() {
        for (int i = 1; i <= numVertices; ++i) {
            for (int j = 1; j <= numVertices; ++j) {
                if (i != j && adjacencyMatrix[i][j] != 1) {
                    return false; // Grafo não é completo
                }
            }
        }
        return true; // Grafo é completo
    }

    // Função para verificar se o grafo é completo usando lista de adjacência
    bool isCompleteList() {
        for (int i = 1; i <= numVertices; ++i) {
            if (adjacencyList[i].size() != numVertices - 1) {
                return false; // Se o tamanho da lista de adjacência de qualquer vértice não for (numVertices - 1), o grafo não é completo
            }
        }
        return true; // Grafo é completo
    }


    // Função para verificar se o grafo é bipartido
    bool isBipartiteList() {
        vector<int> color(numVertices + 1, -1); // Inicializa todos os vértices sem cor (-1)
        for (int i = 1; i <= numVertices; ++i) {
            if (color[i] == -1) { // Se o vértice não foi visitado
                color[i] = 0; // Atribui a cor 0 ao vértice inicial
                queue<int> q;
                q.push(i);

                while (!q.empty()) {
                    int u = q.front();
                    q.pop();

                    for (int v : adjacencyList[u]) {
                        if (color[v] == -1) { // Se o vértice não foi visitado
                            color[v] = 1 - color[u]; // Atribui a cor oposta ao vértice adjacente
                            q.push(v);
                        } else if (color[v] == color[u]) { // Se o vértice adjacente tiver a mesma cor
                            return false; // Grafo não é bipartido
                        }
                    }
                }
            }
        }
        return true; // Grafo é bipartido
    }

    // Função para verificar se o grafo é bipartido usando matriz de adjacência
    bool isBipartiteMatrix() {
        vector<int> color(numVertices + 1, -1); // Inicializa todos os vértices sem cor (-1)
        for (int i = 1; i <= numVertices; ++i) {
            if (color[i] == -1) { // Se o vértice não foi visitado
                color[i] = 0; // Atribui a cor 0 ao vértice inicial
                queue<int> q;
                q.push(i);

                while (!q.empty()) {
                    int u = q.front();
                    q.pop();

                    for (int v = 1; v <= numVertices; ++v) {
                        if (adjacencyMatrix[u][v] == 1) { // Se houver uma aresta de u para v
                            if (color[v] == -1) { // Se o vértice não foi visitado
                                color[v] = 1 - color[u]; // Atribui a cor oposta ao vértice adjacente
                                q.push(v);
                            } else if (color[v] == color[u]) { // Se o vértice adjacente tiver a mesma cor
                                return false; // Grafo não é bipartido
                            }
                        }
                    }
                }
            }
        }
        return true; // Grafo é bipartido
    }

    // Método para imprimir o grafo (matriz de adjacência)
    void printGraphMatrix() {
        cout << "Matriz de Adjacência:\n";
        for (int i = 0; i < numVertices; ++i) {
            for (int j = 0; j < numVertices; ++j) {
                cout << adjacencyMatrix[i][j] << " ";
            }
            cout << endl;
        }
    }

    // Método para imprimir o grafo (lista de adjacência)
    void printGraphList() {
        cout << "Lista de Adjacência:\n";
        for (int i = 0; i < numVertices; ++i) {
            cout << "Vértice " << i << ": ";
            for (int vertex : adjacencyList[i]) {
                cout << vertex << " ";
            }
            cout << endl;
        }
    }
};
