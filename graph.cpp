//
// Created by marcus on 29/03/24.
//
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>

using namespace std;

struct Aresta {
    int origem, destino, peso;
};

class Graph {
private:
    vector<vector<int>> adjacencyMatrix;
    vector<int> adjacencyList[10001];
    vector<vector<int>> peso;
    int numVertices;
    bool directed;

public:
    // Construtor
    Graph(int numVertices, bool directed) {
        this->numVertices = numVertices;
        this->directed = directed;

        // Inicialização da matriz de adjacência
        adjacencyMatrix.resize(numVertices + 1, vector<int>(numVertices + 1, 0));
        peso.resize(numVertices + 1, vector<int>(numVertices + 1, 0));
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

    // Função para adicionar uma aresta na matriz de adjacência com peso
    void addEdgeMatrixWithWeight(int src, int dest, int weight) {
        adjacencyMatrix[src][dest] = 1;
        peso[src][dest] = weight;
        if (!directed) {
            adjacencyMatrix[dest][src] = 1;
            peso[dest][src] = weight;
        }
    }

    // Função para adicionar uma aresta na lista de adjacência com peso
    void addEdgeListWithWeight(int src, int dest, int weight) {
        adjacencyList[src].push_back(dest);
        peso[src][dest] = weight;
        if (!directed) {
            adjacencyList[dest].push_back(src);
            peso[dest].push_back(src);
        }
    }

    // Função para remover uma aresta da matriz de adjacência com peso
    void removeEdgeMatrixWithWeight(int src, int dest) {
        adjacencyMatrix[src][dest] = 0;
        peso[src][dest] = 0;
        if (!directed) {
            adjacencyMatrix[dest][src] = 0;
            peso[dest][src] = 0;
        }
    }

// Função para remover uma aresta da lista de adjacência com peso
    void removeEdgeListWithWeight(int src, int dest) {
        for (int i = 0; i < adjacencyList[src].size(); ++i) {
            if (adjacencyList[src][i] == dest) {
                adjacencyList[src].erase(adjacencyList[src].begin() + i);
                peso[src][dest] = 0;
                if (!directed) {
                    for (int j = 0; j < adjacencyList[dest].size(); ++j) {
                        if (adjacencyList[dest][j] == src) {
                            adjacencyList[dest].erase(adjacencyList[dest].begin() + j);
                            peso[dest][src] = 0;
                            break;
                        }
                    }
                }
                break;
            }
        }
    }

    // Função para adicionar várias arestas com peso usando uma lista de pares de vértices e peso
    void addEdgesFromList(vector<pair<pair<int, int>, int>> edges) {
        for (auto edge : edges) {
            int src = edge.first.first;
            int dest = edge.first.second;
            int weight = edge.second;
            addEdgeListWithWeight(src, dest, weight);
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

    void breadthFirstSearchList(int startVertex) {
        // Array para marcar os vértices visitados
        bool visited[numVertices + 1];
        // Array para armazenar o nível de cada vértice em relação ao vértice inicial
        int level[numVertices + 1];
        // Array para armazenar o pai de cada vértice na árvore de busca em largura
        int parent[numVertices + 1];
        // Array para armazenar a ordem de visita dos vértices
        int visitOrder[numVertices + 1];
        // Variável para acompanhar a próxima posição no array de ordem de visita
        int visitIndex = 0;

        // Inicialização dos arrays
        for (int i = 1; i <= numVertices; ++i) {
            visited[i] = false;
            level[i] = -1;
            parent[i] = -1;
        }

        // Marca o vértice inicial como visitado e define o nível como 0
        visited[startVertex] = true;
        level[startVertex] = 0;

        // Fila para realizar a busca em largura
        queue<int> q;
        // Adiciona o vértice inicial à fila
        q.push(startVertex);

        // Enquanto a fila não estiver vazia
        while (!q.empty()) {
            // Remove o vértice da frente da fila
            int currentVertex = q.front();
            q.pop();

            // Armazena o vértice na ordem de visita
            visitOrder[visitIndex++] = currentVertex;

            // Obtém os vizinhos do vértice atual na lista de adjacência
            vector<int> neighbors = getNeighborsList(currentVertex);

            // Para cada vizinho do vértice atual
            for (int neighbor : neighbors) {
                // Se o vizinho ainda não foi visitado
                if (!visited[neighbor]) {
                    // Marca o vizinho como visitado
                    visited[neighbor] = true;
                    // Define o nível do vizinho como sendo 1 nível mais do que o nível do vértice atual
                    level[neighbor] = level[currentVertex] + 1;
                    // Define o pai do vizinho como o vértice atual
                    parent[neighbor] = currentVertex;
                    // Adiciona o vizinho à fila para explorar seus vizinhos posteriormente
                    q.push(neighbor);
                }
            }
        }

        // Mostra os níveis, pais e ordem de visita dos vértices após a busca em largura
        cout << "Níveis, pais e ordem de visita dos vértices a partir do vértice " << startVertex << ":\n";
        for (int i = 1; i <= numVertices; ++i) {
            cout << "Vértice " << i << ": Nível " << level[i] << ", Pai " << parent[i] << ", Visitado em ordem " << visitOrder[i - 1] << endl;
        }
    }

    void breadthFirstSearchMatrix(int startVertex) {
        // Array para marcar os vértices visitados
        bool visited[numVertices + 1];
        // Array para armazenar o nível de cada vértice em relação ao vértice inicial
        int level[numVertices + 1];
        // Array para armazenar o pai de cada vértice na árvore de busca em largura
        int parent[numVertices + 1];
        // Array para armazenar a ordem de visita dos vértices
        int visitOrder[numVertices];
        // Variável para acompanhar a próxima posição no array de ordem de visita
        int visitIndex = 0;

        // Inicialização dos arrays
        for (int i = 1; i <= numVertices; ++i) {
            visited[i] = false;
            level[i] = -1;
            parent[i] = -1;
        }

        // Marca o vértice inicial como visitado e define o nível como 0
        visited[startVertex] = true;
        level[startVertex] = 0;

        // Fila para realizar a busca em largura
        queue<int> q;
        // Adiciona o vértice inicial à fila
        q.push(startVertex);

        // Enquanto a fila não estiver vazia
        while (!q.empty()) {
            // Remove o vértice da frente da fila
            int currentVertex = q.front();
            q.pop();

            // Armazena o vértice na ordem de visita
            visitOrder[visitIndex++] = currentVertex;

            // Para cada vértice adjacente ao vértice atual
            for (int i = 1; i <= numVertices; ++i) {
                // Se houver uma aresta do vértice atual para o vértice i
                if (adjacencyMatrix[currentVertex][i] == 1) {
                    // Se o vértice i ainda não foi visitado
                    if (!visited[i]) {
                        // Marca o vértice i como visitado
                        visited[i] = true;
                        // Define o nível do vértice i como sendo 1 nível mais do que o nível do vértice atual
                        level[i] = level[currentVertex] + 1;
                        // Define o pai do vértice i como o vértice atual
                        parent[i] = currentVertex;
                        // Adiciona o vértice i à fila para explorar seus vizinhos posteriormente
                        q.push(i);
                    }
                }
            }
        }

        // Mostra os níveis, pais e ordem de visita dos vértices após a busca em largura
        cout << "Níveis, pais e ordem de visita dos vértices a partir do vértice " << startVertex << ":\n";
        for (int i = 1; i <= numVertices; ++i) {
            cout << "Vértice " << i << ": Nível " << level[i] << ", Pai " << parent[i] << ", Visitado em ordem " << visitOrder[i - 1] << endl;
        }
    }

    void depthFirstSearchMatrix(int vertex, bool visited[], int discoveryTime[], int finishTime[], int parent[]) {
        // Marca o vértice como visitado
        visited[vertex] = true;

        // Incrementa o tempo de descoberta do vértice atual
        static int time = 0;
        discoveryTime[vertex] = ++time;

        // Para cada vértice adjacente ao vértice atual
        for (int i = 1; i <= numVertices; ++i) {
            // Se houver uma aresta do vértice atual para o vértice i e o vértice i não foi visitado ainda
            if (adjacencyMatrix[vertex][i] == 1 && !visited[i]) {
                // Define o vértice atual como pai do vértice i
                parent[i] = vertex;
                // Realiza a busca em profundidade recursivamente no vértice i
                depthFirstSearchMatrix(i, visited, discoveryTime, finishTime, parent);
            }
        }

        // Incrementa o tempo de finalização do vértice atual
        finishTime[vertex] = ++time;
    }

    void depthFirstSearchList(int vertex, bool visited[], int discoveryTime[], int finishTime[], int parent[]) {
        // Marca o vértice como visitado
        visited[vertex] = true;

        // Incrementa o tempo de descoberta do vértice atual
        static int time = 0;
        discoveryTime[vertex] = ++time;

        // Obtém os vizinhos do vértice atual na lista de adjacência
        vector<int> neighbors = getNeighborsList(vertex);

        // Para cada vizinho do vértice atual
        for (int neighbor : neighbors) {
            // Se o vizinho não foi visitado ainda
            if (!visited[neighbor]) {
                // Define o vértice atual como pai do vizinho
                parent[neighbor] = vertex;
                // Realiza a busca em profundidade recursivamente no vizinho
                depthFirstSearchList(neighbor, visited, discoveryTime, finishTime, parent);
            }
        }

        // Incrementa o tempo de finalização do vértice atual
        finishTime[vertex] = ++time;
    }

    vector<int> ordenacaoTopologicaMatrix() {
        // Vetor para armazenar o grau de entrada de cada vértice
        vector<int> grauEntrada(numVertices + 1, 0);
        // Vetor para armazenar a ordenação topológica
        vector<int> ordenacao;
        // Fila para realizar a ordenação topológica
        queue<int> fila;

        // Calcula o grau de entrada de cada vértice
        for (int i = 1; i <= numVertices; ++i) {
            for (int j = 1; j <= numVertices; ++j) {
                if (adjacencyMatrix[j][i] == 1) {
                    grauEntrada[i]++;
                }
            }
        }

        // Adiciona os vértices com grau de entrada zero à fila
        for (int i = 1; i <= numVertices; ++i) {
            if (grauEntrada[i] == 0) {
                fila.push(i);
            }
        }

        // Realiza a ordenação topológica
        while (!fila.empty()) {
            int atual = fila.front();
            fila.pop();
            ordenacao.push_back(atual);

            // Para cada vértice adjacente ao vértice atual
            for (int i = 1; i <= numVertices; ++i) {
                if (adjacencyMatrix[atual][i] == 1) {
                    // Decrementa o grau de entrada do vértice adjacente
                    grauEntrada[i]--;
                    // Se o grau de entrada do vértice adjacente se tornar zero, o adiciona à fila
                    if (grauEntrada[i] == 0) {
                        fila.push(i);
                    }
                }
            }
        }

        // Retorna a ordenação topológica
        return ordenacao;
    }

    vector<int> ordenacaoTopologicaList() {
        // Vetor para armazenar o grau de entrada de cada vértice
        vector<int> grauEntrada(numVertices + 1, 0);
        // Vetor para armazenar a ordenação topológica
        vector<int> ordenacao;
        // Fila para realizar a ordenação topológica
        queue<int> fila;

        // Calcula o grau de entrada de cada vértice
        for (int i = 1; i <= numVertices; ++i) {
            vector<int> vizinhos = getNeighborsList(i);
            for (int vizinho : vizinhos) {
                grauEntrada[vizinho]++;
            }
        }

        // Adiciona os vértices com grau de entrada zero à fila
        for (int i = 1; i <= numVertices; ++i) {
            if (grauEntrada[i] == 0) {
                fila.push(i);
            }
        }

        // Realiza a ordenação topológica
        while (!fila.empty()) {
            int atual = fila.front();
            fila.pop();
            ordenacao.push_back(atual);

            // Para cada vizinho do vértice atual
            vector<int> vizinhos = getNeighborsList(atual);
            for (int vizinho : vizinhos) {
                // Decrementa o grau de entrada do vizinho
                grauEntrada[vizinho]--;
                // Se o grau de entrada do vizinho se tornar zero, o adiciona à fila
                if (grauEntrada[vizinho] == 0) {
                    fila.push(vizinho);
                }
            }
        }

        // Retorna a ordenação topológica
        return ordenacao;
    }

    // Função de comparação para ordenar as arestas por peso
    static bool compararArestas(const Aresta &a, const Aresta &b) {
        return a.peso < b.peso;
    }

    vector<Aresta> AGMKruskalMatrix() {
        // Vetor para armazenar a AGM
        vector<Aresta> agm;
        // Vetor para armazenar os conjuntos disjuntos (representados por índices)
        vector<int> conjuntos(numVertices + 1);
        // Vetor para armazenar as arestas ordenadas por peso
        vector<Aresta> arestas;

        // Inicialização dos conjuntos disjuntos
        for (int i = 1; i <= numVertices; ++i) {
            conjuntos[i] = i;
        }

        // Construção do vetor de arestas
        for (int i = 1; i <= numVertices; ++i) {
            for (int j = 1; j <= numVertices; ++j) {
                if (adjacencyMatrix[i][j] != 0) {
                    Aresta aresta;
                    aresta.origem = i;
                    aresta.destino = j;
                    aresta.peso = peso[i][j];
                    arestas.push_back(aresta);
                }
            }
        }

        // Ordenação das arestas por peso
        sort(arestas.begin(), arestas.end(), compararArestas);

        // Iteração sobre as arestas ordenadas
        for (Aresta aresta : arestas) {
            int conjuntoOrigem = encontrarConjunto(conjuntos, aresta.origem);
            int conjuntoDestino = encontrarConjunto(conjuntos, aresta.destino);

            // Se as arestas não pertencem ao mesmo conjunto, adiciona a aresta à AGM
            if (conjuntoOrigem != conjuntoDestino) {
                agm.push_back(aresta);
                unirConjuntos(conjuntos, conjuntoOrigem, conjuntoDestino);
            }
        }

        // Retorna a Árvore Geradora Mínima
        return agm;
    }

    vector<int> AGMPrimMatrix() {
        // Vetor para armazenar a AGM
        vector<int> agm;
        // Vetor para armazenar os custos mínimos de cada vértice até a AGM
        vector<int> custoMinimo(numVertices + 1, INT_MAX);
        // Vetor para armazenar os vértices já incluídos na AGM
        vector<bool> incluido(numVertices + 1, false);

        // Define o custo mínimo do primeiro vértice como 0 para iniciar a AGM a partir dele
        custoMinimo[1] = 0;

        // Loop para adicionar vértices à AGM
        for (int i = 1; i <= numVertices; ++i) {
            // Encontra o vértice com o custo mínimo que ainda não está incluído na AGM
            int verticeMinimo = -1;
            for (int j = 1; j <= numVertices; ++j) {
                if (!incluido[j] && (verticeMinimo == -1 || custoMinimo[j] < custoMinimo[verticeMinimo])) {
                    verticeMinimo = j;
                }
            }

            // Adiciona o vértice mínimo à AGM
            agm.push_back(verticeMinimo);
            incluido[verticeMinimo] = true;

            // Atualiza os custos mínimos dos vértices adjacentes ao vértice adicionado à AGM
            for (int j = 1; j <= numVertices; ++j) {
                if (adjacencyMatrix[verticeMinimo][j] != 0 && !incluido[j] && adjacencyMatrix[verticeMinimo][j] < custoMinimo[j]) {
                    custoMinimo[j] = peso[verticeMinimo][j];
                }
            }
        }

        // Retorna a Árvore Geradora Mínima
        return agm;
    }

    vector<int> AGMPrimList() {
        // Vetor para armazenar a AGM
        vector<int> agm;
        // Vetor para armazenar os custos mínimos de cada vértice até a AGM
        vector<int> custoMinimo(numVertices + 1, INT_MAX);
        // Vetor para armazenar os vértices já incluídos na AGM
        vector<bool> incluido(numVertices + 1, false);

        // Define o custo mínimo do primeiro vértice como 0 para iniciar a AGM a partir dele
        custoMinimo[1] = 0;

        // Loop para adicionar vértices à AGM
        for (int i = 1; i <= numVertices; ++i) {
            // Encontra o vértice com o custo mínimo que ainda não está incluído na AGM
            int verticeMinimo = -1;
            for (int j = 1; j <= numVertices; ++j) {
                if (!incluido[j] && (verticeMinimo == -1 || custoMinimo[j] < custoMinimo[verticeMinimo])) {
                    verticeMinimo = j;
                }
            }

            // Adiciona o vértice mínimo à AGM
            agm.push_back(verticeMinimo);
            incluido[verticeMinimo] = true;

            // Atualiza os custos mínimos dos vértices adjacentes ao vértice adicionado à AGM
            for (int j = 0; j < adjacencyList[verticeMinimo].size(); ++j) {
                int destino = adjacencyList[verticeMinimo][j];
                int pesoAtual = peso[verticeMinimo][j];
                if (!incluido[destino] && pesoAtual < custoMinimo[destino]) {
                    custoMinimo[destino] = pesoAtual;
                }
            }
        }

        // Retorna a Árvore Geradora Mínima
        return agm;
    }



    vector<Aresta> AGMKruskalList() {
        // Vetor para armazenar a AGM
        vector<Aresta> agm;
        // Vetor para armazenar os conjuntos disjuntos (representados por índices)
        vector<int> conjuntos(numVertices + 1);

        // Inicialização dos conjuntos disjuntos
        for (int i = 1; i <= numVertices; ++i) {
            conjuntos[i] = i;
        }

        // Construção do vetor de arestas
        vector<Aresta> arestas;
        for (int i = 1; i <= numVertices; ++i) {
            for (int j = 0; j < adjacencyList[i].size(); ++j) {
                int destinoVertice = adjacencyList[i][j];
                int pesoAresta = peso[i][j];
                Aresta aresta;
                aresta.origem = i;
                aresta.destino = destinoVertice;
                aresta.peso = pesoAresta;
                arestas.push_back(aresta);
            }
        }

        // Ordenação das arestas por peso
        sort(arestas.begin(), arestas.end(), compararArestas);

        // Iteração sobre as arestas ordenadas
        for (Aresta aresta : arestas) {
            int conjuntoOrigem = encontrarConjunto(conjuntos, aresta.origem);
            int conjuntoDestino = encontrarConjunto(conjuntos, aresta.destino);

            // Se as arestas não pertencem ao mesmo conjunto, adiciona a aresta à AGM
            if (conjuntoOrigem != conjuntoDestino) {
                agm.push_back(aresta);
                unirConjuntos(conjuntos, conjuntoOrigem, conjuntoDestino);
            }
        }

        // Retorna a Árvore Geradora Mínima
        return agm;
    }


    // Função para encontrar o conjunto a que um elemento pertence
    int encontrarConjunto(vector<int> &conjuntos, int elemento) {
        // Se o elemento não pertence ao conjunto dele mesmo, recursivamente encontra o conjunto de seu pai
        if (conjuntos[elemento] != elemento) {
            conjuntos[elemento] = encontrarConjunto(conjuntos, conjuntos[elemento]);
        }
        return conjuntos[elemento];
    }

    // Função para unir dois conjuntos em um único conjunto
    void unirConjuntos(vector<int> &conjuntos, int conjuntoA, int conjuntoB) {
        int raizA = encontrarConjunto(conjuntos, conjuntoA);
        int raizB = encontrarConjunto(conjuntos, conjuntoB);

        // Define a raiz do conjunto B como a raiz do conjunto A
        conjuntos[raizB] = raizA;
    }



    int encontrarMinimaDistancia(vector<int> &distancia, vector<bool> &processado, int numVertices) {
        int minimo = INT_MAX, indiceMinimo;

        for (int v = 1; v <= numVertices; v++) {
            if (!processado[v] && distancia[v] <= minimo) {
                minimo = distancia[v];
                indiceMinimo = v;
            }
        }

        return indiceMinimo;
    }

    vector<int> caminhoMinimoMatrix(int origem, int destino) {
        vector<int> distancia(numVertices + 1, INT_MAX); // Vetor para armazenar as distâncias mínimas
        vector<bool> processado(numVertices + 1, false); // Vetor para marcar os vértices já processados
        vector<int> pai(numVertices + 1, -1); // Vetor para armazenar os pais de cada vértice no caminho mínimo

        distancia[origem] = 0; // A distância da origem para ela mesma é zero

        // Encontrar o caminho mínimo para todos os vértices
        for (int count = 1; count <= numVertices - 1; count++) {
            int u = encontrarMinimaDistancia(distancia, processado, numVertices); // Encontrar o vértice com a menor distância ainda não processado
            processado[u] = true; // Marcar o vértice como processado

            // Atualizar as distâncias dos vértices adjacentes ao vértice escolhido
            for (int v = 1; v <= numVertices; v++) {
                if (!processado[v] && adjacencyMatrix[u][v] && distancia[u] != INT_MAX && distancia[u] + adjacencyMatrix[u][v] < distancia[v]) {
                    distancia[v] = distancia[u] + adjacencyMatrix[u][v];
                    pai[v] = u; // Atualizar o pai do vértice v
                }
            }
        }

        // Construir o caminho mínimo do destino para a origem
        vector<int> caminho;
        int verticeAtual = destino;
        while (verticeAtual != -1) {
            caminho.push_back(verticeAtual);
            verticeAtual = pai[verticeAtual];
        }
        reverse(caminho.begin(), caminho.end()); // Inverter o caminho para obter a ordem correta

        return caminho;
    }

    vector<int> caminhoMinimoList(int origem, int destino) {
        vector<int> distancia(numVertices + 1, INT_MAX); // Vetor para armazenar as distâncias mínimas
        vector<int> pai(numVertices + 1, -1); // Vetor para armazenar os pais de cada vértice no caminho mínimo
        vector<bool> visitado(numVertices + 1, false); // Vetor para marcar os vértices já visitados

        distancia[origem] = 0; // A distância da origem para ela mesma é zero

        // Fila de prioridade para selecionar o vértice com a menor distância
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> fila;
        fila.push({0, origem}); // Adiciona a origem à fila com distância zero

        // Loop principal do algoritmo de Dijkstra
        while (!fila.empty()) {
            int u = fila.top().second;
            fila.pop();

            // Marca o vértice como visitado
            visitado[u] = true;

            // Para cada vizinho do vértice atual
            for (int i = 0; i < adjacencyList[u].size(); ++i) {
                int v = adjacencyList[u][i];
                int pesoAtual = peso[u][i]; // Corrigido aqui

                // Se a distância até o vizinho passando pelo vértice atual é menor do que a distância atual armazenada para o vizinho
                if (!visitado[v] && distancia[u] + pesoAtual < distancia[v]) {
                    distancia[v] = distancia[u] + pesoAtual; // Atualiza a distância mínima
                    pai[v] = u; // Atualiza o pai do vizinho
                    fila.push({distancia[v], v}); // Adiciona o vizinho à fila
                }
            }
        }

        // Constrói o caminho mínimo do destino para a origem
        vector<int> caminho;
        int verticeAtual = destino;
        while (verticeAtual != -1) {
            caminho.push_back(verticeAtual);
            verticeAtual = pai[verticeAtual];
        }
        reverse(caminho.begin(), caminho.end()); // Inverte o caminho para obter a ordem correta

        return caminho;
    }

};
