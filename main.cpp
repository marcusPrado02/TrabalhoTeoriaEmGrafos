#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include "graph.cpp"

using namespace std;

// Implementação da classe Graph com todas as funções necessárias (adapte conforme necessário)

int main() {
    int numVertices;
    bool directed;
    int choiceRepresentation;

    cout << "Digite o número de vértices: ";
    cin >> numVertices;

    cout << "O grafo é direcionado? (1 para sim, 0 para não): ";
    cin >> directed;

    cout << "Escolha a representação do grafo: \n";
    cout << "1. Lista de Adjacência\n";
    cout << "2. Matriz de Adjacência\n";
    cin >> choiceRepresentation;

    Graph graph(numVertices, directed);

    int opcao;
    do {
        cout << "\nMenu:\n";
        cout << "1. Adicionar aresta\n";
        cout << "2. Remover aresta\n";
        cout << "3. Vizinhos de um vértice\n";
        cout << "4. Sucessores de um vértice (grafo direcionado)\n";
        cout << "5. Predecessores de um vértice (grafo direcionado)\n";
        cout << "6. Grau de um vértice\n";
        cout << "7. Verificar se o grafo é simples\n";
        cout << "8. Verificar se o grafo é regular\n";
        cout << "9. Verificar se o grafo é completo\n";
        cout << "10. Verificar se o grafo é bipartido\n";
        cout << "11. Visualizar grafo \n";
        cout << "12. Algoritmo de Kruskal (Lista de Adjacência)\n";
        cout << "13. Algoritmo de Kruskal (Matriz de Adjacência)\n";
        cout << "14. Algoritmo de Prim (Lista de Adjacência)\n";
        cout << "15. Algoritmo de Prim (Matriz de Adjacência)\n";
        cout << "16. Ordenação Topológica (Lista de Adjacência)\n";
        cout << "17. Adicionar arestas com peso em sequencia\n"; // Opção adicionada
        cout << "18. Adicionar aresta com peso\n";
        cout << "19. Remover aresta com peso\n";
        cout << "20. Adicionar arestas a partir de uma lista com peso\n";
        cout << "0. Sair\n";


        cout << "\nEscolha uma opção: ";
        cin >> opcao;

        switch(opcao) {
            case 1: {
                int src, dest;
                cout << "Digite o vértice de origem: ";
                cin >> src;
                cout << "Digite o vértice de destino: ";
                cin >> dest;
                if (choiceRepresentation == 1) {
                    graph.addEdgeList(src, dest);
                } else {
                    graph.addEdgeMatrix(src, dest);
                }
                cout << "Aresta adicionada com sucesso.\n";
                break;
            }
            case 2: {
                int src, dest;
                cout << "Digite o vértice de origem: ";
                cin >> src;
                cout << "Digite o vértice de destino: ";
                cin >> dest;
                if (choiceRepresentation == 1) {
                    graph.removeEdgeList(src, dest);
                } else {
                    graph.removeEdgeMatrix(src, dest);
                }
                cout << "Aresta removida com sucesso.\n";
                break;
            }
            case 3: {
                int vertex;
                cout << "Digite o vértice para verificar a vizinhança: ";
                cin >> vertex;
                vector<int> neighbors;
                if (choiceRepresentation == 1) {
                    neighbors = graph.getNeighborsList(vertex);
                } else {
                    neighbors = graph.getNeighborsMatrix(vertex);
                }
                cout << "Vizinhos do vértice " << vertex << ": ";
                for (int neighbor : neighbors) {
                    cout << neighbor << " ";
                }
                cout << endl;
                break;
            }
            case 4: {
                int vertex;
                cout << "Digite o vértice para verificar os sucessores: ";
                cin >> vertex;
                vector<int> successors;
                if (choiceRepresentation == 1) {
                    successors = graph.getSuccessorsList(vertex);
                } else {
                    successors = graph.getSuccessorsMatrix(vertex);
                }
                cout << "Sucessores do vértice " << vertex << ": ";
                for (int successor : successors) {
                    cout << successor << " ";
                }
                cout << endl;
                break;
            }
            case 5: {
                int vertex;
                cout << "Digite o vértice para verificar os predecessores: ";
                cin >> vertex;
                vector<int> predecessors;
                if (choiceRepresentation == 1) {
                    predecessors = graph.getPredecessorsList(vertex);
                } else {
                    predecessors = graph.getPredecessorsMatrix(vertex);
                }
                cout << "Predecessores do vértice " << vertex << ": ";
                for (int predecessor : predecessors) {
                    cout << predecessor << " ";
                }
                cout << endl;
                break;
            }
            case 6: {
                int vertex;
                cout << "Digite o vértice para verificar o grau: ";
                cin >> vertex;
                int degree;
                if (choiceRepresentation == 1) {
                    degree = graph.getDegreeList(vertex);
                } else {
                    degree = graph.getDegreeMatrix(vertex);
                }
                cout << "Grau do vértice " << vertex << ": " << degree << endl;
                break;
            }
            case 7: {
                bool simples;
                if (choiceRepresentation == 1) {
                    simples = graph.isSimpleList();
                } else {
                    simples = graph.isSimpleMatrix();
                }
                if (simples) {
                    cout << "O grafo é simples." << endl;
                } else {
                    cout << "O grafo não é simples." << endl;
                }
                break;
            }
            case 8: {
                bool regular;
                if (choiceRepresentation == 1) {
                    regular = graph.isRegularList();
                } else {
                    regular = graph.isRegularMatrix();
                }
                if (regular) {
                    cout << "O grafo é regular." << endl;
                } else {
                    cout << "O grafo não é regular." << endl;
                }
                break;
            }
            case 9: {
                bool completo;
                if (choiceRepresentation == 1) {
                    completo = graph.isCompleteList();
                } else {
                    completo = graph.isCompleteMatrix();
                }
                if (completo) {
                    cout << "O grafo é completo." << endl;
                } else {
                    cout << "O grafo não é completo." << endl;
                }
                break;
            }
            case 10: {
                bool bipartido;
                if (choiceRepresentation == 1) {
                    bipartido = graph.isBipartiteList();
                } else {
                    bipartido = graph.isBipartiteMatrix();
                }
                if (bipartido) {
                    cout << "O grafo é bipartido." << endl;
                } else {
                    cout << "O grafo não é bipartido." << endl;
                }
                break;
            }
            case 11: {
                if (choiceRepresentation == 1) {
                    graph.printGraphList();
                } else {
                    graph.printGraphMatrix();
                }
                break;
            }
            case 12: {
                if (choiceRepresentation == 1) {
                    vector<Aresta> agmKruskalList = graph.AGMKruskalList();
                    // Exibir a Árvore Geradora Mínima (AGM) obtida pelo algoritmo de Kruskal
                    cout << "Árvore Geradora Mínima (Kruskal) - Lista de Adjacência:\n";
                    for (const Aresta& aresta : agmKruskalList) {
                        cout << "(" << aresta.origem << ", " << aresta.destino << ") - Peso: " << aresta.peso << endl;
                    }
                } else {
                    cout << "Essa opção só está disponível para a representação por lista de adjacência.\n";
                }
                break;
            }
            case 13: {
                if (choiceRepresentation == 2) {
                    vector<Aresta> agmKruskalMatrix = graph.AGMKruskalMatrix();
                    // Exibir a Árvore Geradora Mínima (AGM) obtida pelo algoritmo de Kruskal
                    cout << "Árvore Geradora Mínima (Kruskal) - Matriz de Adjacência:\n";
                    for (const Aresta& aresta : agmKruskalMatrix) {
                        cout << "(" << aresta.origem << ", " << aresta.destino << ") - Peso: " << aresta.peso << endl;
                    }
                } else {
                    cout << "Essa opção só está disponível para a representação por matriz de adjacência.\n";
                }
                break;
            }
            case 14: {
                if (choiceRepresentation == 1) {
                    vector<int> agmPrimList = graph.AGMPrimList();
                    // Exibir a Árvore Geradora Mínima (AGM) obtida pelo algoritmo de Prim
                    cout << "Árvore Geradora Mínima (Prim) - Lista de Adjacência:\n";
                    for (int vertex : agmPrimList) {
                        cout << vertex << " ";
                    }
                    cout << endl;
                } else {
                    cout << "Essa opção só está disponível para a representação por lista de adjacência.\n";
                }
                break;
            }
            case 15: {
                if (choiceRepresentation == 2) {
                    vector<int> agmPrimMatrix = graph.AGMPrimMatrix();
                    // Exibir a Árvore Geradora Mínima (AGM) obtida pelo algoritmo de Prim
                    cout << "Árvore Geradora Mínima (Prim) - Matriz de Adjacência:\n";
                    for (int vertex : agmPrimMatrix) {
                        cout << vertex << " ";
                    }
                    cout << endl;
                } else {
                    cout << "Essa opção só está disponível para a representação por matriz de adjacência.\n";
                }
                break;
            }
            case 16: {
                if (choiceRepresentation == 1) {
                    vector<int> ordenacaoTopologica = graph.ordenacaoTopologicaList();
                    // Exibir a Ordenação Topológica obtida
                    cout << "Ordenação Topológica - Lista de Adjacência:\n";
                    for (int vertex : ordenacaoTopologica) {
                        cout << vertex << " ";
                    }
                    cout << endl;
                } else {
                    cout << "Essa opção só está disponível para a representação por lista de adjacência.\n";
                }
                break;
            }
            case 17: {
                int numArestas;
                cout << "Digite o número de arestas: ";
                cin >> numArestas;
                vector<pair<pair<int, int>, int>> arestas;
                for (int i = 0; i < numArestas; ++i) {
                    int src, dest, peso;
                    cout << "Aresta " << i + 1 << ":\n";
                    cout << "Digite o vértice de origem: ";
                    cin >> src;
                    cout << "Digite o vértice de destino: ";
                    cin >> dest;
                    cout << "Digite o peso da aresta: ";
                    cin >> peso;
                    arestas.push_back({{src, dest}, peso});
                }
                graph.addEdgesFromList(arestas);
                cout << "Arestas adicionadas com sucesso.\n";
                break;
            }
            case 18: {
                int src, dest, weight;
                cout << "Digite o vértice de origem: ";
                cin >> src;
                cout << "Digite o vértice de destino: ";
                cin >> dest;
                cout << "Digite o peso da aresta: ";
                cin >> weight;
                if (choiceRepresentation == 1) {
                    graph.addEdgeListWithWeight(src, dest, weight);
                } else {
                    graph.addEdgeMatrixWithWeight(src, dest, weight);
                }
                cout << "Aresta com peso adicionada com sucesso.\n";
                break;
            }
            case 19: {
                int src, dest, weight;
                cout << "Digite o vértice de origem: ";
                cin >> src;
                cout << "Digite o vértice de destino: ";
                cin >> dest;
                if (choiceRepresentation == 1) {
                    graph.removeEdgeListWithWeight(src, dest);
                } else {
                    graph.removeEdgeMatrixWithWeight(src, dest);
                }
                cout << "Aresta com peso removida com sucesso.\n";
                break;
            }
            case 20: {
                vector<pair<pair<int, int>, int>> novasArestas;

                cout << "Insira as novas arestas no formato ((origem, destino), peso):\n";
                cout << "Exemplo: (<5,4>, 2), (<3,2>, 1),(<1,3>, 5)\n";

                string entrada;
                getline(cin >> ws, entrada);

                // Removendo os caracteres desnecessários da entrada
                entrada.erase(remove(entrada.begin(), entrada.end(), ' '), entrada.end());
                entrada.erase(remove(entrada.begin(), entrada.end(), '<'), entrada.end());
                entrada.erase(remove(entrada.begin(), entrada.end(), '>'), entrada.end());
                entrada.erase(remove(entrada.begin(), entrada.end(), '('), entrada.end());
                entrada.erase(remove(entrada.begin(), entrada.end(), ')'), entrada.end());

                // Dividindo a entrada em substrings separadas por ','
                vector<string> substrings;
                stringstream ss(entrada);
                string substring;
                while (getline(ss, substring, ',')) {
                    substrings.push_back(substring);
                }

                // Processando cada substring para extrair origem, destino e peso
                for (auto sub : substrings) {
                    int origem, destino, peso;
                    sscanf(sub.c_str(), "%d%d%d", &origem, &destino, &peso);
                    novasArestas.push_back(make_pair(make_pair(origem, destino), peso));
                }

                // Adicionando as arestas ao grafo
                for (auto aresta : novasArestas) {
                    int src = aresta.first.first;
                    int dest = aresta.first.second;
                    int peso = aresta.second;

                    if (choiceRepresentation == 1) {
                        graph.addEdgeListWithWeight(src, dest, peso);
                    } else {
                        graph.addEdgeMatrixWithWeight(src, dest, peso);
                    }
                }

                cout << "Arestas adicionadas com sucesso.\n";
                break;
            }
            case 0: {
                cout << "Encerrando o programa.\n";
                break;
            }
            default: {
                cout << "Opção inválida. Tente novamente.\n";
            }
        }
    } while (opcao != 0);

    return 0;
}
