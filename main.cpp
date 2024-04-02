#include "graph.cpp"
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

// Implementação da classe Graph com todas as funções necessárias (adapte
// conforme necessário)

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
    cout << "0. Sair\n";

    cout << "\nEscolha uma opção: ";
    cin >> opcao;

    switch (opcao) {
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
