#include <dg.hpp>
#include <iostream>
using namespace dg;

void printVec(const auto& v) {
    if (v.empty()) std::cout << std::endl;
    else {
        for (auto i = v.begin(); i + 1 != v.end(); ++i) std::cout << *i << ' ';
        std::cout << *(v.end() - 1) << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // ===== Edge =====
    std::cout << "===== Edge =====" << std::endl;
    Edge<int> e1(1);
    Edge<void> e2;

    // ===== DGraphBase =====
    std::cout << "===== DGraphBase =====" << std::endl;
    DGraphBase<int> dgb1;
    std::cout << dgb1.hasNode("foo") << " " << dgb1.hasEdge("foo", "bar") << std::endl;
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    try {
        auto x = dgb1.edge("foo", "bar"); // This edge does not exist.
    } catch (const node_not_exists& e) { std::cout << "Catch exception: " << e.what() << std::endl; }
    dgb1.insertNode("empty-node");
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    dgb1.insertNode("empty-node"); // do it again
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    dgb1.insertNode("empty-node-2");
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    dgb1.insertNode("root", std::map<std::string, int>({ { "n1", 1 }, { "n2", 2 }, { "empty-node", -1 } }));
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    printVec(dgb1.nodesID());
    dgb1.removeNode("n2");
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    printVec(dgb1.nodesID());
    dgb1.removeNode("root");
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    printVec(dgb1.nodesID());
    dgb1.insertEdge("n1", "n2", 12);
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;

    std::cout << ">> dgb2:" << std::endl;
    DGraphBase dgb2;
    dgb2.insertEdge("n1", "n2");
    std::cout << dgb2.numNodes() << " " << dgb2.numEdges() << std::endl;

    // ===== DGraph =====
    std::cout << "===== DGraph =====" << std::endl;
    DGraph<double, char> dg1;
    std::cout << dg1.hasNode("foo") << " " << dg1.hasEdge("foo", "bar") << std::endl;

    return 0;
}
