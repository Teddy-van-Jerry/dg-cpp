/**
 * @file test.cpp
 * @author Wuqiong Zhao (me@wqzhao.org)
 * @brief Example use of dg library.
 * @version 0.1.0
 * @date 2023-04-16
 *
 * @copyright Copyright (c) 2023 Wuqiong Zhao (Teddy van Jerry)
 *
 */

#include <dg.hpp>
#include <iostream>
using namespace dg;

void printVec(const auto& v, std::string sep = " ") {
    if (v.empty()) std::cout << std::endl;
    else {
        for (auto i = v.begin(); i + 1 != v.end(); ++i) std::cout << *i << sep;
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
    std::cout << "(std::function) weight + n: "
              << dgb1.edge(std::string("root"), std::string("n2"),
                           std::function<int(int, int)>([](int weight, int n) { return weight + n; }), 2)
              << std::endl;
    std::cout << "(lambda expr) weight + 1: " << dgb1.edge<int>(std::string("root"), std::string("n2"), [](int weight) {
        return weight + 1;
    }) << std::endl;
    dgb1.maxWeightPath("n1", "n2");
    dgb1.removeNode("n2");
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    printVec(dgb1.nodesID());
    dgb1.removeNode("root");
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    std::cout << dgb1.isConnected("root", "empty-node") << std::endl;
    printVec(dgb1.nodesID());
    dgb1.insertEdge("n1", "n2", 12);
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;

    std::cout << ">> dgb2:" << std::endl;
    DGraphBase dgb2;
    dgb2.insertEdge("n1", "n2");
    std::cout << dgb2.numNodes() << " " << dgb2.numEdges() << std::endl;
    std::cout << dgb2.isConnected("n1", "n2") << std::endl;

    std::cout << ">> dgb3:" << std::endl;
    DGraphBase<double> dgb3;
    dgb3.insertEdge("A", "B", 1.2);
    dgb3.insertEdge("A", "C", 0.5);
    dgb3.insertEdge("C", "B", 1.5);
    auto [w, n] = dgb3.maxWeightPath("A", "B");
    std::cout << "max weight: " << w << std::endl;
    std::cout << "path: ";
    printVec(n, " -> ");

    std::cout << ">> dgb4:" << std::endl;
    decltype(dgb1) dgb4;
    dgb4.insertEdge("A", "B", 1);
    dgb4.insertEdge("A", "C", 2);
    dgb4.insertEdge("n1", "A", 2);
    dgb4.merge(dgb1);
    std::cout << dgb4.numNodes() << " " << dgb4.numEdges() << std::endl;
    printVec(dgb4.nodesID());

    // ===== DGraph =====
    std::cout << "===== DGraph =====" << std::endl;
    DGraph<double, char> dg1;
    std::cout << dg1.hasNode("foo") << " " << dg1.hasEdge("foo", "bar") << std::endl;

    struct DataT {
        double data = 0;
        int foo     = 0;
    };
    std::cout << ">> dg2:" << std::endl;
    DGraphBase<DataT> dg2;
    dg2.insertEdge("A", "B", DataT{ 1.2, 1 });
    dg2.insertEdge("A", "C", DataT{ 0.5, 1 });
    dg2.insertEdge("C", "B", DataT{ 1.5, 1 });
    auto [w2, n2] = dg2.minWeightPath<double>("A", "B", [](DataT x) { return x.data; });
    std::cout << "min weight: " << w2 << std::endl;
    std::cout << "path: ";
    printVec(n2, " -> ");

    std::cout << ">> dg3:" << std::endl;
    DGraph<double, int> dg3;
    dg3.insertNode("A", 0.8);
    dg3.insertEdge("A", "B", 1);
    std::cout << dg3.numNodes() << " " << dg3.numEdges() << std::endl;
    std::cout << std::boolalpha << dg3.strictCheck() << std::endl;
    dg3.insertNode("B", 1.2);
    std::cout << dg3.numNodes() << " " << dg3.numEdges() << std::endl;
    std::cout << std::boolalpha << dg3.strictCheck() << std::endl;

    return 0;
}
