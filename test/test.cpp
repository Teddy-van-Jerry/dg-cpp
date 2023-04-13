#include <dg.hpp>
#include <iostream>
using namespace dg;

int main(int argc, char* argv[]) {
    // ===== Edge ======
    Edge<int> e1(1);
    Edge<void> e2;

    // ===== DGraphBase ======
    DGraphBase<int> dgb1;
    std::cout << dgb1.hasNode("foo") << " " << dgb1.hasEdge("foo", "bar") << std::endl;
    std::cout << dgb1.numNodes() << " " << dgb1.numEdges() << std::endl;
    try {
        auto x = dgb1.edge("foo", "bar"); // This edge does not exist.
    } catch (const node_not_exists& e) { std::cout << "Catch exception: " << e.what() << std::endl; }

    // ===== DGraph ======
    DGraph<double, char> dg1;
    std::cout << dg1.hasNode("foo") << " " << dg1.hasEdge("foo", "bar") << std::endl;

    return 0;
}
