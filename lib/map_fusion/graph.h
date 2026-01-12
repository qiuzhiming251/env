#include <iostream>
#include <list>
#include <unordered_map>

namespace {
constexpr int      kMaxLoopNumber                    = 50;
}  // namespace

template<typename T>
class Graph {
public:
    struct Node {
        T id;
        std::list<Node*> successors = {};
        std::list<Node*> predecessors = {};

        std::unique_ptr<Node> left = nullptr;
        std::unique_ptr<Node> right = nullptr;

        Node(T vertex_id) : id(vertex_id) {}
    };

    void AddNode(T id) {
        if (nodes_.find(id) == nodes_.end()) {
            nodes_[id] = std::make_unique<Node>(id);
        }
    }

    void AddLeftNode(Node* node, T id) {
        if (node == nullptr) {
            return;
        }
        node->left = std::make_unique<Node>(id);
    }

    void AddRightNode(Node* node, T id) {
        if (node == nullptr) {
            return;
        }
        node->right = std::make_unique<Node>(id);
    }

    void AddEdge(T src_id, T dest_id) {
        AddNode(src_id);
        AddNode(dest_id);

        Node* src = nodes_[src_id].get();
        Node* dest = nodes_[dest_id].get();

        src->successors.push_back(dest);
        dest->predecessors.push_back(src);
    }

    void RemoveNode(T id) {
        if (nodes_.find(id) != nodes_.end()) {
            Node* node = nodes_[id].get();

            for (auto successor : node->successors) {
                successor->predecessors.remove(node);
            }
            for (auto predecessor : node->predecessors) {
                predecessor->successors.remove(node);
            }

            nodes_.erase(id);
            // No need to delete manually, unique_ptr will handle it
        }
    }

    void RemoveEdge(T src_id, T dest_id) {
        if (nodes_.find(src_id) != nodes_.end() && nodes_.find(dest_id) != nodes_.end()) {
            Node* src = nodes_[src_id].get();
            Node* dest = nodes_[dest_id].get();

            src->successors.remove(dest);
            dest->predecessors.remove(src);
        }
    }

    void PrintGraph() {
        for (const auto& pair : nodes_) {
            Node* node = pair.second.get();
            AINFO << "Node " << node->id << ": ";

            AINFO << "Successors: ";
            for (auto successor : node->successors) {
                AINFO << successor->id << " -> ";
            }
            AINFO << "null";

            AINFO << " | Predecessors: ";
            for (auto predecessor : node->predecessors) {
                AINFO << predecessor->id << " -> ";
            }
            AINFO << "null";
        }
    }

    void TraverseSuccessors(Node* node) {
        if (node == nullptr) {
            return;
        }

        AINFO << "Successors of Node " << node->id << ": ";
        for (auto successor : node->successors) {
            AINFO << successor->id << " -> ";
        }
        AINFO << "null";
    }

    void TraverseSuccessors(Node* node, std::set<T>& index, int &depth) {
        if (node == nullptr) {
            return;
        }
        depth++;
        if (depth > kMaxLoopNumber) {
            AERROR << "TraverseSuccessors depth > kMaxLoopNumber:   " << depth;
            return;
        }
        for (const auto successor : node->successors) {
            index.insert(successor->id);
            TraverseSuccessors(successor, index, depth);
        }
    }

    void TraversePredecessors(Node* node) {
        if (node == nullptr) {
            return;
        }

        AINFO << "Predecessors of Node " << node->id << ": ";
        for (const auto predecessor : node->predecessors) {
            AINFO << predecessor->id << " -> ";
        }
        AINFO << "null";
    }

    void TraversePredecessors(Node* node, std::set<T>& index, int &depth) {
        if (node == nullptr) {
            return;
        }
        depth++;
        if (depth > kMaxLoopNumber) {
            AERROR << "TraversePredecessors depth > kMaxLoopNumber:   " << depth;
            return;
        }
        for (const auto predecessor : node->predecessors) {
            index.insert(predecessor->id);
            TraversePredecessors(predecessor, index, depth);
        }
    }

    Node* GetNode(T id) {
        if (nodes_.find(id) != nodes_.end()) {
            return nodes_[id].get();
        }
        return nullptr;
    }

    ~Graph() {
        // The unique_ptr will automatically delete the nodes when the map is cleared
        nodes_.clear();
    }

private:
    std::unordered_map<T, std::unique_ptr<Node>> nodes_;
};