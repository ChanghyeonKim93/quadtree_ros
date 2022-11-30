#ifndef _QUADTREE_ARRAY_H_
#define _QUADTREE_ARRAY_H_

#include <iostream>
#include <vector>
#include <cmath>

#include "simple_stack.h"
#include "macro_quadtree.h"

#define STATE_INACTIVATED 0b0000 // 0
#define STATE_ACTIVATED   0b0001 // 1 (0b0001)
#define STATE_BRANCH      0b0011 // 2 (0b0011)
#define STATE_LEAF        0b0101 // 4 (0b0101)

#define IS_INACTIVATED(nd) ( (nd).state == 0b0000         ) 
#define IS_ACTIVATED(nd)   ( (nd).state  & STATE_ACTIVATED) // 브랜치이거나 리프
#define IS_BRANCH(nd)      ( (nd).state == STATE_BRANCH   ) // 브랜치 노드 )
#define IS_LEAF(nd)        ( (nd).state == STATE_LEAF     ) // 리프 노드

#define MAKE_INACTIVATE(nd)( (nd).state = STATE_INACTIVATED) // 아무것도 아니게 만듦
#define MAKE_ACTIVATE(nd)  ( (nd).state = STATE_ACTIVATED  ) // 브랜치이거나 리프로 만듦
#define MAKE_BRANCH(nd)    ( (nd).state = STATE_BRANCH     ) // 브랜치 노드로 만듦
#define MAKE_LEAF(nd)      ( (nd).state = STATE_LEAF       ) // 리프 노드로 만듦


namespace ArrayBased
{
    typedef uint8_t Flag; // 1 bytes

    class Quadtree
    {
    private:
        template <typename T>
        struct QuadPoint // sizeof(T)*2 bytes
        { 
            T x;
            T y;

            // QuadPoint constructor
            QuadPoint() : x(0),y(0) { };
            QuadPoint(T x_, T y_) : x(x_), y(y_) { };
            QuadPoint(const QuadPoint& pos) : x(pos.x), y(pos.y) { }; // copy constructor

            QuadPoint& operator=(const QuadPoint& c){ // copy insert constructor
                x = c.x; y = c.y;
                return *this;
            };
            QuadPoint& operator+=(const QuadPoint& c){
                x += c.x; y += c.y;
                return *this;
            };
            QuadPoint& operator-=(const QuadPoint& c){
                x -= c.x; y -= c.y;
                return *this;
            };

            QuadPoint operator+(const QuadPoint& c){
                return QuadPoint(x + c.x, y + c.y);
            };
            QuadPoint operator-(const QuadPoint& c){
                return QuadPoint(x - c.x, y - c.y);
            };

            friend std::ostream& operator<<(std::ostream& os, const QuadPoint& c){
                os << "["<< c.x <<"," << c.y <<"]";
                return os;
            };
        };

        template <typename T>
        struct QuadRect // sizeof(T)*4 bytes
        {
            QuadPoint<T> tl; // Top left 
            QuadPoint<T> br; // bottom right
            QuadRect() : tl(0,0), br(0,0) { };
            QuadRect(const QuadPoint<T>& tl_, const QuadPoint<T>& br_) : tl(tl_), br(br_) { };

            inline QuadPoint<T> getCenter() {
                return QuadPoint<T>((double)(tl.x + br.x)*0.5, (double)(tl.y + br.y)*0.5);
            };
            friend std::ostream& operator<<(std::ostream& os, const QuadRect& c){
                os << "tl:["<< c.tl.x <<"," << c.tl.y <<"],br:[" << c.br.x <<","<<c.br.y<<"]";
                return os;
            };
        };

        typedef QuadRect<uint16_t> QuadRect_u;
        typedef QuadRect<float>    QuadRect_f;

        struct QuadNode // 10 bytes (actually 10 bytes)
        {
            // AABB (Axis-alinged Bounding box) is not contained, but just 
            // they are just calculated on the fly if needed.
            // This is more efficient because reducing the memory use of a node can 
            // proportionally reduce cache misses when you traverse the tree.
            QuadRect_u rect; // 2 * 4  = 8 bytes (padding size = 4 bytes)

            // If -2, not initialized (no children.) 
            // else if -1,  branch node. (children exist.)
            // else, leaf node.
            Flag   state; // 1 byte  
            int8_t depth; // 1 byte

            QuadNode() : state(STATE_INACTIVATED), depth(-1) {};
            friend std::ostream& operator<<(std::ostream& os, const QuadNode& c){
                os << "count:[" << c.state <<"]";
                return os;
            };
        }; 

        struct QuadElement{ // 12 bytes (actually 16 bytes)
            float  x_nom;   // 4 bytes
            float  y_nom;   // 4 bytes
            uint32_t id_data; // 4 bytes

            QuadElement() : id_data(0),x_nom(-1.0f),y_nom(-1.0f) { };
            QuadElement(float x_nom_, float y_nom_, int id_data_)
            : id_data(id_data_), x_nom(x_nom_), y_nom(y_nom_) { };
        };

        struct QuadNodeElements{ // 8 bytes
            // Stores the ID for the element (can be used to refer to external data).
            std::vector<uint32_t> elem_ids; // 8 bytes

            QuadNodeElements()  { elem_ids.resize(0); };
            inline void reset() { elem_ids.resize(0); };
            inline int getNumElem() const { return elem_ids.size(); };
        };

        struct InsertData{ // 12 bytes (actually 16 bytes)
            float x_nom;   // 4 bytes
            float y_nom;   // 4 bytes
            uint32_t    id_elem; // 4 bytes

            InsertData() : x_nom(-1.0f), y_nom(-1.0f), id_elem(0){ };
            void setData(float x_nom_, float y_nom_, uint32_t id_elem_){
                x_nom   = x_nom_; 
                y_nom   = y_nom_;
                id_elem = id_elem_;
            };
        };

        struct QuaryData{
            float x; // 4 bytes
            float y; // 4 bytes
            float x_nom; // 4 bytes
            float y_nom; // 4 bytes

            uint32_t id_node_cached; // 4 bytes

            uint32_t id_data_matched; // 4 bytes
            uint32_t id_node_matched; // 4 bytes
            uint32_t id_elem_matched; // 4 bytes

            float min_dist2_;
            float min_dist_;
        };

        struct QuadParams {
            // Distance parameter
            float approx_rate; // 0.3~1.0;
            
            // Searching parameter
            Flag flag_adj_search_only;
        };

    public:
        Quadtree(float x_min, float x_max, float y_min, float y_max, 
            uint32_t max_depth, uint32_t max_elem_per_leaf,
            float approx_rate = 1.0, Flag flag_adj = false);
        ~Quadtree();

        void insert(float x, float y, int id_data);
        void NNSearch(float x, float y,
            uint32_t& id_data_matched, uint32_t& id_node_matched);
        void cachedNNSearch(float x, float y, int id_node_cached, 
            uint32_t& id_data_matched, uint32_t& id_node_matched);
        
        void NNSearchDebug(float x, float y, 
            uint32_t& id_data_matched, uint32_t& id_node_matched, uint32_t& n_access);
        void cachedNNSearchDebug(float x, float y, int id_node_cached, 
            uint32_t& id_data_matched, uint32_t& id_node_matched, uint32_t& n_access);

    // Related to generate tree.
    private:
        void insertPrivate(uint32_t id_node, Flag depth);
        void insertPrivateStack();

        inline void makeChildrenLeaves(uint32_t id_child, const QuadRect_u& rect);

        inline void addDataToNode(uint32_t id_node, uint32_t id_elem);
        inline int getNumElemOfNode(uint32_t id_node);

        inline void makeBranch(uint32_t id_node);

    // Related to NN search (private)
    private:
        void nearestNeighborSearchPrivate(); // return id_data matched.
        
        inline bool BWBTest( float x, float y, const QuadRect_u& rect, float radius); // Ball Within Bound
        inline bool BOBTest( float x, float y, const QuadRect_u& rect, float radius); // Ball Overlap Bound
        bool findNearestElem(float x, float y, const QuadNodeElements& elems);

    // Related to cached NN search (private)
    private:
        void cachedNearestNeighborSearchPrivate();

    private:
        QuadParams params_;

    private:
        // Stores all the elements in the quadtree.
        std::vector<QuadElement>  all_elems_;
        std::vector<QuadNodeElements> node_elements;

    private:
        // Stores all the nodes in the quadtree. The second node in this
        // sequence is always the root.
        // index 0 is not used.
        std::vector<QuadNode> nodes;
        uint32_t n_nodes_;
        uint32_t n_node_activated_;
        // |  1  |  2  |  3  |  4  |  5  |  ...
        // | root| tl0 | bl0 | tr0 | br0 |  ...
        // Z-order
        //  id_node * 4  - 2 -> first child id (child 0~3)
        // (id_node + 2) / 4 -> parent id
        // (id_node + 2) % 4 -> quadrant index of this node

        float x_normalizer_;
        float y_normalizer_;

    public:
        uint32_t getNumNodes();
        uint32_t getNumNodesActivated();


    private: 
        // quadtree range (in real scale)
        float x_range_[2]; 
        float y_range_[2];

        uint32_t max_depth_; // Quadtree maximum depth
        uint32_t max_elem_per_leaf_; // The maximum number of elements in the leaf. If maxdepth leaf, no limit to store elements.

    // For insert a data
    private:
        InsertData insert_data_;
        inline void resetInsertData();

    // For nearest neighbor search algorithm
    private:
        QuaryData query_data_;
        SimpleStack<uint32_t> simple_stack_;

        inline void resetNNParameters();
        inline void resetQueryData();
    };
};


#endif