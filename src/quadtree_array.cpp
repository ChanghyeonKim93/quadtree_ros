#include "quadtree_array.h"

namespace ArrayBased
{
    Quadtree::Quadtree(
        float x_min, float x_max,
        float y_min, float y_max,
        uint32_t max_depth, uint32_t max_elem_per_leaf,
        float approx_rate, uint8_t flag_adj)
    : 
    max_depth_(max_depth),
    x_range_{x_min, x_max},
    y_range_{y_min, y_max},
    n_node_activated_(1),
    max_elem_per_leaf_(max_elem_per_leaf)
    {
        std::cout << "sizeof(QuadNode): "<< sizeof(Quadtree::QuadNode) << " Bytes" << std::endl;

        // Make nodes
        n_nodes_ = (std::pow(4,max_depth_+1)-1)/(4-1); // (4^(D+1)-1) / (4 - 1)
        nodes_.resize(n_nodes_+1);
        
        // Initialize a root node.
        MAKE_LEAF(nodes_[1]);

        // Initialize node elements.
        node_elements.resize(nodes_.size());

        // Root size.
        uint16_t br_x = 1 << 13; // 2^13-1
        uint16_t br_y = 1 << 13; // 2^13-1
        
        nodes_[1].rect.tl = QuadPoint<uint16_t>(0, 0);
        nodes_[1].rect.br = QuadPoint<uint16_t>(br_x, br_y);

        // Calculate normalizers 
        x_normalizer_ = (float)nodes_[1].rect.br.x/(x_range_[1] - x_range_[0]);
        y_normalizer_ = (float)nodes_[1].rect.br.y/(y_range_[1] - y_range_[0]);

        if(x_normalizer_ > y_normalizer_) 
            y_normalizer_ = x_normalizer_;
        else 
            x_normalizer_ = y_normalizer_;

        // Parameter setting
        params_.approx_rate = approx_rate;
        if( params_.approx_rate > 1.0f ) params_.approx_rate = 1.0f;
        if( params_.approx_rate < 0.5f ) params_.approx_rate = 0.5f;
        params_.approx_rate = params_.approx_rate*params_.approx_rate;

        params_.flag_adj_search_only = flag_adj;
    };

    Quadtree::~Quadtree() { 
        std::cout << "Quadtree is deleted." << std::endl;
    };

    void Quadtree::insert(float x, float y, int id_data)
    {
        if( x > x_range_[0] && x < x_range_[1]
         && y > y_range_[0] && y < y_range_[1] )
        {
            // normalize input point coordinates
            float x_nom = x*x_normalizer_; 
            float y_nom = y*y_normalizer_;

            // Add the input data as an QuadElement object
            all_elems_.push_back( QuadElement(x_nom, y_nom, (uint32_t)id_data) );
            
            // Initialize the 'insert_data_' 
            // This is to avoid recursive copies of recursive call of function inputs) 
            insert_data_.setData( x_nom, y_nom, all_elems_.size()-1 );

            // Insert query_data_ into the quadtree.
            // this->insertPrivate(1, 0); // Recursion-based approach (DFS)
            this->insertPrivateStack(); // Stack-based approach (DFS), faster than Recursion-based one 1.2 times.
        }
        else 
            throw std::runtime_error("Input {x, y} coordinate is out of the quadtree range.");
    };

    void Quadtree::insertPrivate(uint32_t id_node, uint8_t depth)
    {
        QuadNode&              nd = nodes_[id_node]; // current node.
        QuadNodeElements& ndelems = node_elements[id_node];
        nd.depth = depth;
        
        const float& x_nom = insert_data_.x_nom;
        const float& y_nom = insert_data_.y_nom;

        if(IS_BRANCH(nd))
        {
            // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
            Flag flag_sn, flag_ew;
            FIND_QUADRANT(x_nom, y_nom, nd.rect, flag_sn, flag_ew);
            uint32_t id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
            
            insertPrivate(id_child, depth + 1); // Go to the selected child
        }
        else if( IS_LEAF(nd) ) 
        {
            // This is a leaf node.
            addDataToNode(id_node, insert_data_.id_elem);

            if(depth < max_depth_){ // nonmax depth.
                int n_elem = getNumElemOfNode(id_node);
                if(n_elem > max_elem_per_leaf_){ // too much data. divide.
                    // Make all child to leaf (with no element)
                    uint32_t id_child = GET_FIRST_CHILD_ID(id_node);
                    makeChildrenLeaves(id_child, nd.rect);
                    n_node_activated_ += 4;

                    // Do divide.
                    for(const uint32_t& id_elem_tmp : ndelems.elem_ids) 
                    {
                        Flag flag_sn, flag_ew;
                        QuadElement& elem_tmp = all_elems_[id_elem_tmp];
                        FIND_QUADRANT(elem_tmp.x_nom, elem_tmp.y_nom, nd.rect, flag_sn, flag_ew);
                        uint32_t id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
                        addDataToNode(id_child, id_elem_tmp);
                    }

                    //make this node as a branch
                    makeBranch(id_node);
                }
            }
        }
    };


    void Quadtree::insertPrivateStack()
    {
        const float& x_nom = insert_data_.x_nom;
        const float& y_nom = insert_data_.y_nom;
        uint32_t id_node = 1;
        nodes_[1].depth = 0;

        while(true){
            QuadNode&              nd = nodes_[id_node]; // current node.
            QuadNodeElements& ndelems = node_elements[id_node];

            int depth = nd.depth;

            if(IS_BRANCH(nd)) {
                // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
                Flag flag_sn, flag_ew;
                FIND_QUADRANT(x_nom, y_nom, nd.rect,      flag_sn, flag_ew);
                uint32_t id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
                
                nodes_[id_child].depth = depth + 1;
                id_node = id_child;
            }
            else if(IS_LEAF(nd)) { // This is a leaf node.
                addDataToNode(id_node, insert_data_.id_elem);

                if(depth < max_depth_){ // nonmax depth.
                    int n_elem = getNumElemOfNode(id_node);
                    if(n_elem > max_elem_per_leaf_){ // too much data. divide.
                        // Make all child to leaf (with no element)
                        uint32_t id_child = GET_FIRST_CHILD_ID(id_node);
                        makeChildrenLeaves(id_child, nd.rect);
                        n_node_activated_ += 4;

                        // Do divide.
                        for(const uint32_t& id_elem_tmp : ndelems.elem_ids) 
                        {
                            Flag flag_sn, flag_ew;
                            QuadElement& elem_tmp = all_elems_[id_elem_tmp];
                            FIND_QUADRANT(elem_tmp.x_nom, elem_tmp.y_nom, nd.rect, flag_sn, flag_ew);
                            uint32_t id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
                            nodes_[id_child].depth = depth + 1;
                            addDataToNode(id_child, id_elem_tmp);
                        }

                        //make this node as a branch
                        makeBranch(id_node);
                    }
                }
                break;
            }

        }

        
    };

    inline void Quadtree::addDataToNode(uint32_t id_node, uint32_t id_elem){
        node_elements[id_node].elem_ids.push_back(id_elem);
    };

    inline int Quadtree::getNumElemOfNode(uint32_t id_node){
        return node_elements[id_node].elem_ids.size();
    };


    inline void Quadtree::makeChildrenLeaves(uint32_t id_child, const QuadRect_u& rect){
        uint16_t cent_x = (rect.tl.x + rect.br.x) >> 1;
        uint16_t cent_y = (rect.tl.y + rect.br.y) >> 1;

        MAKE_LEAF(nodes_[id_child]); // (0,0) (top left)
        nodes_[id_child].rect.tl.x = rect.tl.x; nodes_[id_child].rect.tl.y = rect.tl.y; 
        nodes_[id_child].rect.br.x = cent_x;    nodes_[id_child].rect.br.y = cent_y;

        MAKE_LEAF(nodes_[++id_child]); // (0,1) (top right)
        nodes_[id_child].rect.tl.x = cent_x;    nodes_[id_child].rect.tl.y = rect.tl.y; 
        nodes_[id_child].rect.br.x = rect.br.x; nodes_[id_child].rect.br.y = cent_y;

        MAKE_LEAF(nodes_[++id_child]); // (1,0) (bot left)
        nodes_[id_child].rect.tl.x = rect.tl.x; nodes_[id_child].rect.tl.y = cent_y; 
        nodes_[id_child].rect.br.x = cent_x;    nodes_[id_child].rect.br.y = rect.br.y;

        MAKE_LEAF(nodes_[++id_child]); // (1,1) (bot right)
        nodes_[id_child].rect.tl.x = cent_x;    nodes_[id_child].rect.tl.y = cent_y; 
        nodes_[id_child].rect.br.x = rect.br.x; nodes_[id_child].rect.br.y = rect.br.y;
    };

    inline void Quadtree::makeBranch(uint32_t id_node){
        MAKE_BRANCH(nodes_[id_node]);
        node_elements[id_node].reset();
    };

    inline bool Quadtree::BWBTest(float x, float y, const QuadRect_u& rect, float radius){
        // Ball Within Bound (Circle bounded by rect check)
        float rect_size = (float)(rect.br.x-rect.tl.x);
        return (rect_size >= 2*radius) && INBOUND_RECT_PTS(x, y, rect.tl.x+radius,rect.tl.y+radius, rect.br.x-radius, rect.br.y-radius);
    };

    inline bool Quadtree::BOBTest(float x, float y, const QuadRect_u& rect, float R){
        // Ball Overlap Bound (Circle-rect collision check)
        float rect_center_x   = (float)(rect.br.x+rect.tl.x)*0.5f;
        float rect_center_y   = (float)(rect.br.y+rect.tl.y)*0.5f;
        float rect_halfsize_x = (float)(rect.br.x-rect.tl.x)*0.5f;
        float rect_halfsize_y = (float)(rect.br.y-rect.tl.y)*0.5f;

        float dx = fabs(x - rect_center_x);
        float dy = fabs(y - rect_center_y);
        
        if(dx > (rect_halfsize_x + R) ) return false;
        if(dy > (rect_halfsize_y + R) ) return false;

        if(dx <= rect_halfsize_x) return true;
        if(dy <= rect_halfsize_y) return true;

        float corner_dist_sq = (dx-rect_halfsize_x)*(dx-rect_halfsize_x)
                            + (dy-rect_halfsize_y)*(dy-rect_halfsize_y);
        
        return (corner_dist_sq <= (R*R));
    };

    void Quadtree::resetNNParameters(){
        // Initialize values
        simple_stack_.clear();
        query_data_.min_dist2_ = std::numeric_limits<float>::max();
        query_data_.min_dist_  = std::numeric_limits<float>::max();
    };

    void Quadtree::nearestNeighborSearchPrivate(){
        // In this function, search the nearest element from the scratch (from the root node)
        float x_nom = query_data_.x_nom;
        float y_nom = query_data_.y_nom;
        uint32_t& id_data_matched = query_data_.id_data_matched;
        uint32_t& id_elem_matched = query_data_.id_elem_matched;
        uint32_t& id_node_matched = query_data_.id_node_matched;

        // Initialize NN parameters
        resetNNParameters();

        // Start from the root node.
        // Do DeQuadPointh First Search (DFS) 
        simple_stack_.push(1); 
        while( !simple_stack_.empty() ){
            uint32_t id_node   = simple_stack_.topAndPop();
            QuadNode& nd = nodes_[id_node];
            // Current depth <= max_depth_ ?? 

            // If leaf node, find nearest point and 'BWBTest()'
            // if(nd.isLeaf()){
            if(IS_LEAF(nd)){
                simple_stack_.addTotalAccess();
                QuadNodeElements& nd_elems = node_elements[id_node];
                if( findNearestElem(x_nom, y_nom, nd_elems) ) {
                    id_node_matched = id_node;
                }

                if( BWBTest(x_nom, y_nom, nd.rect, query_data_.min_dist_) ) break; // the nearest point is inside the node.
            }
            else { // this is not a leaf node.
                simple_stack_.addTotalAccess();

                // Go to child. Find most probable child first.
                uint32_t id_child = GET_FIRST_CHILD_ID(id_node);
                if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
            }
        }
    };



    bool Quadtree::findNearestElem(float x, float y, const QuadNodeElements& elems_thisnode){
        bool findNewNearest = false;
        for(const uint32_t& id_elem : elems_thisnode.elem_ids)
        {
            QuadElement& elem = all_elems_[id_elem];
            float dist_temp = DIST_EUCLIDEAN(x,y, elem.x_nom,elem.y_nom);
            if(dist_temp < query_data_.min_dist2_)
            {
                this->query_data_.id_data_matched = elem.id_data;
                this->query_data_.id_elem_matched = id_elem;
                this->query_data_.min_dist2_      = dist_temp * params_.approx_rate;
                this->query_data_.min_dist_       = sqrt(query_data_.min_dist2_);
                findNewNearest  = true;
            }
        }
        return findNewNearest;
    };

    void Quadtree::cachedNearestNeighborSearchPrivate()
    {
        // In this function, search the nearest element from the scratch (from the root node)
        const float& x_nom = query_data_.x_nom;
        const float& y_nom = query_data_.y_nom;
        uint32_t& id_data_matched = query_data_.id_data_matched;
        uint32_t& id_elem_matched = query_data_.id_elem_matched;

        // Initialize NN parameters
        resetNNParameters();

        // Start from the cached node. 
        // The cached node should be a leaf node.
        // If the cached node is invalid or the root node, normal NN search is executed.
        if(query_data_.id_node_cached <= 1)
        { 
            nearestNeighborSearchPrivate();
            return;
        }
        else // cached node is neither a root node nor a invalid node.
        {  
            query_data_.id_node_matched = query_data_.id_node_cached;
            simple_stack_.addTotalAccess();

            // Find nearest point in the cached node.
            findNearestElem(x_nom, y_nom, node_elements[query_data_.id_node_matched]);
            if( BWBTest(x_nom, y_nom, nodes_[query_data_.id_node_matched].rect, query_data_.min_dist_) ) {
                // the nearest point is inside the cached node.
                return;
            } 

            // Go up to the until satisfying 'BWBTest == true'
            uint32_t id_node = GET_PARENT_ID(query_data_.id_node_cached);

            while(true)
            {
                simple_stack_.addTotalAccess();

                if(id_node <= 1)
                {
                    id_node = 1;
                    break; // reach the root node.
                } 
                // If 'BWBTest' is passed on this node, the nearest one should be within
                // the chilren of this node. 
                if(BWBTest(x_nom, y_nom, nodes_[id_node].rect, query_data_.min_dist_)) 
                    break; 

                id_node = GET_PARENT_ID(id_node);
            }
                
            // Start from the root node.
            // Do Breadth First Search (BFS) 
            simple_stack_.push(id_node); 
            while(!simple_stack_.empty())
            {
                id_node = simple_stack_.topAndPop();
                QuadNode& nd = nodes_[id_node];

                // If leaf node, find nearest point and 'BWBTest()'
                // if(nd.isLeaf()){
                if(IS_LEAF(nd))
                {
                    simple_stack_.addTotalAccess();
                    QuadNodeElements& nd_elems = node_elements[id_node];
                    if(findNearestElem(x_nom, y_nom, nd_elems)) {
                        query_data_.id_node_matched = id_node;
                    }

                    if( BWBTest(x_nom, y_nom, nd.rect, query_data_.min_dist_) ) 
                        break; // the nearest point is inside the node.
                }
                else{ // this is not a leaf node.
                    simple_stack_.addTotalAccess();

                    // Go to child. Find most probable child first.
                    uint32_t id_child = GET_FIRST_CHILD_ID(id_node);
                    if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_)) 
                        simple_stack_.push(id_child);

                    ++id_child;
                    if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                        simple_stack_.push(id_child);
                    
                    ++id_child;
                    if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                        simple_stack_.push(id_child);
                    
                    ++id_child;
                    if(IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                        simple_stack_.push(id_child);
                    
                }
            }
        }
    };

    void Quadtree::searchNN(float x, float y, 
        uint32_t& id_data_matched, uint32_t& id_node_matched)
    {
        query_data_.x = x; 
        query_data_.y = y;
        query_data_.x_nom = x*x_normalizer_;
        query_data_.y_nom = y*y_normalizer_;
        query_data_.id_node_cached = 1; // non-cached case now.

        nearestNeighborSearchPrivate();

        id_data_matched = query_data_.id_data_matched;
        id_node_matched = query_data_.id_node_matched;
    };

    void Quadtree::searchNNDebug(float x, float y, 
        uint32_t& id_data_matched, uint32_t& id_node_matched, uint32_t& n_access)
    {
        searchNN(x,y, id_data_matched, id_node_matched);
        n_access = simple_stack_.getTotalAccess();
    };

    void Quadtree::searchNNCached(float x, float y, int id_node_cached, 
        uint32_t& id_data_matched, uint32_t& id_node_matched)
    {
        query_data_.x = x;
        query_data_.y = y;
        query_data_.x_nom = x*x_normalizer_;
        query_data_.y_nom = y*y_normalizer_;
        query_data_.id_node_cached = id_node_cached; 
        
        cachedNearestNeighborSearchPrivate();

        id_data_matched = query_data_.id_data_matched;
        id_node_matched = query_data_.id_node_matched;
    };

    void Quadtree::searchNNCachedDebug(float x, float y, int id_node_cached, 
        uint32_t& id_data_matched, uint32_t& id_node_matched, uint32_t& n_access)
    {
        searchNNCached(x,y, id_node_cached, id_data_matched, id_node_matched);
        n_access = simple_stack_.getTotalAccess();
    };

    inline void Quadtree::resetInsertData(){
        insert_data_.id_elem = 0; 
        insert_data_.x_nom   = -1.0f;
        insert_data_.y_nom   = -1.0f;
    };

    inline void Quadtree::resetQueryData(){
        query_data_.id_data_matched = 0;
        query_data_.id_elem_matched = 0;
        query_data_.id_node_cached  = 1;
        query_data_.id_node_matched = 1;
        query_data_.min_dist2_ = std::numeric_limits<float>::max();
        query_data_.min_dist_  = std::numeric_limits<float>::max();
        query_data_.x     = -1.0f;
        query_data_.y     = -1.0f;
        query_data_.x_nom = -1.0f;
        query_data_.y_nom = -1.0f;   
    };


    uint32_t Quadtree::getNumNodes(){
        return n_nodes_;
    };
    uint32_t Quadtree::getNumNodesActivated(){
        return n_node_activated_;
    };

    // Quadtree::QuadNode* Quadtree::getAdjacentNode_greater_of_equal_size(QuadNode* nd, uint8_t dir){
    //     // 찾고자 하는 곳: north adjacent.
    //     // nd != nd.parent.child(south) 이면, 재귀적으로 부모로 계속 올라감. 
    //     // nd == nd.parent.child(south) 이면 nd.parent.child(north) 혹은 그 아래에 해가 있음.

    //     if(nd.parent == nullQuadPointr){
    //         return none;
    //     }
    //     if(nd.parent.children(SW) == nd){
    //         return nd.parent.children[NW];
    //     }
    //     if(nd.parent.children(SE) == nd){
    //         return nd.parent.children[NE];
    //     }

    //     node = getAdjacentNode_greater_or_equal_size(nd.parent);

    //     // nd is guaranteed to be a north child.
    //     return (node.children[SW]
    //         if(nd.parent.children(NW][ == nd)
    //         else node.children[SE]);

    // };

    // // Complexity is bounded by the depth of the quadtree.
    // getAdjacentNode_smaller(){
    //     //계속 남쪽으로 내려가면 된다.
    //     // stack 기반으로 ㄱㄱ
    // };

};