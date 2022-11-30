#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <time.h>
#include <string>
#include <sstream>
#include <exception>

#include <memory>
#include <algorithm>
#include <random>
#include <functional>

#include "quadtree_array.h"
#include "timer.h"

using namespace ArrayBased;

int main(int argc, char **argv) {
    ros::init(argc, argv, "quadtree_node");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("quadtree_node - starts.");

    std::mt19937 engine((unsigned int)time(NULL));
    std::uniform_real_distribution<> distribution(30.0, 750.0);

    auto generator = std::bind(distribution, engine);

    float x_range[2] = {0.f,1032.f};
    float y_range[2] = {0.f,772.f};

    uint32_t max_depth         = 10;
    uint32_t max_elem_per_leaf = 20;

    float approx_rate = 1;
    int n_pts = 150000;
    int n_pts_q = 4000;

    std::vector<std::pair<float,float>> pts_data;
    for(uint32_t i = 0; i < n_pts; ++i){
        pts_data.push_back( std::make_pair<float,float>(generator(), generator()) );
    }

    std::vector<std::pair<float,float>> pts_q;
    for(uint32_t i = 0; i < n_pts_q; ++i){
        pts_q.push_back(std::make_pair<float,float>(generator(), generator()));
    }

    std::vector<uint32_t> id_node_matched_q;
    std::vector<uint32_t> id_data_matched_q;
    id_node_matched_q.resize(n_pts_q);
    id_data_matched_q.resize(n_pts_q);

    std::vector<double> min_dist2_true;
    std::vector<uint32_t> id_data_matched_q_true;
    min_dist2_true.resize(n_pts_q);
    id_data_matched_q_true.resize(n_pts_q);

    for(uint32_t i = 0; i < n_pts_q; ++i){
        double min_dist2 = std::numeric_limits<double>::max();
        uint32_t id_data_tmp = 0;
        for(uint32_t j = 0; j < n_pts; ++j){
            double dist = DIST_EUCLIDEAN(pts_q[i].first, pts_q[i].second, 
                pts_data[j].first, pts_data[j].second);

            if( dist < min_dist2){
                id_data_tmp = j;
                min_dist2   = dist;
            }
        }

        min_dist2_true[i]         = min_dist2;
        id_data_matched_q_true[i] = id_data_tmp;
    }



    try {  
        std::vector<float> time_construct(max_depth+1);
        std::vector<float> time_insert(max_depth+1);

        std::vector<float> time_normal(max_depth+1);
        std::vector<float> time_cached(max_depth+1);

        std::vector<uint32_t> access_normal(max_depth+1);
        std::vector<double> min_access_normal(max_depth+1);
        std::vector<double> max_access_normal(max_depth+1);
        std::vector<double> avg_access_normal(max_depth+1);
        std::vector<double> std_access_normal(max_depth+1);

        std::vector<uint32_t> access_cached(max_depth+1);
        std::vector<double> min_access_cached(max_depth+1);
        std::vector<double> max_access_cached(max_depth+1);
        std::vector<double> avg_access_cached(max_depth+1);
        std::vector<double> std_access_cached(max_depth+1);

        std::vector<uint32_t> diff_normal(max_depth+1);
        std::vector<uint32_t> diff_cached(max_depth+1);

        for(int d = 0; d <= max_depth; ++d){
            timer::tic();
            std::shared_ptr<Quadtree> qt = nullptr;
            qt = std::make_shared<Quadtree>(
                x_range[0],x_range[1],y_range[0], y_range[1], d, max_elem_per_leaf,
                approx_rate);
            time_construct[d] = timer::toc(0);

            // Insert points
            timer::tic();
            for(uint32_t i = 0; i < n_pts; ++i){
                uint32_t id_data = i;
                qt->insert(pts_data[i].first, pts_data[i].second, id_data);
            }

            std::cout << "# nodes - total / activated / ratio: " 
            << qt->getNumNodes() <<", " << qt->getNumNodesActivated() <<", " 
            << (double)qt->getNumNodesActivated()/(double)qt->getNumNodes()*100.0 << " % " << std::endl;

            time_insert[d] = timer::toc(0);

            // normal matching
            std::vector<uint32_t> accesses;
            timer::tic();
            for(uint32_t i = 0; i < n_pts_q; ++i){
                uint32_t access_temp = 0;
                qt->searchNNDebug(pts_q[i].first, pts_q[i].second, 
                    id_data_matched_q[i], id_node_matched_q[i], access_temp);
                access_normal[d] += access_temp;
                accesses.push_back(access_temp);
            }
            time_normal[d] = timer::toc(0);
            std::sort(accesses.begin(),accesses.end());
            min_access_normal[d] = accesses.front();
            max_access_normal[d] = accesses.back();
            double sum = std::accumulate(accesses.begin(), accesses.end(), 0.0);
            double mean = sum / accesses.size();
            double sq_sum = std::inner_product(accesses.begin(), accesses.end(), accesses.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / accesses.size() - mean * mean);
            avg_access_normal[d] = mean;
            std_access_normal[d] = stdev;
            
            for(int i = 0; i < n_pts_q; ++i){
                uint32_t id_mat = id_data_matched_q[i];
                uint32_t id_mat_true = id_data_matched_q_true[i];

                if(id_mat != id_mat_true){
                    diff_normal[d]++;
                    std::cout << "mismatched: " << id_mat <<"," <<id_mat_true <<" / " 
                    << pts_data[id_mat].first <<"," << pts_data[id_mat].second
                    <<" / " << pts_data[id_mat_true].first <<"," <<pts_data[id_mat_true].second 
                    <<" / mindist2 (true,est):" << min_dist2_true[i] <<"," << 
                    DIST_EUCLIDEAN(pts_data[id_mat].first, pts_data[id_mat].second, pts_q[i].first, pts_q[i].second) << std::endl;
                } 
            }

            // Cached matching
            accesses.resize(0);
            timer::tic();
            for(int i = 0; i < n_pts_q; ++i){
                uint32_t access_temp = 0;
                qt->searchNNCachedDebug(pts_q[i].first, pts_q[i].second, id_node_matched_q[i],
                    id_data_matched_q[i], id_node_matched_q[i], access_temp);
                access_cached[d] += access_temp;
                accesses.push_back(access_temp);
            }
            time_cached[d] = timer::toc(0);
            
            for(int i = 0; i < n_pts_q; ++i){
                uint32_t id_mat = id_data_matched_q[i];
                uint32_t id_mat_true = id_data_matched_q_true[i];

                if(id_mat != id_mat_true){
                    diff_cached[d]++;
                    std::cout << "mismatched: " << id_mat <<"," <<id_mat_true <<" / "
                    << pts_data[id_mat].first <<"," << pts_data[id_mat].second
                    <<" / " << pts_data[id_mat_true].first <<"," <<pts_data[id_mat_true].second 
                    <<" / mindist2 (true,est):" << min_dist2_true[i] <<"," << 
                    DIST_EUCLIDEAN(pts_data[id_mat].first, pts_data[id_mat].second, pts_q[i].first, pts_q[i].second) << std::endl;
                } 
            }
            std::sort(accesses.begin(),accesses.end());
            min_access_cached[d] = accesses.front();
            max_access_cached[d] = accesses.back();
            sum = std::accumulate(accesses.begin(), accesses.end(), 0.0);
            mean = sum / accesses.size();
            sq_sum = std::inner_product(accesses.begin(), accesses.end(), accesses.begin(), 0.0);
            stdev = std::sqrt(sq_sum / accesses.size() - mean * mean);
            avg_access_cached[d] = mean;
            std_access_cached[d] = stdev;
        }       

        // Show the test results
        std::cout << "==== TIME ANALYSIS ====\n";
        for(int d = 0; d <= max_depth; ++d) {
            std::cout << " d[" << d <<"] - " 
            <<"build: "<< time_construct[d] <<", "
            <<"insert: " << time_insert[d] <<", "
            <<"NN search (normal/cached): " << time_normal[d] <<", " << time_cached[d] <<"\n";
        }
        std::cout << "==== ACCESS ANALYSIS ====\n";
        for(int d = 0; d < max_depth; ++d){
            std::cout << " d[" << d <<"] - (normal/cached)" 
            << "total: " << access_normal[d] <<", " << access_cached[d] <<" / "
            << "min: " << min_access_normal[d] <<"," << min_access_cached[d] <<" / "
            << "max: " << max_access_normal[d] <<"," << max_access_cached[d] <<" / "
            << "avg: " << avg_access_normal[d] <<"," << avg_access_cached[d] <<" / "
            << "std: " << std_access_normal[d] <<"," << std_access_cached[d] << std::endl;
        }
    }
    catch (std::exception& e) {
        ROS_ERROR(e.what());
    }

    ROS_INFO_STREAM("quadtree_node - TERMINATED.");
    return -1;
}