#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){
          c.clear();
    int last_best = -1;
    const int n = trans_points.size();
    const int m = old_points.size();
    int min_index = 0;
    int second_min_index = 0;
    //ROS_INFO_STREAM("size n: " << n);
    //ROS_INFO_STREAM("size m: " << m);

    //Do for each point
    for(int ind_trans = 0; ind_trans<n; ++ind_trans){
        float min_dist = 100000.00;
        for(int ind_old = 0; ind_old<m; ++ind_old){
            float dist = old_points[ind_trans].distToPoint2(&trans_points[ind_old]);
            if(dist<min_dist){
                min_dist = dist;
                min_index = ind_old;
                if (ind_old == 0){
                    second_min_index = ind_old + 1;
                }else{
                    second_min_index = ind_old - 1;
                }
            }
        }
        c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[min_index], &old_points[second_min_index]));
    }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point. 
  //Initializecorrespondences
      c.clear();
    int last_best = -1;
    const int trans_size = trans_points.size();
    const int old_size = old_points.size();
    
    //Do for each point
    for(int i = 0; i<min(trans_size,old_size); ++i) {
      // initialize best index and best distance for the point p_iw
      int best = -1;
      double best_dist = 99999.0;
      // approximate start index around p_iw
      //int start_index = trans_points[i].theta*(trans_size)/(2*M_PI);
      int we_start_at = (last_best != -1) ? (last_best + 1) : i;
      int up = min(we_start_at + 1,trans_size);
      int down = we_start_at;
      // make first search always up
      double last_dist_up = 99999.0; 
      double last_dist_down = 99999.0;

      bool up_stopped = false;
      bool down_stopped = false;
      while(!(up_stopped && down_stopped)){
        
        bool now_up = !up_stopped;
        if(now_up){
          if(up >= min(trans_size,old_size)){
          
          up_stopped = true;
          continue;
          }
          //obtain the distances between old scan and transformed point
          last_dist_up = trans_points[i].distToPoint2(&old_points[up]);
          //store best value
         if(last_dist_up < best_dist){
          best = up;
          best_dist = last_dist_up;
          }
         if(up > i){
          double angle_diff = trans_points[i].theta - old_points[up].theta;
          double min_dist_up = sin(angle_diff)*trans_points[i].r;
          // check for early stop
          if((min_dist_up*min_dist_up) > best_dist){
            up_stopped = true;
            continue;
          }
          // find point to jump upper bigger and smaller
          up = (old_points[up].r < trans_points[i].r)? jump_table[up][UP_BIG]: jump_table[up][UP_SMALL];
          /*if(old_points[up].r < trans_points[i].r){
            vector<int> v;
            v = jump_table[up];
            up = v[1];
          }
          else{
               vector<int> v;
               v = jump_table[up];
               up = v[0];
             }*/
          }
          // else for
         else{
           up++;
          }
        
        }// now up
        // check downward direction
        if(!now_up){
          if(down < 0){
            //ROS_INFO_STREAM("down stopped true");
            down_stopped = true;
            continue;
          }
          last_dist_down = trans_points[i].distToPoint2(&old_points[down]);
          if(last_dist_down < best_dist){
            best = down;
            best_dist = last_dist_down;
          }
          if(down < i){
            double angle_diff = trans_points[i].theta - old_points[down].theta;
            double min_dist_down = sin(angle_diff)*trans_points[i].r;
            if((min_dist_down*min_dist_down) > best_dist){
              down_stopped = true;
              continue;
            }
            // find point to jump upper bigger and smaller
            down = (old_points[down].r < trans_points[i].r)? jump_table[down][DOWN_BIG]: jump_table[down][DOWN_SMALL];
            /*if(old_points[down].r < trans_points[i].r){
            vector<int> v;
            v = jump_table[down];
            down = v[3];
            }
            else{
               vector<int> v;
               v = jump_table[down];
               down = v[2];
             }*/
          }
          else{
            down --;
          }
        }
       
      } // while loop
      
      last_best = best;
      int second_best;
      if(best == 0){
        second_best = best+1;
      }
      else{
        second_best = best - 1;
      }
      c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best])); 
    }

  
  }


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}