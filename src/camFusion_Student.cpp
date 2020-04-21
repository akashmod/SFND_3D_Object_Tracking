
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
    std::vector<cv::DMatch> bBox_kptMatches;
    float distance_total=0, distance_avg=0;
    for ( std::vector<cv::DMatch>::iterator mat = kptMatches.begin(); mat!= kptMatches.end(); ++mat)
    {
    distance_total = distance_total + mat->distance;
    }
    distance_avg=distance_total/kptMatches.size(); 

    for ( std::vector<cv::DMatch>::iterator mat = kptMatches.begin(); mat!= kptMatches.end(); ++mat)
    {
        //std::vector<cv::KeyPoint>::iterator it = find(boundingBox.keypoints.begin(), boundingBox.keypoints.end(), kptsCurr[mat->trainIdx]);
        cv::KeyPoint kptCurr = kptsCurr[mat->trainIdx];
        //cv::KeyPoint kptPrev = kptsCurr[mat->queryIdx];
            if(boundingBox.roi.contains(kptCurr.pt))
            {
                if (mat->distance<1.2*distance_avg)
                    bBox_kptMatches.push_back(*mat);
            }
    } 
    boundingBox.kptMatches=bBox_kptMatches;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
     // ...
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }


    // // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    double medianDistRatio = distRatios[distRatios.size() / 2];
    //long medIndex = floor(distRatios.size() / 2.0);
    //double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);
    // EOF STUDENT TASK
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
     // ...
    double dT = 0.1; // time between two measurements in seconds
    double x_avg_prev=0, x_avg_curr=0;
    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        if(abs(it->y<=2))
        {
        x_avg_prev+=(it->x)/(double)lidarPointsPrev.size();
        }
    }
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        if(abs(it->y<=2))
        {
        x_avg_curr+=(it->x)/(double)lidarPointsCurr.size();
        }
    }
    
    // for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        
    //     if(abs(it->y<=2)&&abs(it->x-x_avg_prev)<0.1)
    //     {
    //     cout<<it->x<<" ";
    //     minXPrev = minXPrev>it->x ? it->x : minXPrev;
    //     }
    //     }
    //     cout<<endl<<minXPrev<<endl;

    // for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        
    //      if(abs(it->y<=2)&&abs(it->x-x_avg_curr)<0.1)
    //     {
    //         cout<<it->x<<" ";
    //     minXCurr = minXCurr>it->x ? it->x : minXCurr;
    //     }
    //}
    //cout<<endl<<minXCurr<<endl;
    // compute TTC from both measurements
    TTC = (x_avg_curr-0.2) * dT / (x_avg_prev-x_avg_curr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    cv::KeyPoint kpt_curr;
    std::multimap<int,int> bBox_kptmatches;
    int index=0, prev_boxId,curr_boxId;
    for (std::vector<cv::KeyPoint>::iterator kpt_prev = prevFrame.keypoints.begin(); kpt_prev!=prevFrame.keypoints.end(); ++kpt_prev)
    {
        
        kpt_curr = currFrame.keypoints[matches[index].trainIdx];
        for(std::vector<BoundingBox>::iterator bbox_prev = prevFrame.boundingBoxes.begin(); bbox_prev!=prevFrame.boundingBoxes.end(); ++bbox_prev)
            if(bbox_prev->roi.contains(kpt_prev->pt))
                prev_boxId=bbox_prev->boxID;
            
        for(std::vector<BoundingBox>::iterator bbox_curr = currFrame.boundingBoxes.begin(); bbox_curr!=currFrame.boundingBoxes.end(); ++bbox_curr)
        if(bbox_curr->roi.contains(kpt_curr.pt)) 
                curr_boxId=bbox_curr->boxID;
        
        bBox_kptmatches.insert( std::pair<int,int>(prev_boxId,curr_boxId) );
        index++;
    }
    for (std::vector<BoundingBox>::iterator bbox_prev = prevFrame.boundingBoxes.begin(); bbox_prev!=prevFrame.boundingBoxes.end(); ++bbox_prev )
    {
    auto range = bBox_kptmatches.equal_range(bbox_prev->boxID);
    std::multimap<int,int> bBox_kptmatches_swap;
    for (auto i = range.first; i != range.second; ++i)
        bBox_kptmatches_swap.insert(std::pair<int,int>(i->second,i->first));
    int max_cnt=0, max_curr=0;
    for (std::vector<BoundingBox>::iterator bbox_curr = currFrame.boundingBoxes.begin(); bbox_curr!=currFrame.boundingBoxes.end(); ++bbox_curr)
    {
        int cnt = bBox_kptmatches_swap.count(bbox_curr->boxID);
        if (cnt>max_cnt)
        {
            max_curr=bbox_curr->boxID;
            max_cnt=cnt;
        }
    }
    bbBestMatches.insert(std::pair<int,int>(bbox_prev->boxID,max_curr));
    //cout<<bbox_prev->boxID<<" "<<max_curr<<endl;

    }}
        // 'bb' is bounding box

    // std::map<std::pair<int, int>, int> bb_matches;
    // for (auto match : matches)
    // {
    //     // 'match' function was called with 'cur_frame' as reference,
    //     // => trainId refers to keypoint in cur_frame, queryId is previous frame.
    //     auto prev_keypoint = prevFrame.keypoints[match.queryIdx];
    //     auto curr_keypoint = currFrame.keypoints[match.trainIdx];

    //     // count common keypoints for previous and current frames' bounding boxes
    //     for (auto curr_bb : currFrame.boundingBoxes)
    //     {
    //         if (curr_bb.roi.contains(curr_keypoint.pt))
    //         {
    //             for (auto prev_bb : prevFrame.boundingBoxes)
    //             {
    //                 if (prev_bb.roi.contains(prev_keypoint.pt))
    //                 {
    //                     auto key = std::make_pair(prev_bb.boxID, curr_bb.boxID);
    //                     bb_matches[key]++;
    //                 }
    //             }
    //         }
    //     }
    // }

    // // max number of matches per bounding box
    // std::map<int, int> max_matches;

    // // find pair with most matches
    // for (auto bb_match : bb_matches)
    // {
    //     if (max_matches[bb_match.first.first] < bb_match.second)
    //     {
    //         max_matches[bb_match.first.first] = bb_match.second;
    //         bbBestMatches[bb_match.first.first] = bb_match.first.second;
    //     }
    // }
    // }

    // for (std::vector<BoundingBox>::iterator bbox_prev = prevFrame.boundingBoxes.begin(); bbox_prev!=prevFrame.boundingBoxes.end(); ++bbox_prev)
    // {
    //     std::vector<int> bbox_match_count;
    //     if (bbox_prev->keypoints.size()!=0)
    //     {
    //     for (std::vector<cv::KeyPoint>::iterator kpt = bbox_prev->keypoints.begin(); kpt != bbox_prev->keypoints.end(); ++kpt)
    //     {
    //         for(std::vector<BoundingBox>::iterator bbox_curr = currFrame.boundingBoxes.begin(); bbox_curr!=currFrame.boundingBoxes.end(); ++bbox_curr)
    //         {
    //          if(bbox_curr->roi.contains(currFrame.keypoints[matches[std::distance(prevFrame.keypoints.begin(), kpt)].trainIdx].pt))
    //          {
    //              bbox_match_count.push_back(bbox_curr->boxID);
    //              break;
    //          }
    //         }}
    // int max=0, highest_count=bbox_match_count[0];
    // int co;
    // for(int i=0;i<bbox_match_count.size();i++)
    // {
    //     co = (int)count(bbox_match_count.begin(), bbox_match_count.end(), bbox_match_count[i]);
    //     if(co > max)
    //     {       max = co;
    //             highest_count = bbox_match_count[i];
    //     }
    // }
    // bbBestMatches.insert ( std::pair<int,int>(bbox_prev->boxID,highest_count) );
    // cout<<bbox_prev->boxID<<" "<<highest_count<<endl;
    // }}
