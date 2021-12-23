
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
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

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
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

unordered_set<int> Ransac2D(std::vector<LidarPoint> &lidarPoints, int maxIterations, double distanceThreshold){
    unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--) {
        unordered_set<int> inliers;
        while (inliers.size() < 2) {
            inliers.insert(rand()%(lidarPoints.size()));
        }

        float x1, y1, x2, y2;

        auto itr = inliers.begin();
        x1 = lidarPoints[*itr].x;
        y1 = lidarPoints[*itr].y;
        itr++;
        x2 = lidarPoints[*itr].x;
        y2 = lidarPoints[*itr].y;

        float a = (y2 - y1);
        float b = (x2 - x1);
        float c = (x1 * y2 - x2 * y1);

        for (int index = 0; index < lidarPoints.size(); index++) {
            if (inliers.count(index) > 0)
                continue;

            LidarPoint lp = lidarPoints[index];
            float x3 = lp.x;
            float y3 = lp.y;

            float d = fabs(a*x3 + b*y3 + c)/sqrt(a*a + b*b);

            if (d <= distanceThreshold)
                inliers.insert(index);
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    cout << "Ransac done, found " << inliersResult.size() << " inliers out of " << lidarPoints.size() << " points." << endl;

    return inliersResult;
}

std::vector<LidarPoint> removeOutliers(std::vector<LidarPoint> &lidarPoints, int maxIterations, float distanceThreshold){
    // Plane segmentation
    std::unordered_set<int> inliers = Ransac2D(lidarPoints, maxIterations, distanceThreshold);

    std::vector<LidarPoint> lidarPointsInliers;

    for(int index = 0; index < lidarPoints.size(); index++)
    {
        if(inliers.count(index)) {
            LidarPoint point = lidarPoints[index];
            lidarPointsInliers.push_back(point);
        }
    }
    return lidarPointsInliers;
}

std::tuple<double, double, double> quartilesOnXaxisLidarPoints(std::vector<LidarPoint> &lidarPoints){
    std::vector<double> v_x;
    for (LidarPoint lp: lidarPoints)
        v_x.push_back(lp.x);

    auto const Q1 = v_x.size() / 4;
    auto const Q2 = v_x.size() / 2;
    auto const Q3 = Q1 + Q2;

    std::nth_element(v_x.begin(),          v_x.begin() + Q1, v_x.end());
    std::nth_element(v_x.begin() + Q1 + 1, v_x.begin() + Q2, v_x.end());
    std::nth_element(v_x.begin() + Q2 + 1, v_x.begin() + Q3, v_x.end());

    return {v_x[Q1], v_x[Q2], v_x[Q3]};
}

void filterOutliers(std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr){
    // calculate euclidean distance between keypoints in each match
    std::vector<double> v_euclidean_distance;
    for (cv::DMatch match: matches){
        v_euclidean_distance.push_back( cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt) );
    }

    auto const Q1 = v_euclidean_distance.size() / 4;
    auto const Q2 = v_euclidean_distance.size() / 2;
    auto const Q3 = Q1 + Q2;

    std::nth_element(v_euclidean_distance.begin(),          v_euclidean_distance.begin() + Q1, v_euclidean_distance.end());
    std::nth_element(v_euclidean_distance.begin() + Q1 + 1, v_euclidean_distance.begin() + Q2, v_euclidean_distance.end());
    std::nth_element(v_euclidean_distance.begin() + Q2 + 1, v_euclidean_distance.begin() + Q3, v_euclidean_distance.end());

    double IQR = v_euclidean_distance[Q3] - v_euclidean_distance[Q1];
    double low_outlier_thold = v_euclidean_distance[Q1] - 1.5 * IQR;
    double high_outlier_thold = v_euclidean_distance[Q3] + 1.5 * IQR;

    int i = 0;
    for (double dist : v_euclidean_distance) {
        if (dist < low_outlier_thold || dist > high_outlier_thold) {
            matches.erase(matches.begin() + i);
            i--;
        }
        i++;
    }
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    bool trou = true;

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
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        if (trou) {
            putText(topviewImg, str1, cv::Point2f(left-250, bottom-175), cv::FONT_ITALIC, 2, currColor);
            putText(topviewImg, str2, cv::Point2f(left-250, bottom-100), cv::FONT_ITALIC, 2, currColor);
            trou = false;
        } else {
            putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
            putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
            trou = true;
        }
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
    //looking for matches in bounding box, if kpt at trainIdx is in roi
    for (cv::DMatch match: kptMatches){
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)) {
            boundingBox.kptMatches.push_back(match);
        }
    }

    cout << "First step: selected " << boundingBox.kptMatches.size() << " kptsMatches !" << endl;

    // Filter matches by removing outliers, using Inter Quartile Range (IQR) on euclidean distance between keypoints of each match. 
    filterOutliers(boundingBox.kptMatches, kptsPrev, kptsCurr);

    cout << "After filter step: selected " << boundingBox.kptMatches.size() << " kptsMatches !" << endl;

    // add kpts from matches 
    for (cv::DMatch match: boundingBox.kptMatches){
        boundingBox.keypoints.push_back(kptsCurr[match.trainIdx]);
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double dT = 1.0 / frameRate;
    // compute distance ratios between all matched keypoints
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
        cout << "TTC Camera: list of distance ratios is empty." << endl;
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    if (medDistRatio == 1)
    {
        cout << "TTC Camera: Median distance ratio is equal to 1, no change between frame" << endl;
        TTC = NAN;
        return;
    }

    dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
    cout << "TTC Camera: Median distance ratio" << medDistRatio << endl;
    cout << "TTC Camera: " << TTC << endl;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    auto [prev_Q1, prev_Q2, prev_Q3] = quartilesOnXaxisLidarPoints(lidarPointsPrev);
    double prev_IQR = prev_Q3 - prev_Q1;
    double prev_low_outlier_thold = prev_Q1 - 1.5 * prev_IQR;
    double prev_high_outlier_thold = prev_Q3 + 1.5 * prev_IQR;
    cout << "PrevMedian on X axis for lidarpoints is " << prev_Q2 << endl;
    cout << "prev_Q1: " << prev_Q1 << ", prev_Q3: " << prev_Q3 << endl;
    cout << "prev_IQR: " << prev_IQR << endl;
    cout << "prev_low_outlier_thold: " << prev_low_outlier_thold << ", prev_high_outlier_thold: " << prev_high_outlier_thold << endl;

    auto [curr_Q1, curr_Q2, curr_Q3] = quartilesOnXaxisLidarPoints(lidarPointsCurr);
    double curr_IQR = curr_Q3 - curr_Q1;
    double curr_low_outlier_thold = curr_Q1 - 1.5 * curr_IQR;
    double curr_high_outlier_thold = curr_Q3 + 1.5 * curr_IQR;
    cout << "CurrMedian on X axis for lidarpoints is " << curr_Q2 << endl;
    cout << "curr_Q1: " << curr_Q1 << ", curr_Q3: " << curr_Q3 << endl;
    cout << "curr_IQR: " << curr_IQR << endl;
    cout << "curr_low_outlier_thold: " << curr_low_outlier_thold << ", curr_high_outlier_thold: " << curr_high_outlier_thold << endl;

    // auxiliary variables
    double dT = 1.0 / frameRate;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0 && it->x < prev_high_outlier_thold && it->x > prev_low_outlier_thold)
        { // 3D point within ego lane?
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
        }
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0 && it->x < curr_high_outlier_thold && it->x > curr_low_outlier_thold)
        { // 3D point within ego lane?
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
        }
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
    cout << "TTC Lidar: Min X prev: " << minXPrev << ", Min X curr: " << minXCurr << endl;
    cout << "TTC Lidar: (minXCurr * dT): " << (minXCurr * dT) << endl;
    cout << "TTC Lidar: (minXPrev - minXCurr): " << (minXPrev - minXCurr) << endl;
    cout << "TTC Lidar: " << TTC << endl;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    multimap<int,int> bbMatches;
    vector<int> queryBoxId, trainBoxId;
    for (cv::DMatch match: matches) {
        // Looking for the box of queryIdx
        for (BoundingBox bb: prevFrame.boundingBoxes){
            if (bb.roi.contains(prevFrame.keypoints[match.queryIdx].pt)) {
                queryBoxId.push_back(bb.boxID);
            }
        }

        // Looking for the box of trainIdx
        for (BoundingBox bb: currFrame.boundingBoxes){
            if (bb.roi.contains(currFrame.keypoints[match.trainIdx].pt)) {
                trainBoxId.push_back(bb.boxID);
            }
        }

        if (queryBoxId.size() >= 1 && trainBoxId.size() >= 1) {
            for (int qid: queryBoxId) {
                for(int tid: trainBoxId) {
                    bbMatches.insert({qid,tid});
                }
            }
        }
        queryBoxId.clear();
        trainBoxId.clear();
    }

    // Looking for best match between bounding boxes
    map<int, int> count_map;
    int i;
    multimap<int,int>::iterator search_boxID;
    map<int, int>::iterator search;

    for (BoundingBox bb: prevFrame.boundingBoxes){
        search_boxID = bbMatches.find(bb.boxID);
        i = 0;
        while (search_boxID != bbMatches.end() && search_boxID->first == bb.boxID){
            search = count_map.find(search_boxID->second);
            if (search != count_map.end()) {
                search->second = search->second + 1;
            } else {
                count_map.insert({search_boxID->second, 1});
            }
            search_boxID++;
            i++;
        }

        int max_occ = 0;
        int max_value = -1;
        for (const auto& [value, occ]: count_map) {
            if (occ > max_occ) {
                max_occ = occ;
                max_value = value;
            }
        }
        count_map.clear();

        if (max_value > -1) {
            cout << "Best match is boxID " << bb.boxID << " with boxId " << max_value << endl;
            bbBestMatches.insert({bb.boxID, max_value});
        }
    }
}
