#include "Map.h"

#include <mutex>

namespace ORB_SLAM2 {

Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {}

void Map::AddKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.insert(pKF);
  if (pKF->mnId > mnMaxKFid) mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.erase(pKF);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

vector<KeyFrame *> Map::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<MapPoint *> Map::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

vector<MapPoint *> Map::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMaxKFid;
}

void Map::clear() {
  for (set<MapPoint *>::iterator sit = mspMapPoints.begin(),
                                 send = mspMapPoints.end();
       sit != send; sit++)
    delete *sit;

  for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(),
                                 send = mspKeyFrames.end();
       sit != send; sit++)
    delete *sit;

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = 0;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

}  // namespace ORB_SLAM2
