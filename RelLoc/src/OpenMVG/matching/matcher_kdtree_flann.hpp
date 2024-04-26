// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_ARRAYMATCHER_KDTREE_FLANN_H_
#define OPENMVG_MATCHING_ARRAYMATCHER_KDTREE_FLANN_H_

#include "../matching/matching_interface.hpp"
#include "3rdParty/flann/src/cpp/flann/flann.hpp"
#include <memory>
#include <ctime>

namespace openMVG {
namespace matching  {

/// Implement ArrayMatcher as a FLANN KDtree matcher.
// http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
// David G. Lowe and Marius Muja
//
// By default use squared L2 metric (flann::L2<Scalar>)
// sqrt is monotonic so for performance reason we do not compute it.

template < typename Scalar = float, typename  Metric = flann::L2<Scalar> >
class ArrayMatcher_Kdtree_Flann : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;

  ArrayMatcher_Kdtree_Flann() {}

  virtual ~ArrayMatcher_Kdtree_Flann()  {}

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the each
   *  row of the dataset.
   *
   * \return True if success.
   */
  bool Build( const Scalar * dataset, int nbRows, int dimension)  {

    if (nbRows > 0)
    {
      _dimension = dimension;
      //-- Build Flann Matrix container (map to already allocated memory)
      _datasetM = auto_ptr< flann::Matrix<Scalar> >(
          new flann::Matrix<Scalar>((Scalar*)dataset, nbRows, dimension));

      //-- Build FLANN index
      _index = auto_ptr< flann::Index<Metric> > (
          new flann::Index<Metric> (*_datasetM, flann::KDTreeIndexParams(4)) );
      (*_index).buildIndex();

      return true;
    }
    return false;
  }

  /**
   * Search the nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[out]  indice    The indice of array in the dataset that
   *  have been computed as the nearest array.
   * \param[out]  distance  The distance between the two arrays.
   *
   * \return True if success.
   */
  bool SearchNeighbour( const Scalar * query, int * indice, DistanceType * distance)
  {

    if (_index.get() != NULL)  {

      int * indicePTR = indice;
      float * distancePTR = distance;
      flann::Matrix<Scalar> queries((Scalar*)query, 1, _dimension);

      flann::Matrix<int> indices(indicePTR, 1, 1);
      flann::Matrix<float> dists(distancePTR, 1, 1);
      // do a knn search, using 128 checks

      (*_index).knnSearch(queries, indices, dists, 1, flann::SearchParams(128));

      return true;
    }
    else  {
      return false;
    }
  }


/**
   * Search the N nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[in]   nbQuery   The number of query rows
   * \param[out]  indice    The indices of arrays in the dataset that
   *  have been computed as the nearest arrays.
   * \param[out]  distance  The distances between the matched arrays.
   * \param[out]  NN        The number of maximal neighbor that could
   *  will be searched.
   *
   * \return True if success.
   */
  bool SearchNeighbours( const Scalar * query, int nbQuery,
                        vector<int> * pvec_indice,
                        vector<DistanceType> * pvec_distance,
                        size_t NN)
  {
//      std::cout <<"SearchNeighbour" << std::endl;
//    clock_t t1 = clock();
    if (_index.get() != NULL)  {
//        clock_t t2 = clock();
      //-- Check if resultIndices is allocated
      pvec_indice->resize(nbQuery * NN);
      pvec_distance->resize(nbQuery * NN);

      int * indicePTR = &((*pvec_indice)[0]);
      float * distancePTR = &(*pvec_distance)[0];
      flann::Matrix<Scalar> queries((Scalar*)query, nbQuery, _dimension);

      flann::Matrix<int> indices(indicePTR, nbQuery, NN);
      flann::Matrix<float> dists(distancePTR, nbQuery, NN);
      // do a knn search, using 128 checks
//      clock_t t3 = clock();
      (*_index).knnSearch(queries, indices, dists, NN, flann::SearchParams(128));
//      clock_t t4 = clock();
//      std::cout << "  t1: " << double(t2-t1)/CLOCKS_PER_SEC
//                << "  t2: " << double(t3-t2)/CLOCKS_PER_SEC
//                << "  t3: " << double(t4-t3)/CLOCKS_PER_SEC
//                << std::endl;
      return true;
    }
    else  {
      return false;
    }
  }

  private :

  auto_ptr< flann::Matrix<Scalar> > _datasetM;
  auto_ptr< flann::Index<Metric> > _index;
  size_t _dimension;
};

} // namespace matching
} // namespace openMVG

#endif // OPENMVG_MATCHING_ARRAYMATCHER_KDTREE_FLANN_H_
