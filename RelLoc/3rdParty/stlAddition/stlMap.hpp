
/**
 * @file stlMap.h
 * @brief Functor to select key or value of stl maps
 * @author Pierre MOULON
 *
 * Copyright (c) 2011, 2012, 2013 Pierre MOULON
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef STL_MAP_ADDITION_H
#define STL_MAP_ADDITION_H

// ---------------------------
// Usage example :
// ---------------------------

// std::map<int, double> m;
// m[0] = 2.0;
// m[1] = 1.6;
// std::vector<int> keys;
// // Retrieve all keys
// transform(m.begin(), m.end(), back_inserter(keys), RetrieveKey());
// // Log all keys to console
// copy(keys.begin(), keys.end(), ostream_iterator<int>(cout, "\n"));
// // Retrieve all values
// std::vector<double> values;
// // Retrieve all values
// transform(m.begin(), m.end(), back_inserter(values), RetrieveValue());

#include <map>
#include <set>

namespace std{

/// Allow to select the Keys of a map.
struct RetrieveKey
{
    template <typename T>
    typename T::first_type operator()(const T & keyValuePair) const
    {
        return keyValuePair.first;
    }
};

/// Allow to select the Values of a map.
struct RetrieveValue
{
    template <typename T>
    typename T::second_type operator()(const T & keyValuePair) const
    {
        return keyValuePair.second;
    }
};


template<typename keyType, typename leftValue, typename rightValue>
bool IntersectMaps(const map<keyType, leftValue>& left,
                   const map<keyType, rightValue>& right,
                   map<keyType, leftValue>& result)
{
    if (left.empty() || right.empty()) {
        return false;
    }
    typename map<keyType, leftValue>::const_iterator itrL = left.begin();
    typename map<keyType, rightValue>::const_iterator itrR = right.begin();
    while (itrL != left.end() && itrR != right.end())
    {
        if (itrL->first < itrR->first) {
            ++itrL;
        } else if (itrR->first < itrL->first) {
            ++itrR;
        } else
        {
            if (itrL->second == itrR->second)
            {
            //    std::cout << "idx = " << itrL->second << ", " << itrR->second << std::endl;
                result.insert(make_pair(itrL->first, itrL->second));
            }
            ++itrL;
            ++itrR;
        }
    }
    return true;
}

template<typename keyType, typename leftValue, typename rightValue>
bool DifferenceMaps(const map<keyType, leftValue>& left,
                    const map<keyType, rightValue>& right,
                    map<keyType, leftValue>& result)
{
    if (left.empty() || right.empty()) {
        return false;
    }
    typename map<keyType, leftValue>::const_iterator itrL = left.begin();
    typename map<keyType, rightValue>::const_iterator itrR = right.begin();
    while (itrL != left.end() && itrR != right.end())
    {
        if (itrL->first < itrR->first) {
            ++itrL;
            result.insert(make_pair(itrL->first, itrL->second));
        } else if (itrR->first < itrL->first) {
            ++itrR;
        } else{
            ++itrL;
            ++itrR;
        }
    }
    return true;
}

template<typename keyType, typename Value>
bool intersectionSetMap(const set<keyType>& refSet,
                         const map<keyType, Value>& refMap,
                         set<keyType>& result)
{
    if (refSet.empty() || refMap.empty()) {
        return false;
    }
    typename set<keyType>::const_iterator itset = refSet.begin();
    typename map<keyType, Value>::const_iterator itmap = refMap.begin();

    while (itset != refSet.end() && itmap != refMap.end())
    {
        if (*itset < itmap->first) ++itset;
        else if (itmap->first < *itset) ++itmap;
        else {
            result.insert(*itset);
            ++itset; ++itmap;
        }
    }
    return true;
}

template<typename keyType, typename Value>
bool differenceSetMap (const set<keyType>& refSet,
                       const map<keyType, Value>& refMap,
                       set<keyType>& result)
{
    if (refSet.empty() || refMap.empty()) {
        return false;
    }
    typename set<keyType>::const_iterator itset = refSet.begin();
    typename map<keyType, Value>::const_iterator itmap = refMap.begin();

    while (itset != refSet.end() && itmap != refMap.end())
    {
        if (*itset < itmap->first)
        {
            result.insert(*itset);
            ++itset; ++itmap;
        }
        else if (itmap->first < *itset) ++itmap;
        else {
            ++itset; ++itmap;
        }
    }
    return true;
}

} // namespace std

#endif // STL_MAP_ADDITION_H

