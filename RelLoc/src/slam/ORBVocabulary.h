#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "DBoWHeader.h"

namespace ORB_SLAM2 {

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

}  // namespace ORB_SLAM2

#endif  // ORBVOCABULARY_H
