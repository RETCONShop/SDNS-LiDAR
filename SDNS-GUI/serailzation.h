/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        serialization.h
// Purpose:     An old library meant for interfacing 'Cereal' with the SDNS-GUI project
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.24.24 5:35 PM MT: Comments added
//
// Copyright 2024 Andrew Gallagher
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/////////////////////////////////////////////////////////////////////////////////////////////////////


// Note to future teams: This class serves no purpose, but I left it here for you to see how the 'cereal' library works.
// Avoid using 'cereal' at all costs. It's a terrible library, and a massive headache to work with. 
#ifndef SERAILZATION_H
#define SERAILZATION_H

// Cereal support
#include <cereal/archives/binary.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <sstream>
#include "StructsAndStuff.h"

template<class Archive>
void serialize(Archive & archive, FunctionInfo & m) {
    archive(m.functionID, m.hasCallback, m.callbackContents);
}

std::string serializeToMemory(const FunctionInfo& data) {
    std::stringstream ss;
    cereal::BinaryOutputArchive archive(ss);
    archive(data);
    return ss.str();
}

void deserializeFromMemory(const std::string& serializedData, FunctionInfo& result) {
    std::stringstream ss(serializedData);
    cereal::BinaryInputArchive archive(ss);
    archive(result);
}
#endif // SERAILZATION_H
