
#pragma once

#include <string>
#include <vector>

#include "Vector3.h"
#include "Attributes.h"
#include "Monitor.h"

using std::string;
using std::vector;

class Source;
class State;

namespace chunker_countsort_laszip {

	void doChunking(const vector<Source>& sources, const string &targetDir, Vector3 min, Vector3 max, State& state, Attributes& outputAttributes);

}