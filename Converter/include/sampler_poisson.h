
#pragma once

#include <execution>

#include "structures.h"
#include "Attributes.h"
#include "PotreeConverter.h"


struct SamplerPoisson : public Sampler {

	// subsample a local octree from bottom up
	void sample(Node* node, Attributes attributes, double baseSpacing, 
		function<void(Node*)> onNodeCompleted, 
		function<void(Node*)> onNodeDiscarded
	) override {

		struct Point {
			double x;
			double y;
			double z;
			int32_t pointIndex;
			int32_t childIndex;
		};

		function<void(Node*, function<void(Node*)>)> traversePost = [&traversePost](Node* node, function<void(Node*)> callback) {
			for (auto child : node->children) {

				if (child != nullptr && !child->sampled) {
					traversePost(child.get(), callback);
				}
			}

			callback(node);
		};

		const int64_t bytesPerPoint = attributes.bytes;
		const Vector3 scale = attributes.posScale;
		const Vector3 offset = attributes.posOffset;

		traversePost(node, [bytesPerPoint, baseSpacing, scale, offset, &onNodeCompleted, &onNodeDiscarded, attributes](Node* node) {
			node->sampled = true;

			const int64_t numPoints = node->numPoints;

			const auto max = node->max;
			const auto min = node->min;
			const auto size = max - min;
			const auto scale = attributes.posScale;
			const auto offset = attributes.posOffset;

			const bool isLeaf = node->isLeaf();

			if (isLeaf) {
				return false;
			}

			// =================================================================
			// SAMPLING
			// =================================================================
			//
			// first, check for each point whether it's accepted or rejected
			// save result in an array with one element for each point

			int64_t numPointsInChildren = 0;
			for (auto child : node->children) {
				if (child == nullptr) {
					continue;
				}

				numPointsInChildren += child->numPoints;
			}

			vector<Point> points;
			points.reserve(numPointsInChildren);

			vector<vector<int8_t>> acceptedChildPointFlags;
			vector<int64_t> numRejectedPerChild(8, 0);
			int64_t numAccepted = 0;

			for (int32_t childIndex = 0; childIndex < 8; childIndex++) {
				auto child = node->children[childIndex];

				if (child == nullptr) {
					acceptedChildPointFlags.push_back({});
					numRejectedPerChild.push_back({});

					continue;
				}

				vector<int8_t> acceptedFlags(child->numPoints, 0);
				acceptedChildPointFlags.push_back(acceptedFlags);

				for (int64_t i = 0; i < child->numPoints; i++) {
					const int64_t pointOffset = i * attributes.bytes;
					const int32_t* xyz = reinterpret_cast<int32_t*>(child->points->data_u8 + pointOffset);

					const double x = (xyz[0] * scale.x) + offset.x;
					const double y = (xyz[1] * scale.y) + offset.y;
					const double z = (xyz[2] * scale.z) + offset.z;

					Point point = { x, y, z, i & 0xFFFF'FFFF, childIndex };

					points.push_back(point);
				}

			}

			const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

			thread_local vector<Point> dbgAccepted(1'000'000);
			int64_t dbgNumAccepted = 0;
			const double spacing = baseSpacing / pow(2.0, node->level());
			const double squaredSpacing = spacing * spacing;

			const auto squaredDistance = [](Point& a, Point& b) {
				const double dx = a.x - b.x;
				const double dy = a.y - b.y;
				const double dz = a.z - b.z;

				const double dd = dx * dx + dy * dy + dz * dz;

				return dd;
			};

			const auto center = (node->min + node->max) * 0.5;

			//int dbgChecks = -1;
			//int dbgSumChecks = 0;
			//int dbgMaxChecks = 0;

			const auto checkAccept = [/*&dbgChecks, &dbgSumChecks,*/ &dbgNumAccepted, spacing, squaredSpacing, &squaredDistance, center /*, &numDistanceChecks*/](Point candidate) {

				const auto cx = candidate.x - center.x;
				const auto cy = candidate.y - center.y;
				const auto cz = candidate.z - center.z;
				const auto cdd = cx * cx + cy * cy + cz * cz;
				const auto cd = sqrt(cdd);
				const auto limit = (cd - spacing);
				const auto limitSquared = limit * limit;

				int64_t j = 0;
				for (int64_t i = dbgNumAccepted - 1; i >= 0; i--) {

					auto& point = dbgAccepted[i];

					//dbgChecks++;
					//dbgSumChecks++;

					// check distance to center
					const auto px = point.x - center.x;
					const auto py = point.y - center.y;
					const auto pz = point.z - center.z;
					const auto pdd = px * px + py * py + pz * pz;
					//auto pd = sqrt(pdd);

					// stop when differences to center between candidate and accepted exceeds the spacing
					// any other previously accepted point will be even closer to the center.
					if (pdd < limitSquared) {
						return true;
					}

					const double dd = squaredDistance(point, candidate);

					if (dd < squaredSpacing) {
						return false;
					}

					j++;

					// also put a limit at x distance checks
					if (j > 10'000) {
						return true;
					}
				}

				return true;

			};

			constexpr auto parallel = std::execution::par_unseq;
			std::sort(parallel, points.begin(), points.end(), [center](Point a, Point b) -> bool {

				const auto ax = a.x - center.x;
				const auto ay = a.y - center.y;
				const auto az = a.z - center.z;
				const auto add = ax * ax + ay * ay + az * az;

				const auto bx = b.x - center.x;
				const auto by = b.y - center.y;
				const auto bz = b.z - center.z;
				const auto bdd = bx * bx + by * by + bz * bz;

				// sort by distance to center
				return add < bdd;

				// sort by manhattan distance to center
				//return (ax + ay + az) < (bx + by + bz);

				// sort by z axis
				//return a.z < b.z;
			});

			for (Point point : points) {

				//dbgChecks = 0;

				const bool isAccepted = checkAccept(point);

				//dbgMaxChecks = std::max(dbgChecks, dbgMaxChecks);

				if (isAccepted) {
					dbgAccepted[dbgNumAccepted] = point;
					dbgNumAccepted++;
					numAccepted++;
				} else {
					numRejectedPerChild[point.childIndex]++;
				}

				//{ // debug: store sample time in GPS time attribute
				//		auto child = node->children[point.childIndex];
				//		auto data = child->points->data_u8;

				//		//static double value = 0.0;


				//		//value += 0.1;

				//		//if(node->level() <= 2){
				//			std::lock_guard<mutex> lock(mtx_debug);

				//			debug += 0.01;
				//			memcpy(data + point.pointIndex * attributes.bytes + 20, &debug, 8);
				//		//}
				//		//double value = now();

				//		//point.pointIndex
				//}

				acceptedChildPointFlags[point.childIndex][point.pointIndex] = isAccepted ? 1 : 0;

				//abc++;

			}

			auto accepted = make_shared<Buffer>(numAccepted * attributes.bytes);
			for (int64_t childIndex = 0; childIndex < 8; childIndex++) {
				auto child = node->children[childIndex];

				if (child == nullptr) {
					continue;
				}

				auto numRejected = numRejectedPerChild[childIndex];
				auto& acceptedFlags = acceptedChildPointFlags[childIndex];
				auto rejected = make_shared<Buffer>(numRejected * attributes.bytes);

				for (int64_t i = 0; i < child->numPoints; i++) {
					auto isAccepted = acceptedFlags[i];
					int64_t pointOffset = i * attributes.bytes;

					if (isAccepted) {
						accepted->write(child->points->data_u8 + pointOffset, attributes.bytes);
						// rejected->write(child->points->data_u8 + pointOffset, attributes.bytes);
					} else {
						rejected->write(child->points->data_u8 + pointOffset, attributes.bytes);
					}
				}

				if (numRejected == 0 && child->isLeaf()) {
					onNodeDiscarded(child.get());

					node->children[childIndex] = nullptr;
				} if (numRejected > 0) {
					child->points = rejected;
					child->numPoints = numRejected;

					onNodeCompleted(child.get());
				} else if(numRejected == 0) {
					// the parent has taken all points from this child, 
					// so make this child an empty inner node.
					// Otherwise, the hierarchy file will claim that 
					// this node has points but because it doesn't have any,
					// decompressing the nonexistent point buffer fails
					// https://github.com/potree/potree/issues/1125
					child->points = nullptr;
					child->numPoints = 0;
					onNodeCompleted(child.get());
				}
			}

			node->points = accepted;
			node->numPoints = numAccepted;

			//{ // debug
			//	auto avgChecks = dbgSumChecks / points.size();
			//	string msg = "#checks: " + formatNumber(dbgSumChecks) + ", maxChecks: " + formatNumber(dbgMaxChecks) + ", avgChecks: " + formatNumber(avgChecks) + "\n";
			//	cout << msg;
			//}

			return true;
		});
	}

};

