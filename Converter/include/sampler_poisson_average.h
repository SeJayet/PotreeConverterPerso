
#pragma once

#include <execution>

#include "structures.h"
#include "Attributes.h"



struct SamplerPoissonAverage : public Sampler {

	// subsample a local octree from bottom up
	void sample(Node* node, Attributes attributes, double baseSpacing, 
		function<void(Node*)> onNodeCompleted,
		function<void(Node*)> onNodeDiscarded
	) {

		struct Point {
			double x;
			double y;
			double z;
			int32_t pointIndex;
			int32_t childIndex;

			int64_t r = 0;
			int64_t g = 0;
			int64_t b = 0;
			int64_t w = 0;

			int64_t mainIndex = 0;
			bool accepted = false;
		};

		function<void(Node*, function<void(Node*)>)> traversePost = [&traversePost](Node* node, function<void(Node*)> callback) {
			for (auto child : node->children) {

				if (child != nullptr && !child->sampled) {
					traversePost(child.get(), callback);
				}
			}

			callback(node);
		};

		const int bytesPerPoint = attributes.bytes;
		const Vector3 scale = attributes.posScale;
		const Vector3 offset = attributes.posOffset;

		traversePost(node, [bytesPerPoint, baseSpacing, scale, offset, &onNodeCompleted, &attributes](Node* node) {
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

			int numPointsInChildren = 0;
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

			for (int childIndex = 0; childIndex < 8; childIndex++) {
				auto child = node->children[childIndex];

				if (child == nullptr) {
					acceptedChildPointFlags.push_back({});
					numRejectedPerChild.push_back({});

					continue;
				}

				const bool childIsLeaf = child->isLeaf();

				const int offsetRGB = attributes.getOffset("rgb");
				//if (child->isLeaf()) {

				//	vector<CumulativeColor> colors;
				//	colors.reserve(node->numPoints);

				//	for (int i = 0; i < node->numPoints; i++) {
				//		CumulativeColor color;

				//		size_t offsetPoint = i * attributes.bytes;
				//		color.r = reinterpret_cast<uint16_t*>(node->points->data_u8 + offsetPoint + offsetRGB)[0];
				//		color.g = reinterpret_cast<uint16_t*>(node->points->data_u8 + offsetPoint + offsetRGB)[1];
				//		color.b = reinterpret_cast<uint16_t*>(node->points->data_u8 + offsetPoint + offsetRGB)[2];
				//		color.w = 1;

				//		colors.push_back(color);
				//	}

				//	node->colors = colors;
				//}


				vector<int8_t> acceptedFlags(child->numPoints, 0);
				acceptedChildPointFlags.push_back(acceptedFlags);

				for (int i = 0; i < child->numPoints; i++) {
					const int64_t pointOffset = i * attributes.bytes;
					const int32_t* xyz = reinterpret_cast<int32_t*>(child->points->data_u8 + pointOffset);

					const double x = (xyz[0] * scale.x) + offset.x;
					const double y = (xyz[1] * scale.y) + offset.y;
					const double z = (xyz[2] * scale.z) + offset.z;

					Point point = { x, y, z, i, childIndex };

					/*point.r = child->colors[i].r;
					point.g = child->colors[i].g;
					point.b = child->colors[i].b;
					point.w = child->colors[i].w;*/

					const size_t offsetPoint = i * attributes.bytes;
					point.r = reinterpret_cast<uint16_t*>(child->points->data_u8 + offsetPoint + offsetRGB)[0];
					point.g = reinterpret_cast<uint16_t*>(child->points->data_u8 + offsetPoint + offsetRGB)[1];
					point.b = reinterpret_cast<uint16_t*>(child->points->data_u8 + offsetPoint + offsetRGB)[2];
					point.w = 1;

					point.mainIndex = points.size();

					points.push_back(point);
				}

				child->colors = {};

			}

			const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

			//thread_local vector<Point> dbgAccepted(1'000'000);
			//int dbgNumAccepted = 0;
			const double spacing = baseSpacing / pow(2.0, node->level());
			const double squaredSpacing = spacing * spacing;

			auto squaredDistance = [](Point& a, Point& b) {
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

			constexpr double acceptGridSize = 16;
			vector<vector<Point>> gridAccepted(acceptGridSize * acceptGridSize * acceptGridSize);

			const auto checkAccept = [/*&dbgChecks, &dbgSumChecks, &dbgNumAccepted,*/ spacing, squaredSpacing, &squaredDistance, center, min, max, size, &gridAccepted, acceptGridSize](Point candidate) {

				const double dx = acceptGridSize * (candidate.x - min.x) / size.x;
				const double dy = acceptGridSize * (candidate.y - min.y) / size.y;
				const double dz = acceptGridSize * (candidate.z - min.z) / size.z;

				const double dx_min = acceptGridSize * (candidate.x - spacing - min.x) / size.x;
				const double dy_min = acceptGridSize * (candidate.y - spacing - min.y) / size.y;
				const double dz_min = acceptGridSize * (candidate.z - spacing - min.z) / size.z;

				const double dx_max = acceptGridSize * (candidate.x + spacing - min.x) / size.x;
				const double dy_max = acceptGridSize * (candidate.y + spacing - min.y) / size.y;
				const double dz_max = acceptGridSize * (candidate.z + spacing - min.z) / size.z;

				const int ix = std::max(std::min(dx, acceptGridSize - 1.0), 0.0);
				const int iy = std::max(std::min(dy, acceptGridSize - 1.0), 0.0);
				const int iz = std::max(std::min(dz, acceptGridSize - 1.0), 0.0);

				const int x_min = std::max(std::min(dx_min, acceptGridSize - 1.0), 0.0);
				const int y_min = std::max(std::min(dy_min, acceptGridSize - 1.0), 0.0);
				const int z_min = std::max(std::min(dz_min, acceptGridSize - 1.0), 0.0);

				const int x_max = std::max(std::min(dx_max, acceptGridSize - 1.0), 0.0);
				const int y_max = std::max(std::min(dy_max, acceptGridSize - 1.0), 0.0);
				const int z_max = std::max(std::min(dz_max, acceptGridSize - 1.0), 0.0);

				for (int x = x_min; x <= x_max; x++) {
				for (int y = y_min; y <= y_max; y++) {
				for (int z = z_min; z <= z_max; z++) {
					const int index = x + y * acceptGridSize + z * acceptGridSize * acceptGridSize;

					const auto& list = gridAccepted[index];

					for (auto point : list) {
						const double dd = squaredDistance(point, candidate);

						if (dd < squaredSpacing) {
							return false;
						}
					}

				}
				}
				}

				const int indexCurr = ix + iy * acceptGridSize + iz * acceptGridSize * acceptGridSize;
				gridAccepted[indexCurr].push_back(candidate);

				//=====================================================
				// outside-in distance checking as described in paper
				// superseded by distance checking in a grid-neighbourhood
				//=====================================================
				//auto cx = candidate.x - center.x;
				//auto cy = candidate.y - center.y;
				//auto cz = candidate.z - center.z;
				//auto cdd = cx * cx + cy * cy + cz * cz;
				//auto cd = sqrt(cdd);
				//auto limit = (cd - spacing);
				//auto limitSquared = limit * limit;

				//int j = 0;
				//for (int i = dbgNumAccepted - 1; i >= 0; i--) {

				//	auto& point = dbgAccepted[i];

				//	//dbgChecks++;
				//	//dbgSumChecks++;

				//	// check distance to center
				//	auto px = point.x - center.x;
				//	auto py = point.y - center.y;
				//	auto pz = point.z - center.z;
				//	auto pdd = px * px + py * py + pz * pz;
				//	//auto pd = sqrt(pdd);

				//	// stop when differences to center between candidate and accepted exceeds the spacing
				//	// any other previously accepted point will be even closer to the center.
				//	if (pdd < limitSquared) {
				//		return true;
				//	}

				//	double dd = squaredDistance(point, candidate);

				//	if (dd < squaredSpacing) {
				//		return false;
				//	}

				//	j++;

				//	// also put a limit at x distance checks
				//	if (j > 10'000) {
				//		return true;
				//	}
				//}

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
			vector<int32_t> mainToSortMapping(points.size());
			for (int i = 0; i < points.size(); i++) {
				mainToSortMapping[points[i].mainIndex] = i;
			}


			//int abc = 0;
			for (const Point& point : points) {

				//dbgChecks = 0;

				//point.mainIndex = abc;
				//abc++;

				const bool isAccepted = checkAccept(point);

				//dbgMaxChecks = std::max(dbgChecks, dbgMaxChecks);

				if (isAccepted) {
					//dbgAccepted[numAccepted] = point;
					//dbgNumAccepted++;
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

			{// compute average color
				const int offsetRGB = attributes.getOffset("rgb");

				const auto addCandidateToAverage = [node](Point& candidate, Point& average) {
					average.r = average.r + candidate.r;
					average.g = average.g + candidate.g;
					average.b = average.b + candidate.b;
					average.w = average.w + candidate.w;

				};


				for (Point& candidate : points) {
					const double dx = acceptGridSize * (candidate.x - min.x) / size.x;
					const double dy = acceptGridSize * (candidate.y - min.y) / size.y;
					const double dz = acceptGridSize * (candidate.z - min.z) / size.z;

					const double dx_min = acceptGridSize * (candidate.x - spacing - min.x) / size.x;
					const double dy_min = acceptGridSize * (candidate.y - spacing - min.y) / size.y;
					const double dz_min = acceptGridSize * (candidate.z - spacing - min.z) / size.z;

					const double dx_max = acceptGridSize * (candidate.x + spacing - min.x) / size.x;
					const double dy_max = acceptGridSize * (candidate.y + spacing - min.y) / size.y;
					const double dz_max = acceptGridSize * (candidate.z + spacing - min.z) / size.z;

					const int ix = std::max(std::min(dx, acceptGridSize - 1.0), 0.0);
					const int iy = std::max(std::min(dy, acceptGridSize - 1.0), 0.0);
					const int iz = std::max(std::min(dz, acceptGridSize - 1.0), 0.0);

					const int x_min = std::max(std::min(dx_min, acceptGridSize - 1.0), 0.0);
					const int y_min = std::max(std::min(dy_min, acceptGridSize - 1.0), 0.0);
					const int z_min = std::max(std::min(dz_min, acceptGridSize - 1.0), 0.0);

					const int x_max = std::max(std::min(dx_max, acceptGridSize - 1.0), 0.0);
					const int y_max = std::max(std::min(dy_max, acceptGridSize - 1.0), 0.0);
					const int z_max = std::max(std::min(dz_max, acceptGridSize - 1.0), 0.0);

					for (int x = x_min; x <= x_max; x++) {
						for (int y = y_min; y <= y_max; y++) {
							for (int z = z_min; z <= z_max; z++) {
								const int index = x + y * acceptGridSize + z * acceptGridSize * acceptGridSize;

								auto& list = gridAccepted[index];

								for (auto& point : list) {
									const double dd = squaredDistance(point, candidate);

									if (dd < squaredSpacing) {

										
										addCandidateToAverage(candidate, points[mainToSortMapping[point.mainIndex]]);

									}
								}

							}
						}
					}
				}
			}

			auto accepted = make_shared<Buffer>(numAccepted * attributes.bytes);
			vector<CumulativeColor> averagedColors;
			averagedColors.reserve(numAccepted);

			int offsetRGB = attributes.getOffset("rgb");
			size_t j = 0;
			for (int childIndex = 0; childIndex < 8; childIndex++) {
				auto child = node->children[childIndex];

				if (child == nullptr) {
					continue;
				}

				const auto numRejected = numRejectedPerChild[childIndex];
				auto& acceptedFlags = acceptedChildPointFlags[childIndex];
				auto rejected = make_shared<Buffer>(child->numPoints * attributes.bytes);

				for (int i = 0; i < child->numPoints; i++) {
					auto isAccepted = acceptedFlags[i];
					int64_t pointOffset = i * attributes.bytes;

					
					const Point& p = points[mainToSortMapping[j]];

					uint16_t* rgbTarget = reinterpret_cast<uint16_t*>(child->points->data_u8 + i * attributes.bytes + offsetRGB);
					rgbTarget[0] = p.r / p.w;
					rgbTarget[1] = p.g / p.w;
					rgbTarget[2] = p.b / p.w;

					if (isAccepted) {
						accepted->write(child->points->data_u8 + pointOffset, attributes.bytes);

						CumulativeColor color;
						color.r = p.r;
						color.g = p.g;
						color.b = p.b;
						color.w = p.w;
						averagedColors.push_back(color);

						rejected->write(child->points->data_u8 + pointOffset, attributes.bytes);
					} else {
						rejected->write(child->points->data_u8 + pointOffset, attributes.bytes);
					}

					j++;
				}

				if (numRejected == 0) {
					node->children[childIndex] = nullptr;
				} if (numRejected > 0) {
					child->points = rejected;
					//child->numPoints = numRejected;
					//child->colors.clear();
					//child->colors.shrink_to_fit();

					onNodeCompleted(child.get());
				}
			}

			


			node->points = accepted;
			node->colors = averagedColors;
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

