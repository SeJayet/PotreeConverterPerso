#pragma once

#include <execution>

#include "Vector3.h"
#include "LasLoader/LasLoader.h"

struct ScaleOffset {
	Vector3 scale;
	Vector3 offset;
};


inline ScaleOffset computeScaleOffset(Vector3 min, Vector3 max, Vector3 targetScale) {
	// using the center as the origin would be the "right" choice but
	// it would lead to negative integer coordinates.
	// since the Potree 1.7 release mistakenly interprets the coordinates as uint values,
	// we can't do that and we use 0/0/0 as the bounding box minimum as the origin instead.
	//const Vector3 center = (min + max) / 2.0;
	//Vector3 offset = center;

	const Vector3 size = max - min;

	// we can only use 31 bits because of the int/uint mistake in Potree 1.7
	// And we only use 30 bits to be on the safe side.
	constexpr auto interval_30_bits = 0b1'000'000'000'000'000'000'000'000'000'000; // 30 '0's
	const double min_scale_x = size.x / interval_30_bits;
	const double min_scale_y = size.y / interval_30_bits;
	const double min_scale_z = size.z / interval_30_bits;

	ScaleOffset scaleOffset;
	scaleOffset.scale = {
		std::max(targetScale.x, min_scale_x),
		std::max(targetScale.y, min_scale_y),
		std::max(targetScale.z, min_scale_z),
	};
	scaleOffset.offset = min;

	return scaleOffset;
}



inline vector<Attribute> parseExtraAttributes(const LasHeader& header) {

	// vector<uint8_t> extraData;
	vector<Attribute> attributes;

	for (auto && vlr : header.vlrs) {
		if (vlr.recordID == 4) {
			auto extraData = vlr.data;

			constexpr int recordSize = 192;
			const int numExtraAttributes = extraData.size() / recordSize;

			for (int i = 0; i < numExtraAttributes; i++) {

				const int offset = i * recordSize;
				const uint8_t type = read<uint8_t>(extraData, offset + 2);
				const uint8_t options = read<uint8_t>(extraData, offset + 3);

				char chrName[32];
				memcpy(chrName, extraData.data() + offset + 4, 32);
				string name(chrName);

				Vector3 aScale = {1.0, 1.0, 1.0};
				Vector3 aOffset = {0.0, 0.0, 0.0};
				if((options & 0b01000) != 0){
					memcpy(&aScale, extraData.data() + offset + 112, 24);
				}
				if((options & 0b10000) != 0){
					memcpy(&aOffset, extraData.data() + offset + 136, 24);
				}

				char chrDescription[32];
				memcpy(chrDescription, extraData.data() + offset + 160, 32);
				string description(chrDescription);

				const auto info = lasTypeInfo(type);
				string typeName = getAttributeTypename(info.type);
				const int elementSize = getAttributeTypeSize(info.type);

				const int size = info.numElements * elementSize;
				Attribute xyz(name, size, info.numElements, elementSize, info.type);
				xyz.description = description;
				xyz.scale = aScale;
				xyz.offset = aOffset;

				attributes.push_back(xyz);
			}
		}
	}

	return attributes;
}


inline vector<Attribute> computeOutputAttributes(const LasHeader& header) {
	const auto format = header.pointDataFormat;

	Attribute xyz("position", 12, 3, 4, AttributeType::INT32);
	Attribute intensity("intensity", 2, 1, 2, AttributeType::UINT16);
	Attribute returns("returns", 1, 1, 1, AttributeType::UINT8);
	Attribute returnNumber("return number", 1, 1, 1, AttributeType::UINT8);
	Attribute numberOfReturns("number of returns", 1, 1, 1, AttributeType::UINT8);
	Attribute classification("classification", 1, 1, 1, AttributeType::UINT8);
	Attribute scanAngleRank("scan angle rank", 1, 1, 1, AttributeType::UINT8);
	Attribute userData("user data", 1, 1, 1, AttributeType::UINT8);
	Attribute pointSourceId("point source id", 2, 1, 2, AttributeType::UINT16);
	Attribute gpsTime("gps-time", 8, 1, 8, AttributeType::DOUBLE);
	Attribute rgb("rgb", 6, 3, 2, AttributeType::UINT16);
	Attribute wavePacketDescriptorIndex("wave packet descriptor index", 1, 1, 1, AttributeType::UINT8);
	Attribute byteOffsetToWaveformData("byte offset to waveform data", 8, 1, 8, AttributeType::UINT64);
	Attribute waveformPacketSize("waveform packet size", 4, 1, 4, AttributeType::UINT32);
	Attribute returnPointWaveformLocation("return point waveform location", 4, 1, 4, AttributeType::FLOAT);
	Attribute XYZt("XYZ(t)", 12, 3, 4, AttributeType::FLOAT);
	Attribute classificationFlags("classification flags", 1, 1, 1, AttributeType::UINT8);
	Attribute scanAngle("scan angle", 2, 1, 2, AttributeType::INT16);

	vector<Attribute> list;

	if (format == 0) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classification, scanAngleRank, userData, pointSourceId };
	} else if (format == 1) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classification, scanAngleRank, userData, pointSourceId, gpsTime };
	} else if (format == 2) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classification, scanAngleRank, userData, pointSourceId, rgb };
	} else if (format == 3) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classification, scanAngleRank, userData, pointSourceId, gpsTime, rgb };
	} else if (format == 4) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classification, scanAngleRank, userData, pointSourceId, gpsTime,
			wavePacketDescriptorIndex, byteOffsetToWaveformData, waveformPacketSize, returnPointWaveformLocation,
			XYZt
		};
	} else if (format == 5) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classification, scanAngleRank, userData, pointSourceId, gpsTime, rgb,
			wavePacketDescriptorIndex, byteOffsetToWaveformData, waveformPacketSize, returnPointWaveformLocation,
			XYZt
		};
	} else if (format == 6) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classificationFlags, classification, userData, scanAngle, pointSourceId, gpsTime };
	} else if (format == 7) {
		list = { xyz, intensity, returnNumber, numberOfReturns, classificationFlags, classification, userData, scanAngle, pointSourceId, gpsTime, rgb };
	} else {
		cout << "ERROR: currently unsupported LAS format: " << int(format) << endl;

		exit(123);
	}

	vector<Attribute> extraAttributes = parseExtraAttributes(header);

	list.insert(list.end(), extraAttributes.begin(), extraAttributes.end());

	return list;
}

inline Attributes computeOutputAttributes(const vector<Source>& sources, vector<string> requestedAttributes) {
	// TODO: a bit wasteful to iterate over source files and load headers twice

	Vector3 scaleMin = { Infinity, Infinity, Infinity };
	Vector3 min = { Infinity, Infinity, Infinity };
	Vector3 max = { -Infinity, -Infinity, -Infinity };
	Vector3 scale, offset;

	vector<Attribute> fullAttributeList;
	unordered_map<string, int> acceptedAttributeNames;

	// compute scale and offset from all sources
	{
		mutex mtx;
		constexpr auto parallel = std::execution::par;
		for_each(parallel, sources.begin(), sources.end(), [&mtx, &scaleMin, &min, &max, &fullAttributeList, &acceptedAttributeNames](const Source &source) {

			const auto header = loadLasHeader(source.path);

			vector<Attribute> attributes = computeOutputAttributes(header);

			mtx.lock();

			for (auto && attribute : attributes) {
				bool alreadyAdded = acceptedAttributeNames.find(attribute.name) != acceptedAttributeNames.end();

				if (!alreadyAdded) {
					fullAttributeList.push_back(attribute);
					acceptedAttributeNames[attribute.name] = 1;
				}
			}

			scaleMin.x = std::min(scaleMin.x, header.scale.x);
			scaleMin.y = std::min(scaleMin.y, header.scale.y);
			scaleMin.z = std::min(scaleMin.z, header.scale.z);

			min.x = std::min(min.x, header.min.x);
			min.y = std::min(min.y, header.min.y);
			min.z = std::min(min.z, header.min.z);

			max.x = std::max(max.x, header.max.x);
			max.y = std::max(max.y, header.max.y);
			max.z = std::max(max.z, header.max.z);

			mtx.unlock();
			});

		auto scaleOffset = computeScaleOffset(min, max, scaleMin);
		scale = scaleOffset.scale;
		offset = scaleOffset.offset;

		if (scaleMin.x != scale.x || scaleMin.y != scale.y || scaleMin.z != scale.z) {
			cout << "WARNING: " << "scale/offset/bounding box were adjusted. "
				<< "new scale: " << scale.toString() << ", "
				<< "new offset: " << offset.toString() << endl;
		}
	}

	// filter down to optionally specified attributes
	vector<Attribute> filteredAttributeList = fullAttributeList;
	if (requestedAttributes.size() > 0) {
		auto should = requestedAttributes;
		auto is = fullAttributeList;

		// always add position
		should.insert(should.begin(), { "position" });

		vector<Attribute> filtered;

		for (string name : should) {
			auto it = find_if(is.begin(), is.end(), [name](auto& value) {
				return value.name == name;
				});

			if (it != is.end()) {
				filtered.push_back(*it);
			}
		}

		filteredAttributeList = filtered;
	}

	auto attributes = Attributes(filteredAttributeList);
	attributes.posScale = scale;
	attributes.posOffset = offset;

	return attributes;
}

inline string toString(const Attributes& attributes){

	stringstream ss;

	ss << endl << "output attributes: " << endl;

	constexpr int c0 = 30;
	constexpr int c1 = 10;
	constexpr int c2 = 8;
	constexpr int ct = c0 + c1 + c2;

	ss << rightPad("name", c0) << leftPad("offset", c1) << leftPad("size", c2) << endl;
	ss << string(ct, '=') << endl;

	int offset = 0;
	for (auto attribute : attributes.list) {
		ss << rightPad(attribute.name, c0)
			<< leftPad(formatNumber(offset), c1)
			<< leftPad(formatNumber(attribute.size), c2)
			<< endl;

		offset += attribute.size;
	}
	ss << string(ct, '=') << endl;

#ifdef _DEBUG
	cout << "bytes per point: " << attributes.bytes << endl;
#endif // _DEBUG
	ss << leftPad(formatNumber(attributes.bytes), ct) << endl;
	ss << string(ct, '=') << endl;

	return ss.str();
}



