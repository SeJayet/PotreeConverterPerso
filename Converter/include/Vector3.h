
#pragma once

#include <string>
#include <cmath>
#include <limits>
#include <sstream>
#include <algorithm>
#include <string>

using std::string;

struct Vector3{

	double x = double(0.0);
	double y = double(0.0);
	double z = double(0.0);

	Vector3() {

	}

	Vector3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3(double value[3]) {
		this->x = value[0];
		this->y = value[1];
		this->z = value[2];
	}

	double squaredDistanceTo(const Vector3& right) {
		const double dx = right.x - x;
		const double dy = right.y - y;
		const double dz = right.z - z;

		const double dd = dx * dx + dy * dy + dz * dz;

		return dd;
	}

	double distanceTo(const Vector3& right) {
		const double dx = right.x - x;
		const double dy = right.y - y;
		const double dz = right.z - z;

		const double dd = dx * dx + dy * dy + dz * dz;
		const double d = std::sqrt(dd);

		return d;
	}

	double length() {
		return sqrt(x * x + y * y + z * z);
	}

	double max() {
		const double value = std::max(std::max(x, y), z);
		return value;
	}

	Vector3 operator-(const Vector3& right) const {
		return Vector3(x - right.x, y - right.y, z - right.z);
	}

	Vector3 operator+(const Vector3& right) const {
		return Vector3(x + right.x, y + right.y, z + right.z);
	}

	Vector3 operator+(const double& scalar) const {
		return Vector3(x + scalar, y + scalar, z + scalar);
	}

	Vector3 operator/(const double& scalar) const {
		return Vector3(x / scalar, y / scalar, z / scalar);
	}

	Vector3 operator*(const Vector3& right) const {
		return Vector3(x * right.x, y * right.y, z * right.z);
	}

	Vector3 operator*(const double& scalar) const {
		return Vector3(x * scalar, y * scalar, z * scalar);
	}

	string toString() {
		constexpr auto digits = std::numeric_limits<double>::max_digits10;

		std::stringstream ss;
		ss << std::setprecision(digits);
		ss << x << ", " << y << ", " << z;

		return ss.str();
	}

};