#include "dab_xsens_2_fbx_manager.h"
#include <stdexcept>
#include <cmath>

using namespace dab;
using namespace dab::xsens;

# pragma mark Xsens2FbxManager definition

int Xsens2FbxManager::sJointCount = 23;
std::vector<int> Xsens2FbxManager::sJointParents = { -1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 14, 15, 12, 17, 18, 19, 12, 21 };

Xsens2FbxManager::Xsens2FbxManager() :
	mJointCount(sJointCount),
	mJointParents(sJointParents)
{}

Xsens2FbxManager::~Xsens2FbxManager()
{
}

int
Xsens2FbxManager::getJointCount() const
{
	return mJointCount;
}

std::array<float, 4>
Xsens2FbxManager::qconjugate(const std::array<float, 4>& pQ) const
{
	return { pQ[0], -pQ[1], -pQ[2], -pQ[3] };
}

std::array<float, 4>
Xsens2FbxManager::qmul(const std::array<float, 4>& pQ1, const std::array<float, 4>& pQ2) const
{
	const float w1 = pQ1[0], x1 = pQ1[1], y1 = pQ1[2], z1 = pQ1[3];
	const float w2 = pQ2[0], x2 = pQ2[1], y2 = pQ2[2], z2 = pQ2[3];

	std::array<float, 4> q;
	q[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2; // w
	q[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2; // x
	q[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2; // y
	q[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; // z
	return q;
}

std::array<float, 3>
Xsens2FbxManager::qrotateVec(const std::array<float, 4>& pQ, const std::array<float, 3>& pV) const
{
	// Rotate vector pV by quaternion pQ (w,x,y,z), via q * (0,v) * q^-1,
	// expanded into closed form to avoid allocating intermediate quaternions.
	const float qw = pQ[0], qx = pQ[1], qy = pQ[2], qz = pQ[3];
	const float vx = pV[0], vy = pV[1], vz = pV[2];

	const float tx = 2.0f * (qy * vz - qz * vy);
	const float ty = 2.0f * (qz * vx - qx * vz);
	const float tz = 2.0f * (qx * vy - qy * vx);

	return {
		vx + qw * tx + (qy * tz - qz * ty),
		vy + qw * ty + (qz * tx - qx * tz),
		vz + qw * tz + (qx * ty - qy * tx)
	};
}

std::vector<std::array<float, 3>>
Xsens2FbxManager::remapJointIndices(const std::vector<std::array<float, 3>>& pJointData) const
{
	std::vector<std::array<float, 3>> remappedJointData(mJointCount);

	for (size_t jI = 0; jI < static_cast<size_t>(mJointCount); ++jI)
	{
		for (size_t d = 0; d < 3; ++d)
		{
			remappedJointData[jI][d] = pJointData[mJointIndexMap[jI]][d];
		}
	}

	return remappedJointData;
}

std::vector<std::array<float, 4>>
Xsens2FbxManager::remapJointIndices(const std::vector<std::array<float, 4>>& pJointData) const
{
	std::vector<std::array<float, 4>> remappedJointData(mJointCount);

	for (size_t jI = 0; jI < static_cast<size_t>(mJointCount); ++jI)
	{
		for (size_t d = 0; d < 4; ++d)
		{
			remappedJointData[jI][d] = pJointData[mJointIndexMap[jI]][d];
		}
	}

	return remappedJointData;
}

std::vector<std::array<float, 3>>
Xsens2FbxManager::convertWorld2Local(const std::vector<std::array<float, 3>>& pPosWorld, const std::vector<std::array<float, 4>>& pRotWorld) const
{
	if (pPosWorld.size() != static_cast<size_t>(mJointCount))
		throw std::runtime_error("Incorrect Number of Joint Positions");
	if (pRotWorld.size() != static_cast<size_t>(mJointCount))
		throw std::runtime_error("Incorrect Number of Joint Rotations (needed to express pos_local in the parent's rotated frame)");

	std::vector<std::array<float, 3>> posLocal(mJointCount);

	int root = -1;
	for (std::size_t j = 0; j < static_cast<size_t>(mJointCount); ++j)
	{
		if (mJointParents[j] == -1)
		{
			root = static_cast<int>(j);
			break;
		}
	}
	if (root < 0)
		throw std::runtime_error("No root joint found (parent == -1).");

	// Root: local position == world position (no parent frame to express it in).
	posLocal[root] = pPosWorld[root];

	for (std::size_t j = 0; j < static_cast<size_t>(mJointCount); ++j)
	{
		if (static_cast<int>(j) == root)
			continue;

		int p = mJointParents[j];
		if (p < 0)
			continue;

		const std::array<float, 3>& pos_p_world = pPosWorld[static_cast<size_t>(p)];
		const std::array<float, 3>& pos_j_world = pPosWorld[j];

		std::array<float, 3> delta_world = {
			pos_j_world[0] - pos_p_world[0],
			pos_j_world[1] - pos_p_world[1],
			pos_j_world[2] - pos_p_world[2]
		};

		// Rotate the world-space delta into the PARENT's local frame, by
		// applying the inverse of the parent's world rotation. This makes
		// pos_local a rigid, rest-pose-stable bone vector -- constant
		// length/direction regardless of performer motion.
		const std::array<float, 4>& R_p_world = pRotWorld[static_cast<size_t>(p)];
		std::array<float, 4> R_p_inv = qconjugate(R_p_world);

		posLocal[j] = qrotateVec(R_p_inv, delta_world);
	}

	return posLocal;
}

std::vector<std::array<float, 4>>
Xsens2FbxManager::convertWorld2Local(const std::vector<std::array<float, 4>>& pRotWorld) const
{
	if (pRotWorld.size() != static_cast<size_t>(mJointCount))
		throw std::runtime_error("Incorrect Number of Joint Rotations");

	int root = -1;
	for (std::size_t j = 0; j < static_cast<size_t>(mJointCount); ++j)
	{
		if (mJointParents[j] == -1)
		{
			root = static_cast<int>(j);
			break;
		}
	}
	if (root < 0)
		throw std::runtime_error("No root joint found (parent == -1).");

	std::vector<std::array<float, 4>> rotLocal(mJointCount);

	// Root: local == world. No rest-pose subtraction -- Xsens/MVN Analyze
	// already calibrated this anatomically, once, at the start of the
	// capture session (the performer's own T-pose/N-pose calibration done
	// inside MVN Analyze itself, NOT tied to when this program starts).
	rotLocal[root] = pRotWorld[root];

	for (std::size_t j = 0; j < static_cast<size_t>(mJointCount); ++j)
	{
		if (static_cast<int>(j) == root)
			continue;

		int p = mJointParents[j];
		if (p < 0)
			continue;

		const std::array<float, 4>& R_p_world = pRotWorld[static_cast<size_t>(p)];
		const std::array<float, 4>& R_j_world = pRotWorld[j];

		std::array<float, 4> R_p_inv = qconjugate(R_p_world);
		rotLocal[j] = qmul(R_p_inv, R_j_world);
	}

	return rotLocal;
}

std::vector<std::array<float, 3>>
Xsens2FbxManager::swapCoordinates(const std::vector<std::array<float, 3>>& pJointPos) const
{
	std::vector<std::array<float, 3>> swappedJointPos(mJointCount);

	for (size_t jI = 0; jI < static_cast<size_t>(mJointCount); ++jI)
	{
		for (size_t d = 0; d < 3; ++d)
		{
			swappedJointPos[jI][d] = pJointPos[jI][mPosIndexMap[d]];
		}
	}

	return swappedJointPos;
}

std::vector<std::array<float, 4>>
Xsens2FbxManager::swapCoordinates(const std::vector<std::array<float, 4>>& pJointRot) const
{
	std::vector<std::array<float, 4>> swappedJointRot(mJointCount);

	for (size_t jI = 0; jI < static_cast<size_t>(mJointCount); ++jI)
	{
		for (size_t d = 0; d < 4; ++d)
		{
			swappedJointRot[jI][d] = pJointRot[jI][mRotIndexMap[d]];
		}
	}

	return swappedJointRot;
}
