#pragma once

#include <vector>
#include <array>
#include "dab_singleton.h"

namespace dab
{

	namespace xsens
	{

# pragma mark Xsens2FbxManager declaration

		class Xsens2FbxManager : public Singleton<Xsens2FbxManager>
		{
		public:
			Xsens2FbxManager();
			~Xsens2FbxManager();

			int getJointCount() const;

			std::vector<std::array<float, 3>> remapJointIndices(const std::vector<std::array<float, 3>>& pJointData) const;
			std::vector<std::array<float, 4>> remapJointIndices(const std::vector<std::array<float, 4>>& pJointData) const;

			// Position conversion expresses pos_local in the PARENT's rotated
			// local frame (rotated by the inverse of the parent's world rotation),
			// so it needs the matching world rotations. This makes pos_local a
			// rigid, rest-pose-stable bone vector -- constant length/direction
			// regardless of performer motion.
			std::vector<std::array<float, 3>> convertWorld2Local(const std::vector<std::array<float, 3>>& pPosWorld, const std::vector<std::array<float, 4>>& pRotWorld) const;

			// Rotation conversion: NO additional rest-pose calibration is applied
			// here. Xsens/MVN Analyze already anatomically calibrates rot_world
			// relative to the performer's own T-pose/N-pose ONCE, at the start of
			// the capture session, inside MVN Analyze itself. Re-zeroing again in
			// this program (using whatever pose the performer happens to be in
			// when THIS program starts, which is generally NOT a T-pose -- e.g.
			// mid-performance) was double-counting and actively wrong. This
			// function now simply strips the parent's world rotation, exactly
			// mirroring Xsens's own segment-hierarchy convention with no extra
			// reference-frame subtraction.
			std::vector<std::array<float, 4>> convertWorld2Local(const std::vector<std::array<float, 4>>& pRotWorld) const;

			std::vector<std::array<float, 3>> swapCoordinates(const std::vector<std::array<float, 3>>& pJointData) const;
			std::vector<std::array<float, 4>> swapCoordinates(const std::vector<std::array<float, 4>>& pJointData) const;

		protected:
			static int sJointCount;
			static std::vector<int> sJointParents;

			// quaternion / vector helper functions
			std::array<float, 4> qconjugate(const std::array<float, 4>& pQ) const;
			std::array<float, 4> qmul(const std::array<float, 4>& pQ1, const std::array<float, 4>& pQ2) const;
			std::array<float, 3> qrotateVec(const std::array<float, 4>& pQ, const std::array<float, 3>& pV) const;

			int mJointCount;
			std::vector<int> mJointParents;

			std::vector<int> mJointIndexMap = { 0, 15, 16, 17, 18, 19, 20, 21, 22, 1, 2, 3, 4, 11, 12, 13, 14, 7, 8, 9, 10, 5, 6 };
			std::array<int, 3> mPosIndexMap = { 1, 2, 0 };
			std::array<size_t, 4> mRotIndexMap = { 2, 3, 1, 0 };
		};

	};
};
