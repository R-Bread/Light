#ifndef __NARROWPHASE_H__
#define __NARROWPHASE_H__

#include "glm/glm.hpp"
#include "core/assert.hpp"
#include "rigidbody.hpp"
#include "broadphase.hpp"
#include "collider.hpp"
#include <vector>
#include <array>
#include <cstddef> // for std::size_t
#include <memory>

/**
 * TODO: Write BoxSphere and BoxBox functions, make it so that the functions return all the
 * information required for contact resolution.
 *
 * Fix formatting issues, and reorganize the functions in the collisionFunctionMatrix struct.
 */

namespace Physicc::Narrowphase
{
	namespace NarrowphaseImpl {
		float penetrationAlongAxis(BoxCollider* bc1,
								   BoxCollider* bc2,
								   const glm::vec3 axis);
	}

	struct Contact
	{
		Contact(RigidBody* rb1, RigidBody* rb2, glm::vec3 cP, glm::vec3 cN, double p) 
		: Body1(rb1), Body2(rb2), contactPoint(cP), contactNormal(cN), penetration(p)
		{
		}

		RigidBody* Body1;
		RigidBody* Body2;
		glm::vec3 contactPoint;
		glm::vec3 contactNormal;
		double penetration;
		// Friction has been removed as it is a property of rigid body and not of Contact
	};

	template <typename FirstBody, typename SecondBody>
	Contact checkCollision(Broadphase::PotentialContact a);

	// This file includes specific overloads of the above function.
	#include "narrowphase.ipp"

	// This file includes the definition of the collision function matrix class. It cannot
	// be a standalone header file because it needs to be able to see all the template
	// specializations of the checkCollision function to be able to work its magic.
	#include "collisionFunctionMatrix.ipp"

	// The CollisionDetector class definition must come after the collisioFunctionMatrix.ipp and
	// narrowphase.ipp because the compiler needs a full definition of everything in order to work.
	class CollisionDetector
	{
		public:
		CollisionDetector(std::vector<Broadphase::PotentialContact>& v)
		: collisionArray(v)
		{
		}

		// Move individual components to individual function

		void generateContacts()
		{
			// Take in the entire array of potential contacts and then
			// traverse through every single one, and dispatch it to the
			// correct narrow phase collision detecting function.
		}

		std::vector<Contact> getContacts()
		{
			return collisionInfo;
		}

		private:
		std::vector<Broadphase::PotentialContact> collisionArray;
		std::vector<Contact> collisionInfo;
		collisionFunctionMatrix<SphereCollider, BoxCollider> m;
	};
}

#endif // __NARROWPHASE_H__
