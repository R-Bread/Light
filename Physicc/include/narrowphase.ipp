#include "narrowphase.hpp"

template <>
Contact checkCollision<SphereCollider, SphereCollider>(Broadphase::PotentialContact a)
{
	// Returns Contact struct containing contact information
	// if the spheres in `a` are colliding. Otherwise
	// returns nothing.

	std::shared_ptr<SphereCollider> sphere1 = std::dynamic_pointer_cast<SphereCollider>(a.rb1->getCollider());
	std::shared_ptr<SphereCollider> sphere2 = std::dynamic_pointer_cast<SphereCollider>(a.rb2->getCollider());
	glm::vec3 centre1 = sphere1->getCentroid();
	glm::vec3 centre2 = sphere2->getCentroid();
	float radius1 = sphere1->getRadius();
	float radius2 = sphere2->getRadius();

	glm::vec3 centreLine = centre1 - centre2; // Convention is for normal to point from body2 to body1
	float centreLineLength = glm::length(centreLine);

	if (centreLineLength <= radius1 + radius2)
	{	
		return {a.rb1,
				a.rb2,
				(centre1+centre2)/2.0f, // contact point
				centreLine/centreLineLength, // contact normal
				radius1 + radius2 - centreLineLength // penetration
		};
	}
}

template <>
Contact checkCollision<BoxCollider, SphereCollider>(Broadphase::PotentialContact a)
{
	// PotentialContact contains two pointers to rigid bodies.
	// For BoxSphere, the first is a box, the second a sphere.
	// Returns Contact struct if the box and the sphere in `a` are colliding.
	// Otherwise returns nothing.
	
	std::shared_ptr<BoxCollider> box = std::dynamic_pointer_cast<BoxCollider>(a.rb1->getCollider());
	std::shared_ptr<SphereCollider> sphere = std::dynamic_pointer_cast<SphereCollider>(a.rb2->getCollider());

	glm::vec3 sphereCentre = sphere->getCentroid();
	float radius  = sphere->getRadius();
	
	glm::vec3 relativeSphereCentre = box->toBoxCoordinates(sphereCentre);

	glm::vec3 closestPt;
	
	// Clamp the transformed coordinates by half-edges of the box
	float dist = relativeSphereCentre.x; // TODO: Write function for the repetitive code
	if (dist > 0.5) dist = 0.5;
	if (dist < -0.5) dist = -0.5;
	closestPt.x = dist;

	dist = relativeSphereCentre.y;
	if (dist > 0.5) dist = 0.5;
	if (dist < -0.5) dist = -0.5;
	closestPt.y = dist;

	dist = relativeSphereCentre.z;
	if (dist > 0.5) dist = 0.5;
	if (dist < -0.5) dist = -0.5;
	closestPt.z = dist;
	
	closestPt = box->toWorldCoordinates(closestPt);

	glm::vec3 contactNormal = closestPt - sphereCentre; // Convention is for normal to point from body2 to body1
	
	float normalLength = glm::length(contactNormal);

	if (normalLength <= radius) 
	{
		return {a.rb1,
				a.rb2,
				closestPt,
				contactNormal/normalLength,
				radius - normalLength
		};
	}
}

template <>
Contact checkCollision<SphereCollider, BoxCollider>(Broadphase::PotentialContact a)
{
	Contact boxSphereContact = checkCollision<BoxCollider, SphereCollider>(Broadphase::PotentialContact(a.rb2, a.rb1));
	// Exchange bodies and flip the contact normal
	return {boxSphereContact.Body2,
			boxSphereContact.Body1, 
			boxSphereContact.contactPoint,
			-boxSphereContact.contactNormal,
			boxSphereContact.penetration};
}

template <>
Contact checkCollision<BoxCollider, BoxCollider>(Broadphase::PotentialContact a)
{
	// Returns Contact struct if boxes the in `a` are colliding.
	// Otherwise returns nothing.
	std::shared_ptr<BoxCollider> box1 = std::dynamic_pointer_cast<BoxCollider>(a.rb1->getCollider());
	std::shared_ptr<BoxCollider> box2 = std::dynamic_pointer_cast<BoxCollider>(a.rb2->getCollider());

	std::vector<glm::vec3> separatingAxes(15);
	std::vector<glm::vec3> boxAxes = {glm::vec3(1,0,0),
									   glm::vec3(0,1,0),
									   glm::vec3(0,0,1)
									   }; // Act as the axes for both face normals and edges
	std::vector<glm::vec3> box1Axes(3), box2Axes(3);
	glm::vec3 temp;
	float tempFloat;
	glm::vec3 collisionAxis;
	float minPenetration = std::numeric_limits<float>::max();

	// Implementing Seperating Axes Test
		// Obtain axes for SAT
			// Face axes first (more likely to pass (no collision))
	for (std::size_t i=0; i<3; i++){
		temp = glm::normalize(
			box1->makeAxis(boxAxes[i]));
		separatingAxes.push_back(temp);
		box1Axes.push_back(temp);
	}

	for (std::size_t i=0; i<3; i++){
		temp = glm::normalize(
			box2->makeAxis(boxAxes[i]));
		separatingAxes.push_back(temp);
		box2Axes.push_back(temp);
	}
			// Then take edge axes, use their cross products
			// If an axis is 0, discard
	for (std::size_t i=0; i<3; i++){
		for (std::size_t j=0; j<3; j++){
			temp = glm::cross(box1Axes[i], box2Axes[j]);
			if (glm::length(temp)<0.01){
				continue;
			}
			if (std::find(separatingAxes.begin(), separatingAxes.end(), temp) != separatingAxes.end()){
				continue;
			}
			separatingAxes.push_back(temp);
		}
	}
		// Perform SAT for each axis
			// If no overlap in any one test, no collision happening
			// SAT function must return impenetration
	for (std::size_t i=0; i < separatingAxes.size(); i++)
	{
		tempFloat = NarrowphaseImpl::penetrationAlongAxis(box1.get(), box2.get(), separatingAxes[i]);
		if (tempFloat >= 0) {break;}
		if (tempFloat < minPenetration)
		{
			minPenetration = tempFloat;
			collisionAxis = separatingAxes[i];
		}
	}

	return {a.rb1,
			a.rb2,
			glm::vec3(0), // Supposed to return the contact point, but I gotta figure out how to do that
			collisionAxis,
			minPenetration
	};
}
