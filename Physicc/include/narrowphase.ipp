#include "narrowphase.hpp"
#include <cmath>

template <>
Contact checkCollision<Sphere, Sphere>(Broadphase::PotentialContact a)
{
	// returns true if the spheres in `a` are colliding. Otherwise
	// returns false. Also writes the data to the public collision array.

	auto sphere1 = a.rb1;
	auto sphere2 = a.rb2;
	glm::vec3 centre1 = sphere1.getCentroid();
	glm::vec3 centre2 = sphere2.getCentroid();
	float radius1 = sphere1.getCollider()->getRadius();
	float radius2 = sphere2.getCollider()->getRadius();

	glm::vec3 centreLine = centre1 - centre2; // Convention is for normal to point from body2 to body1
	float centreLineLength = glm::length(centreLine);

	if (centreLineLength <= radius1 + radius2)
	{	
		return Contact(rb1,
						rb2,
						(centre1+centre2)/2, // contact point
						centreLine/centreLineLength, // contact normal
						radius1 + radius2 - centreLineLength // penetration
		);
	}


}

template <>
Contact checkCollision<Box, Sphere>(Broadphase::PotentialContact a)
{
	// PotentialContact contains two pointers to rigid bodies.
	// For BoxSphere, the first is a box, the second a sphere.
	// Returns true if the box and the sphere in `a` are colliding.
	// Otherwise returns false. Also writes the data to the public collision array.
	
	auto box = a.rb1;
	auto sphere = a.rb2;

	glm::vec3 sphereCentre = sphere.getCentroid();
	float radius  = sphere.getCollider()->getRadius();
	
	glm::vec3 relativeSphereCentre = box.getCollider()->toBoxCoordinates(sphereCentre);

	glm::vec3 closestPt;
	
	// Clamp the transformed coordinates by half-edges of the box
	float dist = relativeSphereCentre.x;
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
	
	closestPt = box.getCollider()->toWorldCoordinates(closestPt);

	glm::vec3 contactNormal = closestPt - sphereCentre; // Convention is for normal to point from body2 to body1
	
	float normalLength = glm::length(contactNormal);

	if (normalLength <= radius) 
	{
		return Contact(rb1,
					   rb2,
					   closestPt,
					   contactNormal/normalLength,
					   radius - normalLength
		);
	}
}

template <>
Contact checkCollision<Sphere, Box>(Broadphase::PotentialContact a)
{	
	Contact boxSphereContact =  checkCollision<Box, Sphere>(Broadphase::PotentialContact(a.second, a.first));
	// Exchange bodies and flip the contact normal
	return Contact(boxSphereContact.Body2,
				   boxSphereContact.Body1, 
				   boxSphereContact.contactPoint,
				   -boxSphereContact.contactNormal,
				   boxSphereContact.penetration
	);
}

template <>
Contact checkCollision<Box, Box>(Broadphase::PotentialContact a)
{
	// returns true if boxes the in `a` are colliding. Otherwise
	// returns false. Also writes the data to the public collision array.
}
