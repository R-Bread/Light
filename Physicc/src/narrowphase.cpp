#include "narrowphase.hpp"

using namespace Physicc::Narrowphase;
float NarrowphaseImpl::penetrationAlongAxis(BoxCollider* bc1,
                                            BoxCollider* bc2,
                                            const glm::vec3 axis)
{
    // The axis should be normalised, always
    float centreDistance = abs(glm::dot(axis, bc1->getCentroid() - bc2->getCentroid()));

    std::vector<glm::vec4> standardVertices = BoxCollider::getVertices();
    float halfProjection1 = 0; // Longest of the projections of the centre-vertex lines on the axis
    float halfProjection2 = 0;
    float temp;
    for (int i=0; i<3; i++)
    {
        temp = abs(glm::dot(axis, bc1->makeAxis(glm::vec3(standardVertices[i]))));
        if (temp > halfProjection1) {halfProjection1 = temp; }

        temp = abs(glm::dot(axis, bc2->makeAxis(glm::vec3(standardVertices[i]))));
        if (temp > halfProjection2) {halfProjection2 = temp; }
    }

    return centreDistance - halfProjection1 - halfProjection2; // Returns penetration, negative means no penetration
}