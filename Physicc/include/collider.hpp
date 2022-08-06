#ifndef __COLLIDER_H__
#define __COLLIDER_H__

#include "tools/Tracy.hpp"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "boundingvolume.hpp"
#include <vector>

namespace Physicc
{
	/**
	 * @brief Collider class
	 *
	 * This is a abstract class which acts as the base for all the shape specific classes
	 */
	class Collider
	{
	public:
		/**
		 * @brief Get Position of object's center
		 *
		 * @return glm::vec3
		 */
		[[nodiscard]] inline glm::vec3 getPosition() const
		{
			ZoneScoped;

			return m_position;
		}

		/**
		 * @brief get Angle of rotation of the object about its center
		 *
		 * @return glm::vec3
		 */
		[[nodiscard]] inline glm::vec3 getRotate() const
		{
			ZoneScoped;

			return m_rotate;
		}

		/**
		 * @brief get Scale of object
		 *
		 * @return glm::vec3
		 */
		[[nodiscard]] inline glm::vec3 getScale() const
		{
			ZoneScoped;

			return m_scale;
		}

		/**
		 * @brief get Transform matrix of object
		 *
		 * @return glm::mat4
		 */
		[[nodiscard]] inline glm::mat4 getTransform() const
		{
			ZoneScoped;

			return m_transform;
		}


		/**
		 * @brief set Position of object's center
		 *
		 * @param position Takes the (x,y,z) coordinates to place the object's center at
		 */
		inline void setPosition(const glm::vec3 position)
		{
			ZoneScoped;

			m_position = position;
		}

		/**
		 * @brief Set rotation of object about it's center
		 *
		 * @param rotate vec3 containing rotation values about x, y, z axes
		 */
		inline void setRotate(const glm::vec3 rotate)
		{
			ZoneScoped;

			m_rotate = rotate;
		}

		/**
		 * @brief get Position of object's center
		 *
		 * @param scale New scale of the object
		 */
		inline void setScale(const glm::vec3 scale)
		{
			ZoneScoped;

			m_scale = scale;
		}

		void updateTransform();

		virtual BoundingVolume::AABB getAABB() const = 0;
		// Each child will calculate its AABB according to its own shape

		virtual glm::vec3 getCentroid() const = 0;

	protected:
		enum class Type
		{
			Box,
			Sphere,
			Typecount
		};

		/**
		 * Make the constructor protected to prevent instantiation of this class
		 */

		/**
		 * @brief Construct a new Collider::Collider object
		 *
		 * @param position Position of the object. Default = (0,0,0)
		 * @param rotation Rotations about the axes. Default = (0,0,0)
		 * @param scale Length along each of the axes. Default = (1,1,1)
		 */
		Collider(glm::vec3 position = glm::vec3(0),
					glm::vec3 rotation = glm::vec3(0),
					glm::vec3 scale = glm::vec3(1));

	public:
		virtual Type getType() const = 0;

		static Type getStaticType();

	protected:
		glm::vec3 m_position;
		glm::vec3 m_rotate;
		glm::vec3 m_scale;
		glm::mat4 m_transform;
	};

	/**
	 * @brief BoxCollider class
	 *
	 * Box shaped collider, holds the shape and transform of the body.
	 */
	class BoxCollider : public Collider
	{
	public:
		BoxCollider(glm::vec3 position = glm::vec3(0),
					glm::vec3 rotation = glm::vec3(0),
					glm::vec3 scale = glm::vec3(1));

		[[nodiscard]] BoundingVolume::AABB getAABB() const override;

		[[nodiscard]] inline glm::vec3 getCentroid() const override
		{
			ZoneScoped;
			return m_position;
		}

		inline Type getType() const override
		{
			ZoneScoped;
			return Type::Box;
		}

		inline static Type getStaticType()
		{	
			ZoneScoped;
			return Type::Box;
		}

		static std::vector<glm::vec4> initVertices()
		{
			std::vector<glm::vec4> tempVertices = std::vector<glm::vec4>(8, glm::vec4(0, 0, 0, 1.0f)); //Static class member

			//Top-face vertices
			tempVertices[0] = glm::vec4(glm::vec3(0.5f), 1.0f);
			tempVertices[1] = tempVertices[0] - glm::vec4(1, 0, 0, 0);
			tempVertices[2] = tempVertices[0] - glm::vec4(0, 1, 0, 0);
			tempVertices[3] = tempVertices[0] - glm::vec4(1, 1, 0, 0);

			//Bottom-face vertices
			tempVertices[4] = glm::vec4(glm::vec3(-0.5f), 1.0f);
			tempVertices[5] = tempVertices[0] + glm::vec4(1, 0, 0, 0);
			tempVertices[6] = tempVertices[0] + glm::vec4(0, 1, 0, 0);
			tempVertices[7] = tempVertices[0] + glm::vec4(1, 1, 0, 0);
			
			return tempVertices;
		}

		inline static std::vector<glm::vec4> getVertices()
		{
			ZoneScoped;
			return s_vertices;
		}

		glm::vec3 toBoxCoordinates(const glm::vec3& point) const; 

		glm::vec3 toWorldCoordinates(const glm::vec3& point) const; 

		glm::vec3 makeAxis(const glm::vec3& point) const;

	private:
		static std::vector<glm::vec4> s_vertices;
	};

	/**
	 * @brief SphereCollider class
	 *
	 * Sphere shaped collider, holds the radius and transform of the body
	 */
	class SphereCollider : public Collider
	{
	public:
		SphereCollider(float radius = 1.0f,
						glm::vec3 position = glm::vec3(0),
						glm::vec3 rotation = glm::vec3(0),
						glm::vec3 scale = glm::vec3(1));

		[[nodiscard]] BoundingVolume::AABB getAABB() const override;

		[[nodiscard]] inline glm::vec3 getCentroid() const override
		{
			return m_position;
		}

		[[nodiscard]] inline float getRadius() const
		{
			return m_radius;
		}

		inline Type getType() const override
		{
			return Type::Sphere;
		}

		inline static Type getStaticType()
		{
			return Type::Sphere;
		}

	private:
		float m_radius;
	};
}

#endif // __COLLIDER_H__
