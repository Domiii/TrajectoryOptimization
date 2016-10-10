/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef AddPair_H
#define AddPair_H

class Pendulum : public Test
{
public:
    b2Body* pendulum;

	Pendulum()
	{
        // Create pendulum
		{
            const float mass = 1.f;
            const float headRadius = 2.f;
            const b2Vec2 shaftDim(.2f, 10.f);
            const b2Vec2 shaftPos(shaftDim.x/2, 0);
            const b2Vec2 pendWorldPos(0.f, 20);
            const b2Vec2 pendHookWorldPos(pendWorldPos + b2Vec2(shaftDim.x/2, shaftDim.y));

            // create body
			b2BodyDef bodyDef;
            bodyDef.type = b2_dynamicBody;
            bodyDef.position = pendWorldPos;
            pendulum = m_world->CreateBody(&bodyDef);
            
            // add shaft
			b2PolygonShape shaft;
			shaft.SetAsBox(shaftDim.x, shaftDim.y, shaftPos, 0);
			pendulum->CreateFixture(&shaft, mass * .00001f);

            // add head
            b2CircleShape head;
            head.m_radius = headRadius;
            float headVolume = .5f * headRadius * b2_pi * b2_pi;
            head.m_p = b2Vec2(shaftDim.x/2, headRadius - shaftDim.y);
			pendulum->CreateFixture(&head, mass / headVolume);

            // create pendulum joint
			b2BodyDef pendulumHookDef;
            pendulumHookDef.type = b2_staticBody;
            pendulumHookDef.position = pendHookWorldPos;
            b2Body* pendulumHook = m_world->CreateBody(&pendulumHookDef);
            
            b2CircleShape hookShape;
            hookShape.m_radius = 2 * shaftDim.x;
			pendulumHook->CreateFixture(&hookShape, 0);

            b2RevoluteJointDef jointDef;
            jointDef.Initialize(pendulum, pendulumHook, pendHookWorldPos);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
		}
	}

	static Test* Create()
	{
		return new Pendulum;
	}
};

#endif
