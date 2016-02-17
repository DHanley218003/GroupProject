/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef JADGTIGER_H
#define JADGTIGER_H

// This is a fun demo that shows off the wheel joint
class JadgTiger : public Test
{
public:
	JadgTiger()
	{		
		m_hz = 4.0f;
		m_zeta = 0.7f;
		m_speed = 4.6f;

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground->CreateFixture(&fd);

			float32 hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			float32 x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 80.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 10.0f, 5.0f));
			ground->CreateFixture(&fd);

			x += 20.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x, 20.0f));
			ground->CreateFixture(&fd);
		}

		// Teeter
		{
			b2BodyDef bd;
			bd.position.Set(140.0f, 1.0f);
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(10.0f, 0.25f);
			body->CreateFixture(&box, 1.0f);

			b2RevoluteJointDef jd;
			jd.Initialize(ground, body, body->GetPosition());
			jd.lowerAngle = -8.0f * b2_pi / 180.0f;
			jd.upperAngle = 8.0f * b2_pi / 180.0f;
			jd.enableLimit = true;
			m_world->CreateJoint(&jd);

			body->ApplyAngularImpulse(100.0f);
		}

		// Bridge
		{
			int32 N = 20;
			b2PolygonShape shape;
			shape.SetAsBox(1.0f, 0.125f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 0.6f;

			b2RevoluteJointDef jd;

			b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(161.0f + 2.0f * i, -0.125f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(160.0f + 2.0f * i, -0.125f);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);

				prevBody = body;
			}

			b2Vec2 anchor(160.0f + 2.0f * N, -0.125f);
			jd.Initialize(prevBody, ground, anchor);
			m_world->CreateJoint(&jd);
		}

		// Boxes
		{
			b2PolygonShape box;
			box.SetAsBox(0.5f, 0.5f);

			b2Body* body = NULL;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			bd.position.Set(230.0f, 0.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 1.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 2.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 3.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 4.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);
		}

		// JadgTiger
		{
			b2PolygonShape lowerChassispt1;
			b2Vec2 vertices[5];
			vertices[0].Set(0.71f, 0.95f);
			vertices[1].Set(6.26f, 0.84f);
			vertices[2].Set(6.18f, 1.6f);
			vertices[3].Set(0.64f, 1.32f);
			vertices[4].Set(0.71f, 0.95f);
			lowerChassispt1.Set(vertices, 5);

			b2PolygonShape lowerChassispt2;
			b2Vec2 vertices[5];
			vertices[0].Set(6.26f, 0.84f);
			vertices[1].Set(6.74f, 1.16f);
			vertices[2].Set(6.71f, 1.59f);
			vertices[3].Set(6.18f, 1.6f);
			vertices[4].Set(0.71f, 0.95f);
			lowerChassispt2.Set(vertices, 5);

			b2PolygonShape lowerChassispt3;
			b2Vec2 vertices[6];
			vertices[0].Set(6.74f, 1.16f);
			vertices[1].Set(7.72f, 0.98f);
			vertices[2].Set(7.57f, 1.38f);
			vertices[3].Set(7.21f, 1.52f);
			vertices[4].Set(6.71f, 1.59f);
			vertices[5].Set(6.74f, 1.16f);
			lowerChassispt3.Set(vertices, 6);

			b2PolygonShape upperChassispt1;
			b2Vec2 vertices[4];
			vertices[0].Set(0.27f, 1.67f);
			vertices[1].Set(0.5f, 1.5f);
			vertices[2].Set(0.56f, 1.87f);
			vertices[3].Set(0.27f, 1.67f);
			upperChassispt1.Set(vertices, 4);

			b2PolygonShape upperChassispt2;
			b2Vec2 vertices[6];
			vertices[0].Set(0.64f, 1.32f);
			vertices[1].Set(7.21f, 1.52f);
			vertices[2].Set(6.77f, 1.84f);
			vertices[3].Set(0.56f, 1.87f);
			vertices[4].Set(0.5f, 1.5f);
			vertices[5].Set(0.64f, 1.32f);
			upperChassispt2.Set(vertices, 6);

			b2PolygonShape gunpt1;
			b2Vec2 vertices[5];
			vertices[0].Set(5.59f, 1.86f);
			vertices[1].Set(6.76f, 2.01f);
			vertices[2].Set(6.79f, 2.35f);
			vertices[3].Set(5.34f, 2.82f);
			vertices[4].Set(5.59f, 1.86f);
			gunpt1.Set(vertices, 5);

			b2PolygonShape gunpt2;
			b2Vec2 vertices[5];
			vertices[0].Set(6.79f, 2.01f);
			vertices[1].Set(10.75f, 2.04f);
			vertices[2].Set(10.75f, 2.22f);
			vertices[3].Set(6.79f, 2.35f);
			vertices[4].Set(6.79f, 2.01f);
			gunpt2.Set(vertices, 5);

			b2PolygonShape turret;
			b2Vec2 vertices[5];
			vertices[0].Set(2.63f, 2.84f);
			vertices[1].Set(2.5f, 1.87f);
			vertices[2].Set(5.59f, 1.79f);
			vertices[3].Set(5.34f, 2.82f);
			vertices[4].Set(2.63f, 2.84f);
			turret.Set(vertices, 5);

			b2CircleShape circle;
			circle.m_radius = 0.34f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 1.0f);
			m_JadgTiger = m_world->CreateBody(&bd);
			m_JadgTiger->CreateFixture(&lowerChassispt1, 1.0f);
			m_JadgTiger->CreateFixture(&lowerChassispt2, 1.0f);
			m_JadgTiger->CreateFixture(&lowerChassispt3, 1.0f);
			m_JadgTiger->CreateFixture(&upperChassispt1, 1.0f);
			m_JadgTiger->CreateFixture(&upperChassispt2, 1.0f);
			m_JadgTiger->CreateFixture(&gunpt1, 1.0f);
			m_JadgTiger->CreateFixture(&gunpt2, 1.0f);
			m_JadgTiger->CreateFixture(&turret, 1.0f);

			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;
			fd.filter.categoryBits = 0x0002;
			fd.filter.maskBits = 0xFFFF & ~0x0004;

			bd.position.Set(-1.0f, 0.35f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&fd);

			bd.position.Set(-0.2f, 0.35f);
			m_wheel3 = m_world->CreateBody(&bd);
			m_wheel3->CreateFixture(&fd);

			bd.position.Set(0.6f, 0.35f);
			m_wheel5 = m_world->CreateBody(&bd);
			m_wheel5->CreateFixture(&fd);

			fd.filter.categoryBits = 0x0004;
			fd.filter.maskBits = 0xFFFF;

			bd.position.Set(-0.6f, 0.4f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&fd);

			bd.position.Set(0.2f, 0.4f);
			m_wheel4 = m_world->CreateBody(&bd);
			m_wheel4->CreateFixture(&fd);

			bd.position.Set(1.0f, 0.4f);
			m_wheel6 = m_world->CreateBody(&bd);
			m_wheel6->CreateFixture(&fd);

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_JadgTiger, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel3, m_wheel3->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring3 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel4, m_wheel4->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring4 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel5, m_wheel5->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring5 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel6, m_wheel6->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring6 = (b2WheelJoint*)m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			m_spring1->SetMotorSpeed(m_speed);
			break;

		case 's':
			m_spring1->SetMotorSpeed(0.0f);
			break;

		case 'd':
			m_spring1->SetMotorSpeed(-m_speed);
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;

		case 'e':
			m_hz += 1.0f;
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;
		}
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;

		settings->viewCenter.x = m_JadgTiger->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new JadgTiger;
	}

	b2Body* m_JadgTiger;
	b2Body* m_wheel1;
	b2Body* m_wheel2;
	b2Body* m_wheel3;
	b2Body* m_wheel4;
	b2Body* m_wheel5;
	b2Body* m_wheel6;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
	b2WheelJoint* m_spring1;
	b2WheelJoint* m_spring2;
	b2WheelJoint* m_spring3;
	b2WheelJoint* m_spring4;
	b2WheelJoint* m_spring5;
	b2WheelJoint* m_spring6;
};

#endif
