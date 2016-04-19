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

#ifndef JadgTiger_H
#define JadgTiger_H

class JadgTiger : Test
{
public:

	JadgTiger()
	{
		m_hz = 4.0f;
		m_zeta = 0.7f;
		m_speed = 0;
		m_mass = 11.95f;
		m_torque = 150.0f;
		m_bulletVelocity = 950.0f;
		m_roundType = 0;
		float density = 10.0f;

		enum _entityCategory
		{
			ODD_WHEEL = 0x0002,
			EVEN_WHEEL = 0x0004,
			TANK = 0x0008,
			TRACKS = 0x0010
		};

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

			float32 hs[10] = { 0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f };

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
			m_shell = NULL;
			b2PolygonShape lowerChassispt1;
			b2Vec2 vertices[8];
			vertices[0].Set(0.71f, 0.95f);
			vertices[1].Set(6.26f, 0.84f);
			vertices[2].Set(6.18f, 1.5f);
			vertices[3].Set(0.64f, 1.32f);
			lowerChassispt1.Set(vertices, 4);

			b2PolygonShape lowerChassispt2;
			vertices[0].Set(6.26f, 0.84f);
			vertices[1].Set(6.74f, 1.16f);
			vertices[2].Set(6.71f, 1.5f);
			vertices[3].Set(6.18f, 1.5f);
			lowerChassispt2.Set(vertices, 4);


			b2PolygonShape lowerChassispt3;
			vertices[0].Set(6.74f, 1.16f);
			vertices[1].Set(7.72f, 0.98f);
			vertices[2].Set(7.57f, 1.38f);
			vertices[3].Set(7.21f, 1.52f);
			vertices[4].Set(6.71f, 1.5f);
			lowerChassispt3.Set(vertices, 5);
			
			b2PolygonShape lowerChassispt4;
			vertices[0].Set(0.675f, 1.15f);
			vertices[1].Set(7.57f, 1.38f);
			vertices[2].Set(7.21f, 1.52f);
			vertices[3].Set(0.64f, 1.32f);
			lowerChassispt4.Set(vertices, 4);

			b2PolygonShape upperChassispt1;
			vertices[0].Set(0.27f, 1.67f);
			vertices[1].Set(0.5f, 1.5f);
			vertices[2].Set(0.56f, 1.87f);
			upperChassispt1.Set(vertices, 3);

			b2PolygonShape upperChassispt2;
			vertices[0].Set(0.64f, 1.32f);
			vertices[1].Set(7.21f, 1.52f);
			vertices[2].Set(6.77f, 1.84f);
			vertices[3].Set(0.56f, 1.87f);
			vertices[4].Set(0.5f, 1.5f);
			upperChassispt2.Set(vertices, 5);

			b2PolygonShape gunpt1;
			vertices[0].Set(5.39f, 1.86f);
			vertices[1].Set(6.79f, 2.01f);
			vertices[2].Set(6.79f, 2.35f);
			vertices[3].Set(5.14f, 2.82f);
			gunpt1.Set(vertices, 4);

			b2PolygonShape gunpt2;
			gunpt2.SetAsBox(2.0f, 0.064f, b2Vec2(8.79f, 2.16f), 0.0f);

			b2PolygonShape turret;
			vertices[0].Set(2.63f, 2.84f);
			vertices[1].Set(2.5f, 1.87f);
			vertices[2].Set(5.59f, 1.84f);
			vertices[3].Set(5.34f, 2.82f);
			turret.Set(vertices, 4);

			b2PolygonShape miscFront1;
			vertices[0].Set(7.17f, 1.56f);
			vertices[1].Set(7.41f, 1.56f);
			vertices[2].Set(7.41f, 1.59f);
			vertices[3].Set(7.17f, 1.59f);
			miscFront1.Set(vertices, 4);

			b2PolygonShape miscFront2;
			vertices[0].Set(7.2f, 1.59f);
			vertices[1].Set(7.3f, 1.59f);
			vertices[2].Set(7.3f, 1.64f);
			vertices[3].Set(7.2f, 1.64f);
			miscFront2.Set(vertices, 4);

			b2PolygonShape miscFront3;
			vertices[0].Set(7.23f, 1.64f);
			vertices[1].Set(7.28f, 1.64f);
			vertices[2].Set(7.28f, 1.69f);
			vertices[3].Set(7.23f, 1.69f);
			miscFront3.Set(vertices, 4);

			b2PolygonShape miscFront4;
			vertices[0].Set(7.22f, 1.69f);
			vertices[1].Set(7.36f, 1.69f);
			vertices[2].Set(7.36f, 1.84f);
			vertices[3].Set(7.22f, 1.84f);
			vertices[4].Set(7.18f, 1.8f);
			vertices[5].Set(7.18f, 1.74f);
			miscFront4.Set(vertices, 6);

			b2PolygonShape miscBack1;
			vertices[0].Set(0.71f, 1.87f);
			vertices[1].Set(1.19f, 1.87f);
			vertices[2].Set(1.19f, 1.96f);
			vertices[3].Set(0.71f, 1.96f);
			miscBack1.Set(vertices, 4);

			b2PolygonShape miscBack2;
			vertices[0].Set(1.3f, 1.87f);
			vertices[1].Set(1.65f, 1.87f);
			vertices[2].Set(1.65f, 1.97f);
			vertices[3].Set(1.3f, 1.97f);
			miscBack2.Set(vertices, 4);

			b2PolygonShape miscBack3;
			vertices[0].Set(1.65f, 1.87f);
			vertices[1].Set(1.88f, 1.87f);
			vertices[2].Set(1.88f, 1.94f);
			vertices[3].Set(1.65f, 1.94f);
			miscBack3.Set(vertices, 4);

			b2PolygonShape miscBack4;
			vertices[0].Set(1.88f, 1.87f);
			vertices[1].Set(2.21f, 1.87f);
			vertices[2].Set(2.21f, 1.97f);
			vertices[3].Set(1.88f, 1.97f);
			miscBack4.Set(vertices, 4);

			b2PolygonShape miscHatch1;
			vertices[0].Set(2.44f, 1.98f);
			vertices[1].Set(2.52f, 1.97f);
			vertices[2].Set(2.6f, 2.65f);
			vertices[3].Set(2.53f, 2.66f);
			miscHatch1.Set(vertices, 4);

			b2PolygonShape miscHatch2;
			vertices[0].Set(2.39f, 2.16f);
			vertices[1].Set(2.46f, 2.15f);
			vertices[2].Set(2.47f, 2.19f);
			vertices[3].Set(2.39f, 2.2f);
			miscHatch2.Set(vertices, 4);

			b2PolygonShape miscHatch3;
			vertices[0].Set(2.39f, 2.16f);
			vertices[1].Set(2.42f, 2.15f);
			vertices[2].Set(2.43f, 2.44f);
			vertices[3].Set(2.4f, 2.44f);
			miscHatch3.Set(vertices, 4);

			b2PolygonShape miscHatch4;
			vertices[0].Set(2.4f, 2.4f);
			vertices[1].Set(2.5f, 2.39f);
			vertices[2].Set(2.51f, 2.43f);
			vertices[3].Set(2.4f, 2.44f);
			miscHatch4.Set(vertices, 4);

			b2PolygonShape miscTop1;
			vertices[0].Set(2.73f, 2.83f);
			vertices[1].Set(2.87f, 2.83f);
			vertices[2].Set(2.87f, 2.95f);
			vertices[3].Set(2.73f, 2.95f);
			miscTop1.Set(vertices, 4);

			b2PolygonShape miscTop2;
			vertices[0].Set(2.91f, 2.83f);
			vertices[1].Set(3.01f, 2.83f);
			vertices[2].Set(3.01f, 2.93f);
			vertices[3].Set(2.91f, 2.93f);
			miscTop2.Set(vertices, 4);

			b2PolygonShape miscTop3;
			vertices[0].Set(3.48f, 2.83f);
			vertices[1].Set(3.89f, 2.83f);
			vertices[2].Set(3.89f, 2.97f);
			vertices[3].Set(3.48f, 2.97f);
			miscTop3.Set(vertices, 4);

			b2PolygonShape miscTop4;
			vertices[0].Set(3.89f, 2.83f);
			vertices[1].Set(4.02f, 2.83f);
			vertices[2].Set(4.02f, 2.9f);
			vertices[3].Set(3.89f, 2.9f);
			miscTop4.Set(vertices, 4);

			b2PolygonShape miscTop5;
			vertices[0].Set(4.02f, 2.83f);
			vertices[1].Set(4.29f, 2.83f);
			vertices[2].Set(4.29f, 2.94f);
			vertices[3].Set(4.02f, 2.94f);
			miscTop5.Set(vertices, 4);

			b2PolygonShape miscTop6;
			vertices[0].Set(4.29f, 2.83f);
			vertices[1].Set(4.42f, 2.83f);
			vertices[2].Set(4.42f, 2.97f);
			vertices[3].Set(4.29f, 2.97f);
			miscTop6.Set(vertices, 4);

			b2PolygonShape miscTop7;
			vertices[0].Set(4.42f, 2.83f);
			vertices[1].Set(4.5f, 2.83f);
			vertices[2].Set(4.5f, 2.9f);
			vertices[3].Set(4.42f, 2.9f);
			miscTop7.Set(vertices, 4);

			b2PolygonShape miscTop8;
			vertices[0].Set(4.5f, 2.83f);
			vertices[1].Set(4.7f, 2.83f);
			vertices[2].Set(4.7f, 2.95f);
			vertices[3].Set(4.5f, 2.95f);
			miscTop8.Set(vertices, 4);

			b2PolygonShape miscTop9;
			vertices[0].Set(4.54f, 2.95f);
			vertices[1].Set(4.6f, 2.95f);
			vertices[2].Set(4.6f, 3.41f);
			vertices[3].Set(4.54f, 3.41f);
			miscTop9.Set(vertices, 4);

			b2PolygonShape miscTop10;
			vertices[0].Set(4.7f, 2.83f);
			vertices[1].Set(4.91f, 2.83f);
			vertices[2].Set(4.91f, 3.02f);
			vertices[3].Set(4.7f, 3.02f);
			miscTop10.Set(vertices, 4);

			b2PolygonShape miscGunStand;
			miscGunStand.SetAsBox(0.01f, 0.45f, b2Vec2(1.73f, 2.39f), 0.0f);

			b2PolygonShape miscGun;
			miscGun.SetAsBox(0.01f, 0.25f, b2Vec2(1.73f, 2.85f), 45.0f);

			b2PolygonShape miscGunStock;
			vertices[0].Set(4.7f, 2.83f);
			vertices[1].Set(4.91f, 2.83f);
			vertices[2].Set(4.91f, 3.02f);
			vertices[3].Set(4.7f, 3.02f);
			miscGunStock.Set(vertices, 4);

			b2PolygonShape miscGunGrip;
			vertices[0].Set(4.7f, 2.83f);
			vertices[1].Set(4.91f, 2.83f);
			vertices[2].Set(4.91f, 3.02f);
			vertices[3].Set(4.7f, 3.02f);
			miscGunGrip.Set(vertices, 4);

			b2PolygonShape miscCross1;
			vertices[0].Set(3.79f, 2.55f);
			vertices[1].Set(4.1f, 2.55f);
			vertices[2].Set(4.1f, 2.66f);
			vertices[3].Set(3.79f, 2.66f);
			miscCross1.Set(vertices, 4);

			b2PolygonShape miscCross2;
			vertices[0].Set(3.88f, 2.45f);
			vertices[1].Set(4.0f, 2.45f);
			vertices[2].Set(4.0f, 2.77f);
			vertices[3].Set(3.88f, 2.77f);
			miscCross2.Set(vertices, 4);

			b2PolygonShape miscSide1;
			vertices[0].Set(2.72f, 2.03f);
			vertices[1].Set(2.9f, 2.03f);
			vertices[2].Set(2.9f, 2.34f);
			vertices[3].Set(2.72f, 2.34f);
			miscSide1.Set(vertices, 4);

			b2PolygonShape miscSide2;
			vertices[0].Set(3.27f, 2.03f);
			vertices[1].Set(3.42f, 2.03f);
			vertices[2].Set(3.42f, 2.34f);
			vertices[3].Set(3.27f, 2.34f);
			miscSide2.Set(vertices, 4);

			b2PolygonShape miscSide3;
			vertices[0].Set(3.67f, 2.03f);
			vertices[1].Set(3.82f, 2.03f);
			vertices[2].Set(3.82f, 2.34f);
			vertices[3].Set(3.67f, 2.34f);
			miscSide3.Set(vertices, 4);

			b2PolygonShape miscSide4;
			vertices[0].Set(4.09f, 2.03f);
			vertices[1].Set(4.24f, 2.03f);
			vertices[2].Set(4.24f, 2.34f);
			vertices[3].Set(4.09f, 2.34f);
			miscSide4.Set(vertices, 4);

			b2PolygonShape miscSide5;
			vertices[0].Set(4.49f, 2.03f);
			vertices[1].Set(4.64f, 2.03f);
			vertices[2].Set(4.64f, 2.34f);
			vertices[3].Set(4.49f, 2.34f);
			miscSide5.Set(vertices, 4);

			b2PolygonShape miscSide6;
			vertices[0].Set(5.0f, 2.05f);
			vertices[1].Set(5.18f, 2.05f);
			vertices[2].Set(5.18f, 2.34f);
			vertices[3].Set(5.0f, 2.34f);
			miscSide6.Set(vertices, 4);

			b2PolygonShape miscSide7;
			vertices[0].Set(2.77f, 2.43f);
			vertices[1].Set(3.02f, 2.43f);
			vertices[2].Set(3.02f, 2.63f);
			vertices[3].Set(2.77f, 2.63f);
			miscSide7.Set(vertices, 4);

			b2PolygonShape miscSide8;
			vertices[0].Set(3.28f, 2.43f);
			vertices[1].Set(3.42f, 2.43f);
			vertices[2].Set(3.42f, 2.65f);
			vertices[3].Set(3.28f, 2.65f);
			miscSide8.Set(vertices, 4);

			b2PolygonShape miscSide9;
			vertices[0].Set(3.68f, 2.43f);
			vertices[1].Set(3.82f, 2.43f);
			vertices[2].Set(3.82f, 2.65f);
			vertices[3].Set(3.68f, 2.65f);
			miscSide9.Set(vertices, 4);

			b2PolygonShape miscSide10;
			vertices[0].Set(4.12f, 2.43f);
			vertices[1].Set(4.27f, 2.43f);
			vertices[2].Set(4.27f, 2.65f);
			vertices[3].Set(4.12f, 2.65f);
			miscSide10.Set(vertices, 4);

			b2PolygonShape miscSide11;
			vertices[0].Set(4.52f, 2.43f);
			vertices[1].Set(4.77f, 2.43f);
			vertices[2].Set(4.77f, 2.65f);
			vertices[3].Set(4.52f, 2.65f);
			miscSide11.Set(vertices, 4);

			b2PolygonShape miscSide12;
			vertices[0].Set(5.03f, 2.46f);
			vertices[1].Set(5.19f, 2.46f);
			vertices[2].Set(5.19f, 2.62f);
			vertices[3].Set(5.03f, 2.62f);
			miscSide12.Set(vertices, 4);

			b2CircleShape circle;
			circle.m_radius = 0.34f;

			b2FixtureDef fd;
			fd.shape = &lowerChassispt4;
			fd.density = m_mass;
			fd.filter.categoryBits = TANK;
			fd.filter.maskBits = 0xFFFF & ~TRACKS;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 0.0f);

			m_JadgTiger = m_world->CreateBody(&bd);

			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &lowerChassispt1;
			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &lowerChassispt2;
			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &lowerChassispt3;
			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &upperChassispt1;
			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &upperChassispt2;
			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &miscGunStand;
			m_JadgTiger->CreateFixture(&fd);
			fd.shape = &miscGun;
			m_JadgTiger->CreateFixture(&fd);
			m_JadgTiger->CreateFixture(&miscFront1, m_mass);
			m_JadgTiger->CreateFixture(&miscFront2, m_mass);
			m_JadgTiger->CreateFixture(&miscFront3, m_mass);
			m_JadgTiger->CreateFixture(&miscFront3, m_mass);
			m_JadgTiger->CreateFixture(&miscFront4, m_mass);
			m_JadgTiger->CreateFixture(&miscBack1, m_mass);
			m_JadgTiger->CreateFixture(&miscBack2, m_mass);
			m_JadgTiger->CreateFixture(&miscBack3, m_mass);
			m_JadgTiger->CreateFixture(&miscBack3, m_mass);
			m_JadgTiger->CreateFixture(&miscBack4, m_mass);

			m_turret = m_world->CreateBody(&bd); // block on tank
			m_turret->CreateFixture(&turret, 1.0f);
			m_turret->CreateFixture(&miscHatch1, 1.0f);
			m_turret->CreateFixture(&miscHatch2, 1.0f);
			m_turret->CreateFixture(&miscHatch3, 1.0f);
			m_turret->CreateFixture(&miscHatch4, 1.0f);
			m_turret->CreateFixture(&miscTop1, 1.0f);
			m_turret->CreateFixture(&miscTop2, 1.0f);
			m_turret->CreateFixture(&miscTop3, 1.0f);
			m_turret->CreateFixture(&miscTop4, 1.0f);
			m_turret->CreateFixture(&miscTop5, 1.0f);
			m_turret->CreateFixture(&miscTop6, 1.0f);
			m_turret->CreateFixture(&miscTop7, 1.0f);
			m_turret->CreateFixture(&miscTop8, 1.0f);
			m_turret->CreateFixture(&miscTop9, 1.0f);
			m_turret->CreateFixture(&miscTop10, 1.0f);
			m_turret->CreateFixture(&miscCross1, 1.0f);
			m_turret->CreateFixture(&miscCross2, 1.0f);
			m_turret->CreateFixture(&miscSide1, 1.0f);
			m_turret->CreateFixture(&miscSide2, 1.0f);
			m_turret->CreateFixture(&miscSide3, 1.0f);
			m_turret->CreateFixture(&miscSide4, 1.0f);
			m_turret->CreateFixture(&miscSide5, 1.0f);
			m_turret->CreateFixture(&miscSide6, 1.0f);
			m_turret->CreateFixture(&miscSide7, 1.0f);
			m_turret->CreateFixture(&miscSide8, 1.0f);
			m_turret->CreateFixture(&miscSide9, 1.0f);
			m_turret->CreateFixture(&miscSide10, 1.0f);
			m_turret->CreateFixture(&miscSide11, 1.0f);
			m_turret->CreateFixture(&miscSide12, 1.0f);

			m_gun = m_world->CreateBody(&bd);
			m_gun->CreateFixture(&gunpt1, 1.0f); // gun shroud
			m_gun->CreateFixture(&gunpt2, 1.0f); // gun barrel

			b2DistanceJointDef jtd;
			jtd.collideConnected = true;
			jtd.length = 2.0f;
			jtd.Initialize(m_JadgTiger, m_turret, b2Vec2(4, 1.84), b2Vec2(4, 2));
			m_tankTurret = (b2DistanceJoint*)m_world->CreateJoint(&jtd);


			b2RevoluteJointDef jtbd;
			jtbd.lowerAngle = -0.07f;
			jtbd.upperAngle = 0.15f;
			jtbd.enableLimit = true;
			jtbd.motorSpeed = 0.0f;
			jtbd.maxMotorTorque = 500.0f;
			jtbd.enableMotor = true;
			jtbd.Initialize(m_turret, m_gun, b2Vec2(5.32f, 2.29f));
			m_gunBarrel = (b2RevoluteJoint*)m_world->CreateJoint(&jtbd);

			//b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = m_mass;
			fd.friction = 0.9f;
			fd.filter.categoryBits = ODD_WHEEL;
			fd.filter.maskBits = 0xFFFF & ~EVEN_WHEEL;


			bd.position.Set(2.33f, 0.4f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&fd);

			bd.position.Set(3.33f, 0.4f);
			m_wheel3 = m_world->CreateBody(&bd);
			m_wheel3->CreateFixture(&fd);

			bd.position.Set(4.33f, 0.4f);
			m_wheel5 = m_world->CreateBody(&bd);
			m_wheel5->CreateFixture(&fd);

			bd.position.Set(5.33f, 0.4f);
			m_wheel7 = m_world->CreateBody(&bd);
			m_wheel7->CreateFixture(&fd);

			bd.position.Set(6.33f, 0.4f);
			m_wheel9 = m_world->CreateBody(&bd);
			m_wheel9->CreateFixture(&fd);

			fd.filter.categoryBits = EVEN_WHEEL;
			fd.filter.maskBits = 0xFFFF;


			bd.position.Set(1.36f, 0.67f);
			m_wheel0 = m_world->CreateBody(&bd);
			m_wheel0->CreateFixture(&fd);

			bd.position.Set(2.83f, 0.4f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&fd);

			bd.position.Set(3.83f, 0.4f);
			m_wheel4 = m_world->CreateBody(&bd);
			m_wheel4->CreateFixture(&fd);

			bd.position.Set(4.83f, 0.4f);
			m_wheel6 = m_world->CreateBody(&bd);
			m_wheel6->CreateFixture(&fd);

			bd.position.Set(5.83f, 0.4f);
			m_wheel8 = m_world->CreateBody(&bd);
			m_wheel8->CreateFixture(&fd);

			circle.m_radius = 0.4f;

			bd.position.Set(7.15f, 0.85f);
			m_wheel10 = m_world->CreateBody(&bd);
			m_wheel10->CreateFixture(&fd);

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_JadgTiger, m_wheel0, m_wheel0->GetPosition(), axis);
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel10, m_wheel10->GetPosition(), axis);
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring3 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring4 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel3, m_wheel3->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring5 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel4, m_wheel4->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring6 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel5, m_wheel5->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring7 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel6, m_wheel6->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring8 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel7, m_wheel7->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring9 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel8, m_wheel8->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring10 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_JadgTiger, m_wheel9, m_wheel9->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = m_torque;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring11 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			// track
			#define DEGTORAD 0.0174532925199432957f
			#define RADTODEG 57.295779513082320876f

			b2PolygonShape trackSegment;
			trackSegment.SetAsBox(0.08f, 0.05f);

			float t_x = 1.5f;
			float t_y = 1.05f;

			bd.type = b2_dynamicBody;
			bd.position.Set(t_x, t_y);
			t_x += 0.08f;

			fd.shape = &trackSegment;
			fd.density = density;
			fd.friction = 1.0f;
			fd.filter.categoryBits = TRACKS;
			fd.filter.maskBits = 0xFFFF & ~TANK;

			b2Body* m_link = m_world->CreateBody(&bd); //create first link
			m_link->CreateFixture(&fd);
			b2Body* m_link_coppy = m_link;

			b2RevoluteJointDef jdd;
			jdd.localAnchorA.Set(0.04f, 0.0f);
			jdd.localAnchorB.Set(-0.04f, 0.0f);

			for (int i = 0; i < 30; i++) //top part of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(t_x, t_y);

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				t_x += 0.17f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);
				
				m_link = m_track;//prepare for next iteration
			}

			float _x = 6.6f;
			float _y = 1.1f;
			float angle = 110.0f;
			float angle_2 = 0;

			for (int i = 0; i < 6; i++) //from the top part of the track to the front wheel
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x, _y);

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);

				_x += 0.067f;
				_y += 0.04f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

				m_link = m_track;//prepare for next iteration
			}
			angle = 110.0f;
			angle_2 = 0;
			float radius = 0.5f;
			_x = 7.15f;
			_y = 0.855f;

			for (int i = 0; i < 12; i++) //front wheel
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x + radius*cos(angle*DEGTORAD), _y + radius*sin(angle*DEGTORAD));

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);

				angle_2 -= 15.5f;
				angle -= 18.0f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

				m_link = m_track;//prepare for next iteration
			}

			angle_2 += 5.5f;
			_x = 7.05f;
			_y = 0.35f;
			for (int i = 0; i < 5; i++) //from the front wheel to the bottom part of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x, _y);

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);

				_x -= 0.124f;
				_y -= 0.08f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

				m_link = m_track;//prepare for next iteration
			}

			t_x = 6.5f;
			t_y = 0.045f;
			for (int i = 0; i < 28; i++) //bottom part of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(t_x, t_y);

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				t_x -= 0.17f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

				m_link = m_track;//prepare for next iteration
			}

			angle_2 = 162.2f;
			_x = 1.7f;
			_y = 0.06f;
			for (int i = 0; i < 9; i++) //from the back wheel to bottom of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x, _y);

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);

				_x -= 0.075f;
				_y += 0.022f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

				m_link = m_track;//prepare for next iteration
			}

			angle = 243.35f;
			angle_2 = 156.35f;
			radius = 0.38f;
			_x = 1.32f;
			_y = 0.67f;

			for (int i = 0; i < 13; i++) //back wheel
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x + radius*cos(angle*DEGTORAD), _y + radius*sin(angle*DEGTORAD));

				fd.shape = &trackSegment;
				fd.density = density;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);

				angle_2 -= 12.85f;
				angle -= 12.85f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

				m_link = m_track;//prepare for next iteration
			}

			jdd.bodyA = m_link;
			jdd.bodyB = m_link_coppy;
			jdd.collideConnected = true;
			m_joint = (b2RevoluteJoint*)m_world->CreateJoint(&jdd);

		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			if (m_torque < 100.0f)
				m_torque = 100.0f;
			m_speed = b2Min(4.0f, m_speed + 1.0f);
			break;

		case 's':
			m_speed = 0;
			m_torque = 100.0f;
			break;

		case 'd':
			m_speed = b2Max(-15.0f, m_speed - 1.875f);
			m_torque -= 4.0f;
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			break;

		case 'e':
			m_hz += 1.0f;
			break;

		case 'u':
			if (m_gunBarrel->GetJointAngle() < (0.15f)) // 15 degrees elevation from level
				m_gunBarrel->SetMotorSpeed(1.0f);
			break;

		case 'j':
			if (m_gunBarrel->GetJointAngle() > (0.07f)) // 7 degress deperession from level
				m_gunBarrel->SetMotorSpeed(-1.0f);
			break;


		case 'o':
			m_roundType++;
			if (m_roundType == 0)
				m_bulletVelocity = 950.0f;
			else if (m_roundType == 1)
				m_bulletVelocity = 845.0f;
			else if (m_roundType == 2)
				m_bulletVelocity = 880.0f;
			else
				m_roundType = -1;
			break;

		case 'm':
			m_gunBarrel->SetMotorSpeed(0.0f);

			if (m_shell != NULL)
			{
				m_world->DestroyBody(m_shell);
				m_shell = NULL;
			}

			b2PolygonShape shape;
			b2Vec2 vertices[5];
			vertices[0].Set(0.0f, 0.0f);
			vertices[1].Set(0.256f, 0.0f);
			vertices[2].Set(0.512f, 0.064f);
			vertices[3].Set(0.256f, 0.128f);
			vertices[4].Set(0.0f, 0.128);
			shape.Set(vertices, 5);

			b2FixtureDef fsd;
			fsd.shape = &shape;
			fsd.density = 28.3f;
			fsd.restitution = 0.05f;

			b2Vec2 posn = m_gunBarrel->GetAnchorB();
			float32 angle = m_gunBarrel->GetJointAngle()
				+ m_turret->GetAngle();

			posn.x += 2.0f * cos(angle);
			posn.y += 2.0f * sin(angle);

			b2BodyDef bsd;
			bsd.type = b2_dynamicBody;
			bsd.bullet = true;
			bsd.position.Set(posn.x + 2.0f, posn.y);
			bsd.angle = angle;

			m_shell = m_world->CreateBody(&bsd);
			m_shell->CreateFixture(&fsd);
			m_shell->SetLinearVelocity(b2Vec2(m_bulletVelocity*cos(angle),
				m_bulletVelocity*sin(angle)));
			break;

		}
		m_spring1->SetMotorSpeed(m_speed);
		m_spring2->SetMotorSpeed(m_speed);
		m_spring3->SetMotorSpeed(m_speed);
		m_spring4->SetMotorSpeed(m_speed);
		m_spring5->SetMotorSpeed(m_speed);
		m_spring6->SetMotorSpeed(m_speed);
		m_spring7->SetMotorSpeed(m_speed);
		m_spring8->SetMotorSpeed(m_speed);
		m_spring9->SetMotorSpeed(m_speed);
		m_spring10->SetMotorSpeed(m_speed);
		m_spring11->SetMotorSpeed(m_speed);

		m_spring1->SetSpringFrequencyHz(m_hz);
		m_spring2->SetSpringFrequencyHz(m_hz);
		m_spring3->SetSpringFrequencyHz(m_hz);
		m_spring4->SetSpringFrequencyHz(m_hz);
		m_spring5->SetSpringFrequencyHz(m_hz);
		m_spring6->SetSpringFrequencyHz(m_hz);
		m_spring7->SetSpringFrequencyHz(m_hz);
		m_spring8->SetSpringFrequencyHz(m_hz);
		m_spring9->SetSpringFrequencyHz(m_hz);
		m_spring10->SetSpringFrequencyHz(m_hz);
		m_spring11->SetSpringFrequencyHz(m_hz);
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e, gun up = u, gun down = j, fire = m, charge size = o");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g, current charge size = %g", m_hz, m_zeta, m_bulletVelocity);
		m_textLine += 15;

		settings->viewCenter.x = m_JadgTiger->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new JadgTiger;
	}

	b2Body* m_JadgTiger;

	b2Body* m_turret;

	b2Body* m_gun;

	b2Body* m_wheel0;
	b2Body* m_wheel1;
	b2Body* m_wheel2;
	b2Body* m_wheel3;
	b2Body* m_wheel4;
	b2Body* m_wheel5;
	b2Body* m_wheel6;
	b2Body* m_wheel7;
	b2Body* m_wheel8;
	b2Body* m_wheel9;
	b2Body* m_wheel10;
	b2Body* m_track;

	b2DistanceJoint* m_tankTurret;
	b2RevoluteJoint* m_gunBarrel;

	b2Body* m_shell;

	int m_roundType;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
	float32 m_mass;
	float32 m_torque;
	float32 m_bulletVelocity;

	b2WheelJoint* m_spring1;
	b2WheelJoint* m_spring2;
	b2WheelJoint* m_spring3;
	b2WheelJoint* m_spring4;
	b2WheelJoint* m_spring5;
	b2WheelJoint* m_spring6;
	b2WheelJoint* m_spring7;
	b2WheelJoint* m_spring8;
	b2WheelJoint* m_spring9;
	b2WheelJoint* m_spring10;
	b2WheelJoint* m_spring11;
	b2RevoluteJoint* m_joint;
};

#endif
