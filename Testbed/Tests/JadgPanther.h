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

#ifndef JADGPANTHER_H
#define JADGPANTHER_H

// This is a fun demo that shows off the wheel joint
class JadgPanther : public Test
{
public:
	JadgPanther()
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

			float32 hs[10] = {0.25f, 1.0f, 1.5f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

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
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 10.0f, 1.0f));
			ground->CreateFixture(&fd);

			x += 10.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 50.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 50.0f;
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

		// Car
		{
			m_shell = NULL;
			b2PolygonShape chassis;
			b2Vec2 vertices[10];
			vertices[0].Set(-2.28f, 1.25f);
			//vertices[1].Set(3.1f, 1.47f);
			vertices[1].Set(3.19f, 1.54f);
			vertices[2].Set(2.64f, 1.93f);
			vertices[3].Set(-2.72f, 1.93f);
			vertices[4].Set(-2.46f, 1.43f);
			chassis.Set(vertices, 5);

			b2PolygonShape turret;
			vertices[0].Set(-0.82f, 1.93f);
			vertices[1].Set(2.64f, 1.93f);
			vertices[2].Set(1.95f, 2.42f);
			vertices[3].Set(-0.52f, 2.64f);
			turret.Set(vertices, 4);

			b2PolygonShape rim;
			vertices[0].Set(1.99f, 2.38f);
			vertices[1].Set(3.19f, 1.53f);
			vertices[2].Set(3.17f, 1.69f);
			vertices[3].Set(2.11f, 2.38f);
			rim.Set(vertices, 4);


			b2PolygonShape shroud;
			vertices[0].Set(2.27f, 2.41f);
			vertices[1].Set(2.3f, 2.26f);
			vertices[2].Set(2.84f, 1.91f);
			vertices[3].Set(2.85f, 2.13f);
			vertices[4].Set(2.34f, 2.41f);
			shroud.Set(vertices, 5);

			b2PolygonShape slant1;
			vertices[0].Set(2.85f, 2.13f);
			vertices[1].Set(2.84f, 1.91f);
			vertices[2].Set(2.91f, 1.86f);
			vertices[3].Set(2.91f, 2.08f);
			slant1.Set(vertices, 4);

			b2PolygonShape turretextra1;
			vertices[0].Set(1.83f, 2.42f);
			vertices[1].Set(2.08f, 2.24f);
			vertices[2].Set(2.11f, 2.3f);
			vertices[3].Set(1.95f, 2.42f);
			turretextra1.Set(vertices, 4);

			b2PolygonShape turretextra2;
			vertices[0].Set(1.44f, 2.53f);
			vertices[1].Set(1.42f, 2.46f);
			vertices[2].Set(1.86f, 2.42f);
			vertices[3].Set(1.81f, 2.53f);
			turretextra2.Set(vertices, 4);

			b2PolygonShape turretextra3;
			vertices[0].Set(1.47f, 2.65f);
			vertices[1].Set(1.47f, 2.53f);
			vertices[2].Set(1.57f, 2.53f);
			vertices[3].Set(1.57f, 2.65f);
			turretextra3.Set(vertices, 4);

			b2PolygonShape turretextra4;
			vertices[0].Set(0.91f, 2.56f);
			vertices[1].Set(0.89f, 2.52f);
			vertices[2].Set(1.12f, 2.5f);
			vertices[3].Set(1.1f, 2.54f);
			turretextra4.Set(vertices, 4);

			b2PolygonShape turretextra5;
			vertices[0].Set(-0.57f, 2.6f);
			vertices[1].Set(-0.84f, 2.03f);
			vertices[2].Set(-0.8f, 2.0f);
			vertices[3].Set(-0.53f, 2.6f);
			turretextra5.Set(vertices, 4);

			b2PolygonShape turretextra6;
			vertices[0].Set(-0.72f, 2.56f);
			vertices[1].Set(-0.7f, 2.34f);
			vertices[2].Set(-0.6f, 2.56f);
			turretextra6.Set(vertices, 3);

			b2PolygonShape turretextra7;
			vertices[0].Set(-0.66f, 2.64f);
			vertices[1].Set(-0.66f, 2.56f);
			vertices[2].Set(-0.61f, 2.56f);
			vertices[3].Set(-0.6f, 2.64f);
			turretextra7.Set(vertices, 4);

			b2PolygonShape turretextra8;
			vertices[0].Set(0.39f, 2.66f);
			vertices[1].Set(0.38f, 2.56f);
			vertices[2].Set(0.4f, 2.56f);
			vertices[3].Set(0.41f, 2.66f);
			turretextra8.Set(vertices, 4);

			b2PolygonShape turretextra9;
			vertices[0].Set(0.41f, 2.66f);
			vertices[1].Set(0.41f, 2.63f);
			vertices[2].Set(0.56f, 2.63f);
			vertices[3].Set(0.56f, 2.66f);
			turretextra9.Set(vertices, 4);

			b2PolygonShape turretextra10;
			vertices[0].Set(0.5f, 2.63f);
			vertices[1].Set(0.5f, 2.58f);
			vertices[2].Set(0.56f, 2.58f);
			vertices[3].Set(0.56f, 2.63f);
			turretextra10.Set(vertices, 4);

			b2PolygonShape turretextra11;
			vertices[0].Set(0.4f, 2.58f);
			vertices[1].Set(0.4f, 2.56f);
			vertices[2].Set(0.56f, 2.54f);
			vertices[3].Set(0.56f, 2.58f);
			turretextra11.Set(vertices, 4);

			b2PolygonShape turretextra12;
			vertices[0].Set(0.06f, 2.72f);
			vertices[1].Set(0.05f, 2.59f);
			vertices[2].Set(0.38f, 2.56f);
			vertices[3].Set(0.39f, 2.68f);
			turretextra12.Set(vertices, 4);

			b2PolygonShape turretextra13;
			vertices[0].Set(-0.37f, 2.69f);
			vertices[1].Set(-0.38f, 2.62f);
			vertices[2].Set(0.05f, 2.59f);
			vertices[3].Set(0.05f, 2.64f);
			turretextra13.Set(vertices, 4);

			b2PolygonShape turretextra14;
			vertices[0].Set(-0.28f, 2.75f);
			vertices[1].Set(-0.28f, 2.68f);
			vertices[2].Set(-0.18f, 2.67f);
			vertices[3].Set(-0.19f, 2.75f);
			turretextra14.Set(vertices, 4);

			b2PolygonShape pillar1;
			vertices[0].Set(0.09f, 2.75f);
			vertices[1].Set(0.09f, 2.71f);
			vertices[2].Set(0.13f, 2.71f);
			vertices[3].Set(0.13f, 2.74f);
			pillar1.Set(vertices, 4);

			b2PolygonShape pillar2;
			vertices[0].Set(0.22f, 2.735f);
			vertices[1].Set(0.22f, 2.7f);
			vertices[2].Set(0.24f, 2.7f);
			vertices[3].Set(0.24f, 2.732f);
			pillar2.Set(vertices, 4);

			b2PolygonShape pillar3;
			vertices[0].Set(0.32f, 2.73f);
			vertices[1].Set(0.32f, 2.69f);
			vertices[2].Set(0.36f, 2.68f);
			vertices[3].Set(0.36f, 2.72f);
			pillar3.Set(vertices, 4);

			b2PolygonShape pillartop;
			vertices[0].Set(0.07f, 2.78f);
			vertices[1].Set(0.07f, 2.75f);
			vertices[2].Set(0.39f, 2.72f);
			vertices[3].Set(0.39f, 2.74f);
			pillartop.Set(vertices, 4);

			b2PolygonShape antenna;
			vertices[0].Set(-0.65f, 4.2f);
			vertices[1].Set(-0.65f, 2.64f);
			vertices[2].Set(-0.63f, 2.64f);
			vertices[3].Set(-0.64f, 4.2f);
			antenna.Set(vertices, 4);

			

			b2PolygonShape barrel;
			barrel.SetAsBox(2.285f,0.1f,b2Vec2(5.10f,1.96f),0.0f);

			b2PolygonShape barrel1;
			vertices[0].Set(2.81f, 2.08f);
			vertices[1].Set(2.81f, 1.83f);
			vertices[2].Set(3.67f, 1.83f);
			vertices[3].Set(3.67f, 2.08f);
			barrel1.Set(vertices, 4);

			b2PolygonShape slant2;
			vertices[0].Set(3.67f, 2.08f);
			vertices[1].Set(3.67f, 1.83f);
			vertices[2].Set(3.75f, 1.86f);
			vertices[3].Set(3.75f, 2.06f);
			slant2.Set(vertices, 4);

			b2PolygonShape barrel2;
			vertices[0].Set(3.75f, 2.06f);
			vertices[1].Set(3.75f, 1.86f);
			vertices[2].Set(6.31f, 1.9f);
			vertices[3].Set(6.31f, 2.03f);
			barrel2.Set(vertices, 4);

			b2PolygonShape slant3;
			vertices[0].Set(6.31f, 2.03f);
			vertices[1].Set(6.31f, 1.9f);
			vertices[2].Set(6.34f, 1.88f);
			vertices[3].Set(6.34f, 2.04f);
			slant3.Set(vertices, 4);

			b2PolygonShape barrelBox;
			vertices[0].Set(6.34f, 2.04f);
			vertices[1].Set(6.34f, 1.88f);
			vertices[2].Set(6.49f, 1.88f);
			vertices[3].Set(6.49f, 2.04f);
			barrelBox.Set(vertices, 4);

			b2PolygonShape barrelLip1;
			vertices[0].Set(6.49f, 2.04f);
			vertices[1].Set(6.49f, 2.02f);
			vertices[2].Set(6.63f, 2.02f);
			vertices[3].Set(6.63f, 2.08f);
			barrelLip1.Set(vertices, 4);

			b2PolygonShape barrelLip2;
			vertices[0].Set(6.49f, 1.9f);
			vertices[1].Set(6.49f, 1.87f);
			vertices[2].Set(6.63f, 1.83f);
			vertices[3].Set(6.63f, 1.9f);
			barrelLip2.Set(vertices, 4);

			b2PolygonShape barrelrec;
			vertices[0].Set(6.6f, 2.02f);
			vertices[1].Set(6.6f, 1.9f);
			vertices[2].Set(6.63f, 1.9f);
			vertices[3].Set(6.63f, 2.02f);
			barrelrec.Set(vertices, 4);

			b2PolygonShape barrelend1;
			vertices[0].Set(6.62f, 2.06f);
			vertices[1].Set(6.62f, 2.02f);
			vertices[2].Set(6.85f, 2.02f);
			vertices[3].Set(6.82f, 2.06f);
			barrelend1.Set(vertices, 4);

			b2PolygonShape barrelend2;
			vertices[0].Set(6.62f, 2.02f);
			vertices[1].Set(6.62f, 1.89f);
			vertices[2].Set(6.65f, 1.89f);
			vertices[3].Set(6.65f, 2.02f);
			barrelend2.Set(vertices, 4);

			b2PolygonShape barrelend3;
			vertices[0].Set(6.62f, 1.89f);
			vertices[1].Set(6.62f, 1.85f);
			vertices[2].Set(6.82f, 1.85f);
			vertices[3].Set(6.85f, 1.89f);
			barrelend3.Set(vertices, 4);

			b2PolygonShape barrelend4;
			vertices[0].Set(6.79f, 2.02f);
			vertices[1].Set(6.79f, 1.89f);
			vertices[2].Set(6.85f, 1.89f);
			vertices[3].Set(6.85f, 2.02f);
			barrelend4.Set(vertices, 4);
		
			b2PolygonShape chassis2;
			vertices[0].Set(-1.74f, 0.52f);
			vertices[1].Set(3.09f, 0.52f);
			vertices[2].Set(3.88f, 1.06f);
			vertices[3].Set(3.19f, 1.54f);
			vertices[4].Set(-2.28f, 1.25f);
			chassis2.Set(vertices, 5);

			b2PolygonShape sidelip;
			vertices[0].Set(-2.5f, 1.5f);
			vertices[1].Set(-2.29f, 1.25f);
			vertices[2].Set(3.43f, 1.47f);
			vertices[3].Set(3.42f, 1.51f);
			sidelip.Set(vertices, 4);
			
			b2PolygonShape sidelipext1;
			vertices[0].Set(3.42f, 1.51f);
			vertices[1].Set(3.43f, 1.47f);
			vertices[2].Set(3.68f, 1.42f);
			vertices[3].Set(3.69f, 1.45f);
			sidelipext1.Set(vertices, 4);

			b2PolygonShape sidelipext2;
			vertices[0].Set(3.69f, 1.45f);
			vertices[1].Set(3.68f, 1.42f);
			vertices[2].Set(3.84f, 1.33f);
			vertices[3].Set(3.86f, 1.34f);
			sidelipext2.Set(vertices, 4);

			b2PolygonShape hitch;
			//vertices[0].Set(-2.31f, 1.24f);
			vertices[0].Set(-2.39f, 1.29f);
			vertices[1].Set(-2.39f, 1.04f);
			//vertices[2].Set(-2.36f, 0.99f);
			//vertices[3].Set(-2.36f, 1.01f);
			vertices[2].Set(-2.12f, 1.06f);
			vertices[3].Set(-2.31f, 1.29f);
			//vertices[7].Set(-2.28f, 1.24f);
			hitch.Set(vertices, 4);

			b2PolygonShape hitchext;
			vertices[0].Set(-2.39f, 1.04f);
			vertices[1].Set(-2.39f, 0.99f);
			vertices[2].Set(-2.36f, 0.99f);
			vertices[3].Set(-2.36f, 1.04f);
			hitchext.Set(vertices, 4);

			b2PolygonShape exhaust1;
			vertices[0].Set(-2.7f, 1.36f);
			vertices[1].Set(-2.65f, 1.27f);
			vertices[2].Set(-2.31f, 1.29f);
			vertices[3].Set(-2.5f, 1.5f);
			exhaust1.Set(vertices, 4);

			b2PolygonShape exhaust2;
			vertices[0].Set(-2.94f, 1.77f);
			vertices[1].Set(-2.7f, 1.36f);
			vertices[2].Set(-2.5f, 1.5f);
			vertices[3].Set(-2.65f, 1.8f);
			exhaust2.Set(vertices, 4);

			b2PolygonShape sidebox1;
			vertices[0].Set(-0.5f, 1.9f);
			vertices[1].Set(-0.5f, 1.72f);
			vertices[2].Set(-0.46f, 1.72f);
			vertices[3].Set(-0.46f, 1.9f);
			sidebox1.Set(vertices, 4);

			b2PolygonShape sidebox2;
			vertices[0].Set(-0.46f, 1.89f);
			vertices[1].Set(-0.46f, 1.73f);
			vertices[2].Set(-0.12f, 1.73f);
			vertices[3].Set(-0.12f, 1.89f);
			sidebox2.Set(vertices, 4);

			b2PolygonShape sidebox3;
			vertices[0].Set(-0.12f, 1.9f);
			vertices[1].Set(-0.12f, 1.72f);
			vertices[2].Set(-0.08f, 1.72f);
			vertices[3].Set(-0.08f, 1.9f);
			sidebox3.Set(vertices, 4);

			b2PolygonShape pipe;
			vertices[0].Set(-2.97f, 2.07f);
			vertices[1].Set(-2.97f, 1.99f);
			vertices[2].Set(-2.9f, 1.99f);
			vertices[3].Set(-2.9f, 2.07f);
			pipe.Set(vertices, 4);

			b2PolygonShape pipe2;
			vertices[0].Set(-2.9f, 2.07f);
			vertices[1].Set(-2.9f, 1.99f);
			vertices[2].Set(-2.85f, 1.96f);
			vertices[3].Set(-2.82f, 2.04f);
			pipe2.Set(vertices, 4);

			b2PolygonShape pipe3;
			vertices[0].Set(-2.82f, 2.04f);
			vertices[1].Set(-2.85f, 1.96f);
			vertices[2].Set(-2.67f, 1.77f);
			vertices[3].Set(-2.65f, 1.8f);
			pipe3.Set(vertices, 4);

			b2PolygonShape back1;
			vertices[0].Set(-2.46f, 1.98f);
			vertices[1].Set(-2.5f, 1.93f);
			vertices[2].Set(-2.2f, 1.93f);
			vertices[3].Set(-2.2f, 1.98f);
			back1.Set(vertices, 4);

			b2PolygonShape back2;
			vertices[0].Set(-2.19f, 2.01f);
			vertices[1].Set(-2.19f, 1.93f);
			vertices[2].Set(-2.05f, 1.93f);
			vertices[3].Set(-2.05f, 2.1f);
			back2.Set(vertices, 4);

			b2PolygonShape back3;
			vertices[0].Set(-1.95f, 1.99f);
			vertices[1].Set(-1.95f, 1.93f);
			vertices[2].Set(-1.31f, 1.93f);
			vertices[3].Set(-1.31f, 1.99f);
			back3.Set(vertices, 4);

			b2PolygonShape back4;
			vertices[0].Set(-1.26f, 1.99f);
			vertices[1].Set(-1.26f, 1.93f);
			vertices[2].Set(-1.18f, 1.93f);
			vertices[3].Set(-1.18f, 1.99f);
			back4.Set(vertices, 4);

			b2PolygonShape back5;
			vertices[0].Set(-0.88f, 2.02f);
			vertices[1].Set(-0.93f, 1.99f);
			vertices[2].Set(-0.93f, 1.93f);
			vertices[3].Set(-0.83f, 1.93f);
			vertices[4].Set(-0.84f, 1.99f);
			back5.Set(vertices, 5);

			b2PolygonShape light1;
			vertices[0].Set(3.26f, 1.53f);
			vertices[1].Set(3.42f, 1.53f);
			vertices[2].Set(3.37f, 1.6f);
			vertices[3].Set(3.31f, 1.6f);
			light1.Set(vertices, 4);

			b2PolygonShape light2;
			vertices[0].Set(3.32f, 1.63f);
			vertices[1].Set(3.32f, 1.61f);
			vertices[2].Set(3.36f, 1.61f);
			vertices[3].Set(3.36f, 1.63f);
			light2.Set(vertices, 4);

			b2PolygonShape light3;
			vertices[0].Set(3.29f, 1.8f);
			vertices[1].Set(3.29f, 1.63f);
			vertices[2].Set(3.39f, 1.63f);
			vertices[3].Set(3.39f, 1.8f);
			light3.Set(vertices, 4);

			b2PolygonShape light4;
			vertices[0].Set(3.39f, 1.8f);
			vertices[1].Set(3.39f, 1.63f);
			vertices[2].Set(3.43f, 1.7f);
			vertices[3].Set(3.43f, 1.74f);
			light4.Set(vertices, 4);

			b2PolygonShape cross1;
			vertices[0].Set(1.62f, 2.36f);
			vertices[1].Set(1.62f, 2.07f);
			vertices[2].Set(1.73f, 2.07f);
			vertices[3].Set(1.73f, 2.36f);
			cross1.Set(vertices, 4);

			b2PolygonShape cross2;
			vertices[0].Set(1.53f, 2.27f);
			vertices[1].Set(1.53f, 2.16f);
			vertices[2].Set(1.82f, 2.16f);
			vertices[3].Set(1.82f, 2.27f);
			cross2.Set(vertices, 4);

			b2PolygonShape cross3;
			vertices[0].Set(1.65f, 2.36f);
			vertices[1].Set(1.65f, 2.07f);
			vertices[2].Set(1.71f, 2.07f);
			vertices[3].Set(1.71f, 2.36f);
			cross3.Set(vertices, 4);

			b2PolygonShape cross4;
			vertices[0].Set(1.53f, 2.24f);
			vertices[1].Set(1.53f, 2.19f);
			vertices[2].Set(1.82f, 2.19f);
			vertices[3].Set(1.82f, 2.24f);
			cross4.Set(vertices, 4);

			b2PolygonShape bodyextra;
			vertices[0].Set(2.34f, 2.13f);
			vertices[1].Set(2.3f, 2.08f);
			vertices[2].Set(2.75f, 1.76f);
			vertices[3].Set(2.78f, 1.82f);
			bodyextra.Set(vertices, 4);

			b2PolygonShape machineGunRound;
			vertices[0].Set(3.02f, 1.66f);
			vertices[1].Set(3.02f, 1.72f);
			vertices[2].Set(3.01f, 1.76f);
			vertices[3].Set(2.98f, 1.78f);
			vertices[4].Set(2.93f, 1.78f);
			vertices[5].Set(2.88f, 1.76f);
			machineGunRound.Set(vertices, 6);

			b2PolygonShape barrelMachineGun1;
			barrelMachineGun = &barrelMachineGun1;
			barrelMachineGun->SetAsBox(0.095f,0.020f,b2Vec2(3.08f,1.75f),0.0f);


			b2CircleShape circle;
			circle.m_radius = 0.305f;
			
			b2FixtureDef fdb;
			fdb.shape = &chassis;
			fdb.density = 1.0f;
			fdb.friction = 0.9f;
			fdb.filter.categoryBits = 0x0002;
			fdb.filter.maskBits = 0xFFFF &~ 0x0004 & ~0x0006;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			//bd.position.Set(0.0f, 1.0f);
			m_car = m_world->CreateBody(&bd);
			m_car->CreateFixture(&fdb);
			
			
			fdb.shape = &chassis2;
			m_car->CreateFixture(&fdb);
			
			m_turret = m_world->CreateBody(&bd);
			m_car->CreateFixture(&turret, 1.0f);
			m_car->CreateFixture(&rim, 1.0f);
			m_car->CreateFixture(&shroud, 1.0f);
			m_car->CreateFixture(&slant1, 1.0f);
			m_car->CreateFixture(&turretextra1, 1.0f);
			m_car->CreateFixture(&turretextra2, 1.0f);
			m_car->CreateFixture(&turretextra3, 1.0f);
			m_car->CreateFixture(&turretextra4, 1.0f);
			m_car->CreateFixture(&turretextra5, 1.0f);
			m_car->CreateFixture(&turretextra6, 1.0f);
			m_car->CreateFixture(&turretextra7, 1.0f);
			m_car->CreateFixture(&turretextra8, 1.0f);
			m_car->CreateFixture(&turretextra9, 1.0f);
			m_car->CreateFixture(&turretextra10, 1.0f);
			m_car->CreateFixture(&turretextra11, 1.0f);
			m_car->CreateFixture(&turretextra12, 1.0f);
			m_car->CreateFixture(&turretextra13, 1.0f);
			m_car->CreateFixture(&turretextra14, 1.0f);
			m_car->CreateFixture(&bodyextra, 1.0f);
			m_car->CreateFixture(&antenna, 1.0f);
			m_car->CreateFixture(&pillar1, 1.0f);
			m_car->CreateFixture(&pillar2, 1.0f);
			m_car->CreateFixture(&pillar3, 1.0f);
			m_car->CreateFixture(&pillartop, 1.0f);
			m_car->CreateFixture(&sidelip, 1.0f);
			m_car->CreateFixture(&sidelipext1, 1.0f);
			m_car->CreateFixture(&sidelipext2, 1.0f);
			m_car->CreateFixture(&hitch, 1.0f);
			m_car->CreateFixture(&hitchext, 1.0f);
			m_car->CreateFixture(&exhaust1, 1.0f);
			m_car->CreateFixture(&exhaust2, 1.0f);
			m_car->CreateFixture(&sidebox1, 1.0f);
			m_car->CreateFixture(&sidebox2, 1.0f);
			m_car->CreateFixture(&sidebox3, 1.0f);
			m_car->CreateFixture(&pipe, 1.0f);
			m_car->CreateFixture(&pipe2, 1.0f);
			m_car->CreateFixture(&pipe3, 1.0f);
			m_car->CreateFixture(&back1, 1.0f);
			m_car->CreateFixture(&back2, 1.0f);
			m_car->CreateFixture(&back3, 1.0f);
			m_car->CreateFixture(&back4, 1.0f);
			m_car->CreateFixture(&back5, 1.0f);
			m_car->CreateFixture(&light1, 1.0f);
			m_car->CreateFixture(&cross1, 1.0f);
			m_car->CreateFixture(&cross2, 1.0f);
			m_car->CreateFixture(&cross3, 1.0f);
			m_car->CreateFixture(&cross4, 1.0f);
			m_car->CreateFixture(&light2, 1.0f);
			m_car->CreateFixture(&light3, 1.0f);
			m_car->CreateFixture(&light4, 1.0f);
			m_car->CreateFixture(&machineGunRound,1.0f);
			m_car->CreateFixture(&barrelMachineGun1,1.0f);

			m_barrel = m_world->CreateBody(&bd);
			m_barrel->CreateFixture(&barrel1, 1.0f);
			m_barrel->CreateFixture(&slant2, 1.0f);
			m_barrel->CreateFixture(&barrel2, 1.0f);
			m_barrel->CreateFixture(&slant3, 1.0f);
			m_barrel->CreateFixture(&barrelBox, 1.0f);
			m_barrel->CreateFixture(&barrelLip1, 1.0f);
			m_barrel->CreateFixture(&barrelLip2, 1.0f);
			m_barrel->CreateFixture(&barrelrec, 1.0f);
			m_barrel->CreateFixture(&barrelend1, 1.0f);
			m_barrel->CreateFixture(&barrelend2, 1.0f);
			m_barrel->CreateFixture(&barrelend3, 1.0f);
			m_barrel->CreateFixture(&barrelend4, 1.0f);

			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;
			fd.filter.categoryBits = 0x0004;
			fd.filter.maskBits = 0xFFFF & ~ 0x0006;


			bd.position.Set(-1.97f, 0.77f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&fd);

			circle.m_radius = 0.44f;
			bd.position.Set(-0.74f, 0.52f);
			m_wheel3 = m_world->CreateBody(&bd);
			m_wheel3->CreateFixture(&fd);

			bd.position.Set(0.27f, 0.52f);
			m_wheel5 = m_world->CreateBody(&bd);
			m_wheel5->CreateFixture(&fd);

			bd.position.Set(1.5f, 0.52f);
			m_wheel7 = m_world->CreateBody(&bd);
			m_wheel7->CreateFixture(&fd);

			
			bd.position.Set(2.66f, 0.52f);
			m_wheel9 = m_world->CreateBody(&bd);
			m_wheel9->CreateFixture(&fd);

			//
			fd.filter.categoryBits = 0x0006;
			fd.filter.maskBits = 0xFFFF;


			bd.position.Set(-1.3f, 0.52f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&fd);

			bd.position.Set(-0.19f, 0.52f);
			m_wheel4 = m_world->CreateBody(&bd);
			m_wheel4->CreateFixture(&fd);

			bd.position.Set(0.93f, 0.52f);
			m_wheel6 = m_world->CreateBody(&bd);
			m_wheel6->CreateFixture(&fd);


			bd.position.Set(2.09f, 0.52f);
			m_wheel8 = m_world->CreateBody(&bd);
			m_wheel8->CreateFixture(&fd);

			
			circle.m_radius = 0.385f;
			bd.position.Set(3.48f, 0.86f);
			m_wheel10 = m_world->CreateBody(&bd);
			m_wheel10->CreateFixture(&fd);

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_car, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel3, m_wheel3->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring3 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel4, m_wheel4->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring4 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel5, m_wheel5->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring5 = (b2WheelJoint*)m_world->CreateJoint(&jd);
			//
			jd.Initialize(m_car, m_wheel6, m_wheel6->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring6 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel7, m_wheel7->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring7 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel8, m_wheel8->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring8 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel9, m_wheel9->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring9 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel10, m_wheel10->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring10 = (b2WheelJoint*)m_world->CreateJoint(&jd);


			//tracks
			b2PolygonShape shape;
			b2RevoluteJointDef jtrd;

			jtrd.collideConnected = false;
			jtrd.localAnchorA.Set(0.1f,0.0f);  // Joint is set to be at the end of each track section
			jtrd.localAnchorB.Set(-0.1f,0.0f); // Joint is set to be at the end of each track section

			fd.filter.categoryBits = 0x0008;
			fd.filter.maskBits = 0xFFFF &~0x0002;
			fd.density = 1.0f;
			shape.SetAsBox(0.1f,0.02f);
			fd.shape = &shape;

			//Along the top from left to right
			for(int i=0; i<26; i++)
			{
				if(!i) // if the counter i is at 0.
				{
					bd.position.Set(-1.97f,1.10f);
					m_prev_track = m_world->CreateBody(&bd);
					m_prev_track->CreateFixture(&fd);
					m_first_track = m_prev_track;	// Set the position of the first track so we can complete the circuit
				}	
				else
				{
					bd.position.Set(-1.97f+i*0.2f, 1.10f );//0.83f);
					m_track = m_world->CreateBody(&bd);
					m_track->CreateFixture(&fd);
					jtrd.bodyA = m_prev_track;
					jtrd.bodyB = m_track;
					m_world->CreateJoint(&jtrd);
					m_prev_track = m_track;
				}
			}

			b2Vec2 a, p;	// Two vector: a for actual position of track, p for the centre of rotation (the wheel hub).
			a.x = 3.36;		
			a.y = 1.24;
			p.x = 3.38f; // position of the rear wheel hub.
			p.y = 0.86f;

			float theta, theta_old;

			//Around front wheel at (1.0,0.4) of radius 0.4;
			for(int i=0; i<8; i++)
			{
				theta = -i*b2_pi/7.0f;
				theta_old = -(i-1)*b2_pi/7.0f;	// Keep track of previous theta so anchor A is set correctly.

				if (i>1) jtrd.localAnchorA.Set(0.1f*cos(theta_old),0.1f*sin(theta_old));
				jtrd.localAnchorB.Set(-0.1f*cos(theta),-0.1f*sin(theta));

				shape.SetAsBox(0.1f,0.02f,b2Vec2(0.0f,0.0f),theta); // box shape rotated through theta
				bd.position.Set(a.x*cos(theta)-a.y*sin(theta)+p.x*(1-cos(theta))+ p.y*sin(theta), 
										a.x*sin(theta)+a.y*cos(theta)+p.y*(1-cos(theta))- p.x*sin(theta) + 0.05f ); // position box according to your notes

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				jtrd.bodyA = m_prev_track;
				jtrd.bodyB = m_track;

				m_world->CreateJoint(&jtrd);
				m_prev_track = m_track;
			}

			jtrd.localAnchorA.Set(0.1f*cos(theta),0.1f*sin(theta)); //Reset anchors for tracks outside rotation
			jtrd.localAnchorB.Set(0.1f,0.0f);
			//bottom tracks
			for(int i=0; i<20; i++)
			{
				jtrd.localAnchorA.Set(-0.1f,0.0f); //Reset Anchor A.
				
				bd.position.Set(p.x-(i+1)*0.2f, 0.025f);
				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);
				jtrd.bodyA = m_prev_track;
				jtrd.bodyB = m_track;
				m_world->CreateJoint(&jtrd);
				m_prev_track = m_track;
			}

			a.x = -1.97f; // position at base of rear wheel
			a.y = 0.46f;
			p.x = -1.97;	// position of the rear wheel hub.
			p.y = 0.78f;

			//Around back wheel at (-1.0,0.4) of radius 0.4;
			for(int i=0; i<8; i++)
			{
				theta = -i*b2_pi/6.0f;
				if(i>1)theta_old = -(i-1)*b2_pi/6.0f; // Keep track of previous theta so anchor A is set correctly.

				if ( i>1 ) jtrd.localAnchorA.Set(-0.1f*cos(theta_old),-0.1f*sin(theta_old));
				jtrd.localAnchorB.Set(0.1f*cos(theta),0.1f*sin(theta));

				shape.SetAsBox(0.1f,0.02f,b2Vec2(0.0f,0.0f),theta);
				bd.position.Set(a.x*cos(theta)-a.y*sin(theta)+p.x*(1-cos(theta))+ p.y*sin(theta), 
										a.x*sin(theta)+a.y*cos(theta)+p.y*(1-cos(theta))- p.x*sin(theta) + 0.05f );

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				jtrd.bodyA = m_prev_track;
				jtrd.bodyB = m_track;

				m_world->CreateJoint(&jtrd);
				m_prev_track = m_track;
			}

			//Connect last link to first link
			jtrd.localAnchorA.Set(-0.1f*cos(theta),-0.1f*sin(theta)); 
			jtrd.localAnchorB.Set(-0.1f,0.0);
			jtrd.bodyB = m_first_track;
			jtrd.bodyA = m_prev_track;
			m_world->CreateJoint(&jtrd);
			b2RevoluteJointDef jtbd;
			jtbd.lowerAngle = -0.008f * b2_pi;
			jtbd.upperAngle = 0.0145f * b2_pi;
			jtbd.enableLimit = true;
			jtbd.motorSpeed = 0.0f;
			jtbd.maxMotorTorque = 100.0f;
			jtbd.enableMotor = true;
			jtbd.Initialize( m_car, m_barrel, b2Vec2(1.45f, 2.175f) );
			m_turretbarrel = (b2RevoluteJoint*)m_world->CreateJoint(&jtbd);
	
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
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
			break;

		case 's':
			m_spring1->SetMotorSpeed(0.0f);
			m_spring2->SetMotorSpeed(0.0f);
			m_spring3->SetMotorSpeed(0.0f);
			m_spring4->SetMotorSpeed(0.0f);
			m_spring5->SetMotorSpeed(0.0f);
			m_spring6->SetMotorSpeed(0.0f);
			m_spring7->SetMotorSpeed(0.0f);
			m_spring8->SetMotorSpeed(0.0f);
			m_spring9->SetMotorSpeed(0.0f);
			m_spring10->SetMotorSpeed(0.0f);
			break;

		case 'd':
			m_spring1->SetMotorSpeed(-m_speed);
			m_spring2->SetMotorSpeed(-m_speed);
			m_spring3->SetMotorSpeed(-m_speed);
			m_spring4->SetMotorSpeed(-m_speed);
			m_spring5->SetMotorSpeed(-m_speed);
			m_spring6->SetMotorSpeed(-m_speed);
			m_spring7->SetMotorSpeed(-m_speed);
			m_spring8->SetMotorSpeed(-m_speed);
			m_spring9->SetMotorSpeed(-m_speed);
			m_spring10->SetMotorSpeed(-m_speed);
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

		case 'u':
			if( m_turretbarrel->GetJointAngle() < (0.0145f * b2_pi) )
			m_turretbarrel->SetMotorSpeed( 1.0f );
			break;

		case 'j':
			if( m_turretbarrel->GetJointAngle() > -0.008f * b2_pi )
			m_turretbarrel->SetMotorSpeed( -1.0f );
			break;

		case 'm':
		{
			m_turretbarrel->SetMotorSpeed(0.0f);

			if(m_shell!=NULL)
			{
				m_world->DestroyBody(m_shell);
				m_shell = NULL;
			}

			b2PolygonShape shape;
			shape.SetAsBox(0.2f, 0.1f);


			b2FixtureDef fsd;
			fsd.shape = &shape;
			fsd.density = 20.0f;
			fsd.restitution = 0.05f;
			//fsd.filter.categoryBits = 0x0004;
			//fsd.filter.maskBits = 0xFFFF & ~0x0008;

			b2Vec2 posn = m_turretbarrel->GetAnchorB();
			float32 angle = m_turretbarrel->GetJointAngle()+m_turret->GetAngle();

			posn.x += 5.6f*cos(angle);
			posn.y += 0.1f*sin(angle);

			b2BodyDef bsd;
			bsd.type = b2_dynamicBody;
			bsd.bullet = true;
			bsd.position.Set(posn.x, posn.y);

			m_shell = m_world->CreateBody(&bsd);
			m_shell->CreateFixture(&fsd);

			m_shell->SetLinearVelocity(b2Vec2(50.0f*cos(angle), 50.0f*sin(angle)));

		}
	}
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e, fire = m, turret up = u, turret down = j");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;

		settings->viewCenter.x = m_car->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new JadgPanther;
	}

	b2Body* m_car;
	b2Body* m_chassis;
	b2Body* m_chassis2;
	b2Body* m_turret;
	b2Body* m_rim;
	b2Body* m_shroud;
	b2Body* m_machineGunRound;
	b2Body* m_slant1;
	b2Body* m_turretextra1;
	b2Body* m_turretextra2;
	b2Body* m_turretextra3;
	b2Body* m_turretextra4;
	b2Body* m_turretextra5;
	b2Body* m_turretextra6;
	b2Body* m_turretextra7;
	b2Body* m_turretextra8;
	b2Body* m_turretextra9;
	b2Body* m_turretextra10;
	b2Body* m_turretextra11;
	b2Body* m_turretextra12;
	b2Body* m_turretextra13;
	b2Body* m_turretextra14;
	b2Body* m_bodyextra;
	b2Body* m_antenna;
	b2Body* m_pillar1;
	b2Body* m_pillar2;
	b2Body* m_pillar3;
	b2Body* m_pillartop;
	b2Body* m_sidelip;
	b2Body* m_sidelipext1;
	b2Body* m_sidelipext2;
	b2Body* m_hitch;
	b2Body* m_hitchext;
	b2Body* m_exhaust1;
	b2Body* m_exhaust2;
	b2Body* m_sidebox1;
	b2Body* m_sidebox2;
	b2Body* m_sidebox3;
	b2Body* m_pipe;
	b2Body* m_pipe2;
	b2Body* m_pipe3;
	b2Body* m_back1;
	b2Body* m_back2;
	b2Body* m_back3;
	b2Body* m_back4;
	b2Body* m_back5;
	b2Body* m_light1;
	b2Body* m_light2;
	b2Body* m_light3;
	b2Body* m_light4;
	b2Body* m_cross1;
	b2Body* m_cross2;
	b2Body* m_cross3;
	b2Body* m_cross4;
	b2Body* m_shell;
	b2Body* m_barrel;
	b2Body* m_barrel1;
	b2Body* m_slant2;
	b2Body* m_barrel2;
	b2Body* m_slant3;
	b2Body* m_barrelBox;
	b2Body* m_barrelLip1;
	b2Body* m_barrelLip2;
	b2Body* m_barrelrec;
	b2Body* m_barrelend1;
	b2Body* m_barrelend2;
	b2Body* m_barrelend3;
	b2Body* m_barrelend4;
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
	b2BodyDef chassis;
	b2Body* m_track;
	b2Body* m_prev_track;
	b2Body* m_first_track;
	b2PolygonShape* barrelMachineGun;
	b2PolygonShape barrelMachineGun1;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
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
	b2RevoluteJoint* m_turretbarrel;
	b2DistanceJoint* m_carturret;
	b2WeldJoint* m_chasisturret;
};

#endif
