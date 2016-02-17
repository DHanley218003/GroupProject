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

#ifndef T34_85_H
#define T34_85_H

class T34_85 : public Test
{
public:
	T34_85()
	{		
		m_hz = 4.0f;
		m_zeta = 0.7f;
		m_speed = 10.0f;

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

			float32 hs[10] = {0.25f, 1.0f, 4.0f, 4.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

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
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 20.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 20.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 10.0f, 5.0f));
			ground->CreateFixture(&fd);
			 
			x += 10.0f;
			float32 y = 5.0f;
			for (int32 i = 0; i < 10; ++i)
			{
				if (i % 2 == 0)
				{
					shape.Set(b2Vec2(x, y), b2Vec2(x + 1.0f, y + 0.5f));
					ground->CreateFixture(&fd);
					y = 5.5f;
				}
				else
				{
					shape.Set(b2Vec2(x, y), b2Vec2(x + 1.0f, y - 0.5f));
					ground->CreateFixture(&fd);
					y = 5.0f;
				}
				x += 1.0f;
			}

			shape.Set(b2Vec2(x, 5.0f), b2Vec2(x + 10.0f, 3.0f));
			ground->CreateFixture(&fd);

			x += 10.0f;
			shape.Set(b2Vec2(x, 3.0f), b2Vec2(x + 40.0f, 3.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 3.0f), b2Vec2(x + 15.0f, 10.0f));
			ground->CreateFixture(&fd);
			
			x += 15.0f;
			shape.Set(b2Vec2(x, 10.0f), b2Vec2(x + 5.0f, 10.0f));
			ground->CreateFixture(&fd);
			
			x += 5.0f;
			shape.Set(b2Vec2(x, 10.0f), b2Vec2(x, 7.0f));
			ground->CreateFixture(&fd);

			shape.Set(b2Vec2(x, 7.0f), b2Vec2(x + 30.0f, 7.0f));
			ground->CreateFixture(&fd);

			x += 30.0f;
			for (int32 i = 0; i < 5; ++i)
			{
				shape.Set(b2Vec2(x, 7.0f), b2Vec2(x, 4.0f));
				ground->CreateFixture(&fd);

				shape.Set(b2Vec2(x, 4.0f), b2Vec2(x + 1.0f, 4.0f));
				ground->CreateFixture(&fd);

				x += 1.0f;
				shape.Set(b2Vec2(x, 4.0f), b2Vec2(x, 7.0f));
				ground->CreateFixture(&fd);

				shape.Set(b2Vec2(x, 7.0f), b2Vec2(x + 1.0f, 7.0f));
				ground->CreateFixture(&fd);
				x += 1.0f;
			}

			shape.Set(b2Vec2(x, 7.0f), b2Vec2(x + 10.0f, 7.0f));
			ground->CreateFixture(&fd);

			x += 10.0f;
			shape.Set(b2Vec2(x, 7.0f), b2Vec2(x, 8.0f));
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
		
		//Boxes
		{
			float32 a = 0.2f;
			b2PolygonShape shape;
			shape.SetAsBox(a, a);

			float32 x = 55.5f;
			float32 y = -1.8f;

			for (int32 i = 0; i < 25; i++)
			{
				if (i != 0 && i % 5 == 0)
				{
					x += 1.0f; 
					y = -1.8f;
				}

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(x, y);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 1.0f);

				y += a * 2 + 0.01f;	
			}

			b2Vec2 x_1(268.0f, 3.2f);
			b2Vec2 y_1;
			b2Vec2 deltaX(0.225f, 0.45f);
			b2Vec2 deltaY(0.45f, 0.0f);

			for (int32 i = 0; i < 10; ++i)
			{
				y_1 = x_1; // reset y to start of the next level.

				for (int32 j = i; j < 10; ++j)
				{
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position = y_1;
					b2Body* body = m_world->CreateBody(&bd);
					body->CreateFixture(&shape, 1.0f);

					y_1 += deltaY; // Move to next horizontal position
				}

				x_1 += deltaX; // Move up a level and across half a block
			}

			b2Vec2 x_2(310.25f, 7.2f);
			
			for (int32 i = 0; i < 15; ++i)
			{
				y_1 = x_2; // reset y to start of the next level.

				for (int32 j = i; j < 20; ++j)
				{
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position = y_1;
					b2Body* body = m_world->CreateBody(&bd);
					body->CreateFixture(&shape, 1.0f);

					y_1 += deltaY; // Move to next horizontal position
				}

				x_2 += deltaX; // Move up a level and across half a block
			}
		}

		// Tank
		{
			b2PolygonShape chassis;
			b2Vec2 vertices[8];
			vertices[0].Set(0.47f - 3.305f, 0.46f);
			vertices[1].Set(5.46f - 3.305f, 0.46f);
			vertices[2].Set(5.78f - 3.305f, 0.73f);
			vertices[3].Set(5.59f - 3.305f, 1.06f);
			vertices[4].Set(4.52f - 3.305f, 1.69f);
			vertices[5].Set(0.89f - 3.305f, 1.69f);
			vertices[6].Set(0.17f - 3.305f, 0.95f);
			vertices[7].Set(0.17f - 3.305f, 0.69f);
			chassis.Set(vertices, 8);
			
			b2PolygonShape chassis_2;
			vertices[0].Set(0.89f - 3.305f, 1.69f);
			vertices[1].Set(4.52f - 3.305f, 1.69f);
			vertices[2].Set(4.62f - 3.305f, 1.83f);
			vertices[3].Set(1.13f - 3.305f, 1.83f);
			chassis_2.Set(vertices, 4);

			b2PolygonShape chassis_3;
			vertices[0].Set(0.89f - 3.305f, 1.69f);
			vertices[1].Set(0.99f - 3.305f, 1.75f);
			vertices[2].Set(0.95f - 3.305f, 1.81f);
			vertices[3].Set(0.89f - 3.305f, 1.81f);
			chassis_3.Set(vertices, 4);

			b2PolygonShape exhaust;
			vertices[0].Set(0.37f - 3.305f, 1.16f);
			vertices[1].Set(0.69f - 3.305f, 1.48f);
			vertices[2].Set(0.46f - 3.305f, 1.48f);
			vertices[3].Set(0.26f - 3.305f, 1.32f);
			exhaust.Set(vertices, 4);

			b2PolygonShape exhaust_2;
			vertices[0].Set(0.16f - 3.305f, 1.16f);
			vertices[1].Set(0.32f - 3.305f, 1.24f);
			vertices[2].Set(0.28f - 3.305f, 1.30f);
			vertices[3].Set(0.13f - 3.305f, 1.23f);
			exhaust_2.Set(vertices, 4);

			b2PolygonShape mudguard;
			vertices[0].Set(0.37f - 3.305f, 1.20f);
			vertices[1].Set(5.81f - 3.305f, 1.20f);
			vertices[2].Set(5.81f - 3.305f, 1.24f);
			vertices[3].Set(0.37f - 3.305f, 1.24f);
			mudguard.Set(vertices, 4);

			b2PolygonShape mudguard_2;
			vertices[0].Set(5.81f - 3.305f, 1.20f);
			vertices[1].Set(6.12f - 3.305f, 0.90f);
			vertices[2].Set(6.12f - 3.305f, 0.94f);
			vertices[3].Set(5.81f - 3.305f, 1.24f);
			mudguard_2.Set(vertices, 4);

			b2PolygonShape mudguard_3;
			vertices[0].Set(0.0f - 3.305f, 1.01f);
			vertices[1].Set(0.37f - 3.305f, 1.20f);
			vertices[2].Set(0.37f - 3.305f, 1.24f);
			vertices[3].Set(0.0f - 3.305f, 1.05f);
			mudguard_3.Set(vertices, 4);

			b2PolygonShape turret_small;
			vertices[0].Set(4.71f - 3.305f, 1.58f);
			vertices[1].Set(5.36f - 3.305f, 1.195f);
			vertices[2].Set(5.36f - 3.305f, 1.37f);
			vertices[3].Set(5.16f - 3.305f, 1.58f);
			turret_small.Set(vertices, 4);

			b2PolygonShape barrel_small;
			vertices[0].Set(5.36f - 3.305f, 1.27f);
			vertices[1].Set(5.46f - 3.305f, 1.27f);
			vertices[2].Set(5.46f - 3.305f, 1.35f);
			vertices[3].Set(5.36f - 3.305f, 1.35f);
			barrel_small.Set(vertices, 4);
			
			b2PolygonShape barrel_small_2;
			vertices[0].Set(5.46f - 3.305f, 1.30f);
			vertices[1].Set(5.59f - 3.305f, 1.30f);
			vertices[2].Set(5.59f - 3.305f, 1.32f);
			vertices[3].Set(5.46f - 3.305f, 1.32f);
			barrel_small_2.Set(vertices, 4);
			
			b2PolygonShape turret;
			vertices[0].Set(2.44f - 3.305f, 1.83f);
			vertices[1].Set(4.62f - 3.305f, 1.83f);
			vertices[2].Set(4.62f - 3.305f, 2.23f);
			vertices[3].Set(4.48f - 3.305f, 2.32f);
			vertices[4].Set(4.08f - 3.305f, 2.39f);
			vertices[5].Set(2.72f - 3.305f, 2.39f);
			turret.Set(vertices, 6);

			b2PolygonShape turret_2;
			vertices[0].Set(4.62f - 3.305f, 1.83f);
			vertices[1].Set(5.03f - 3.305f, 1.83f);
			vertices[2].Set(5.03f - 3.305f, 2.03f);
			vertices[3].Set(4.90f - 3.305f, 2.23f);
			vertices[4].Set(4.62f - 3.305f, 2.23f);
			turret_2.Set(vertices, 5);

			b2PolygonShape turret_3;
			vertices[0].Set(3.03f - 3.305f, 2.39f);
			vertices[1].Set(3.56f - 3.305f, 2.39f);
			vertices[2].Set(3.52f - 3.305f, 2.43f);
			vertices[3].Set(3.08f - 3.305f, 2.43f);
			turret_3.Set(vertices, 4);

			b2PolygonShape turret_4;
			vertices[0].Set(3.94f - 3.305f, 2.39f);
			vertices[1].Set(4.07f - 3.305f, 2.39f);
			vertices[2].Set(4.07f - 3.305f, 2.53f);
			vertices[3].Set(4.02f - 3.305f, 2.60f);
			vertices[4].Set(3.99f - 3.305f, 2.60f);
			vertices[5].Set(3.94f - 3.305f, 2.53f);
			turret_4.Set(vertices, 6);
			
			b2PolygonShape barrel;
			vertices[0].Set(4.8f - 3.305f, 2.06f);
			vertices[1].Set(6.61f - 3.305f, 2.06f);
			vertices[2].Set(6.61f - 3.305f, 2.20f);
			vertices[3].Set(4.8f - 3.305f, 2.20f);
			barrel.Set(vertices, 4);
			
			// track
			#define DEGTORAD 0.0174532925199432957f
			#define RADTODEG 57.295779513082320876f

			b2PolygonShape trackSegment;
			trackSegment.SetAsBox(0.08f, 0.05f);

			float t_x = 0.4f;
			float t_y = 1.0f;
			
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(t_x, t_y);
			t_x += 0.08f;

			b2FixtureDef fd;
			fd.shape = &trackSegment;
			fd.density = 1.0f;
			fd.friction = 1.0f;
			fd.filter.categoryBits = 0x0004;
			fd.filter.maskBits = 0xFFFF;
			
			b2Body* m_link = m_world->CreateBody( &bd ); //create first link
			m_link->CreateFixture( &fd );
			b2Body* m_link_coppy = m_link;

			b2RevoluteJointDef jdd;
			jdd.localAnchorA.Set( 0.04f, 0.0f);
			jdd.localAnchorB.Set(-0.04f, 0.0f);

			for (int i = 0; i < 46; i++) //top part of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(t_x, t_y);

				fd.shape = &trackSegment;
				fd.density = 1.0f;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				t_x += 0.08f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );

				m_link = m_track;//prepare for next iteration
			}

			float angle = 78.0f;
			float angle_2 = -14.5;
			float radius = 0.27f;
			float _x = 5.70f;
			float _y = 0.73f;

			for (int i = 0; i < 9; i++) //front wheel
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x+radius*cos(angle*DEGTORAD), _y+radius*sin(angle*DEGTORAD));

				fd.shape = &trackSegment;
				fd.density = 1.0f;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);
				
				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);
			
				angle_2 -= 15.5f;
				angle -= 16.0f;

				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );

				m_link = m_track;//prepare for next iteration
			}

			angle_2 += 5.5f;
			_x = 5.81f;
			_y = 0.48f;
			for (int i = 0; i < 11; i++) //from the front wheel to the bottom part of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x, _y);

				fd.shape = &trackSegment;
				fd.density = 1.0f;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);
				
				b2Vec2 pos = m_track->GetPosition();
				m_track->SetTransform(pos, angle_2*DEGTORAD);

				_x -= 0.067f;
				_y -= 0.04f;
				
				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );

				m_link = m_track;//prepare for next iteration
			}

			t_x = 4.988f;
			t_y = 0.045f;
			for (int i = 0; i < 31; i++) //bottom part of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(t_x, t_y);

				fd.shape = &trackSegment;
				fd.density = 1.0f;
				fd.friction = 1.0f;

				m_track = m_world->CreateBody(&bd);
				m_track->CreateFixture(&fd);

				t_x -= 0.08f;
				
				jdd.bodyA = m_link;
				jdd.bodyB = m_track;
				jdd.collideConnected = false;
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );

				m_link = m_track;//prepare for next iteration
			}
			
			angle_2 = 162.2f;
			_x = 1.15f;
			_y = 0.053f;
			for (int i = 0; i < 12; i++) //from the back wheel to bottom of the track
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x, _y);

				fd.shape = &trackSegment;
				fd.density = 1.0f;
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
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );

				m_link = m_track;//prepare for next iteration
			}

			angle = 243.35f;
			angle_2 = 156.35f;
			radius = 0.355f;
			_x = 0.4f;
			_y = 0.64f;

			for (int i = 0; i < 12; i++) //back wheel
			{
				bd.type = b2_dynamicBody;
				bd.position.Set(_x+radius*cos(angle*DEGTORAD), _y+radius*sin(angle*DEGTORAD));

				fd.shape = &trackSegment;
				fd.density = 1.0f;
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
				m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );

				m_link = m_track;//prepare for next iteration
			}

			jdd.bodyA = m_link;
			jdd.bodyB = m_link_coppy;
			jdd.collideConnected = false;
			m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &jdd );
			
			//end of the track

			b2CircleShape circle;
			circle.m_radius = 0.445f;

			b2CircleShape circle_1;
			circle_1.m_radius = 0.34f;

			b2CircleShape circle_2;
			circle_2.m_radius = 0.255f;

			bd.type = b2_dynamicBody;
			bd.position.Set(3.305f, 0.0f);

			fd.density = 1.0f;
			fd.filter.categoryBits = 0x0002;
			fd.filter.maskBits = 0xFFFF & ~0x0004;

			fd.shape = &chassis;
			m_car = m_world->CreateBody(&bd);
			m_car->CreateFixture(&fd);

			fd.shape = &chassis_2;
			m_car->CreateFixture(&fd);

			fd.shape = &chassis_3;
			m_car->CreateFixture(&fd);

			fd.shape = &turret_small;
			m_car->CreateFixture(&fd);

			fd.shape = &barrel_small;
			m_car->CreateFixture(&fd);

			fd.shape = &barrel_small_2;
			m_car->CreateFixture(&fd);

			fd.shape = &exhaust;
			m_car->CreateFixture(&fd);

			fd.shape = &exhaust_2;
			m_car->CreateFixture(&fd);

			fd.shape = &mudguard;
			m_car->CreateFixture(&fd);

			fd.shape = &mudguard_2;
			m_car->CreateFixture(&fd);

			fd.shape = &mudguard_3;
			m_car->CreateFixture(&fd);

			fd.shape = &turret;
			m_turret = m_world->CreateBody(&bd);
			m_turret->CreateFixture(&fd);

			fd.shape = &turret_2;
			m_turret->CreateFixture(&fd);

			fd.shape = &turret_3;
			m_turret->CreateFixture(&fd);

			fd.shape = &turret_4;
			m_turret->CreateFixture(&fd);

			fd.shape = &barrel;
			m_barrel = m_world->CreateBody(&bd);
			m_barrel->CreateFixture(&fd);
			
			b2DistanceJointDef jtd;
			jtd.collideConnected = true;
			jtd.length = 0.01f;
			jtd.Initialize( m_car, m_turret, b2Vec2(3.53f, 1.83f), b2Vec2(3.53f, 1.84f) );
			m_car_turret = (b2DistanceJoint*)m_world->CreateJoint(&jtd);
			
			b2RevoluteJointDef jtbd;
			jtbd.lowerAngle = -0.1f * b2_pi;
			jtbd.upperAngle = 0.1f * b2_pi;
			jtbd.enableLimit = true;
			jtbd.motorSpeed = 0.0f;
			jtbd.maxMotorTorque = 40.0f;
			jtbd.enableMotor = true;
			jtbd.Initialize( m_turret, m_barrel, b2Vec2(4.90f, 2.13f) );
			m_turretbarrel = (b2RevoluteJoint*)m_world->CreateJoint(&jtbd);
			
			fd.shape = &circle;
			fd.friction = 0.9f;
			fd.filter.categoryBits = 0x0004;
			fd.filter.maskBits = 0xFFFF;

			bd.position.Set(1.19f, 0.48f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&fd);
		
			bd.position.Set(2.10f, 0.48f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&fd);

			bd.position.Set(3.01f, 0.48f);
			m_wheel3 = m_world->CreateBody(&bd);
			m_wheel3->CreateFixture(&fd);

			bd.position.Set(4.01f, 0.48f);
			m_wheel4 = m_world->CreateBody(&bd);
			m_wheel4->CreateFixture(&fd);

			bd.position.Set(4.99f, 0.48f);
			m_wheel5 = m_world->CreateBody(&bd);
			m_wheel5->CreateFixture(&fd);
			
			fd.shape = &circle_1;
			bd.position.Set(0.4f, 0.64f);
			m_wheel_back = m_world->CreateBody(&bd);
			m_wheel_back->CreateFixture(&fd);
			
			fd.shape = &circle_2;
			bd.position.Set(5.70f, 0.73f);
			m_wheel_front = m_world->CreateBody(&bd);
			m_wheel_front->CreateFixture(&fd);
			
			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_car, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (b2WheelJoint*) m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel3, m_wheel3->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring3 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel4, m_wheel4->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring4 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel5, m_wheel5->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring5 = (b2WheelJoint*)m_world->CreateJoint(&jd);
			
			jd.Initialize(m_car, m_wheel_back, m_wheel_back->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = 100000;
			jd.dampingRatio = 0.0f;
			m_spring_back = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel_front, m_wheel_front->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = 100000;
			jd.dampingRatio = 0.0f;
			m_spring_front = (b2WheelJoint*)m_world->CreateJoint(&jd);
			
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a': // Reverse
			m_spring1->SetMotorSpeed(m_speed);
			m_spring2->SetMotorSpeed(m_speed);
			m_spring3->SetMotorSpeed(m_speed);
			m_spring4->SetMotorSpeed(m_speed);
			m_spring5->SetMotorSpeed(m_speed);
			m_spring_back->SetMotorSpeed(m_speed);
			m_spring_front->SetMotorSpeed(m_speed);
			break;

		case 's': // Stop
			m_spring1->SetMotorSpeed(0.0f);
			m_spring2->SetMotorSpeed(0.0f);
			m_spring3->SetMotorSpeed(0.0f);
			m_spring4->SetMotorSpeed(0.0f);
			m_spring5->SetMotorSpeed(0.0f);
			m_spring_back->SetMotorSpeed(0.0f);
			m_spring_front->SetMotorSpeed(0.0f);
			break;

		case 'd': // Forward
			m_spring1->SetMotorSpeed(-m_speed);
			m_spring2->SetMotorSpeed(-m_speed);
			m_spring3->SetMotorSpeed(-m_speed);
			m_spring4->SetMotorSpeed(-m_speed);
			m_spring5->SetMotorSpeed(-m_speed);
			m_spring_back->SetMotorSpeed(-m_speed);
			m_spring_front->SetMotorSpeed(-m_speed);
			break;

		case 'q': // Decrease natural freq.
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			m_spring3->SetSpringFrequencyHz(m_hz);
			m_spring4->SetSpringFrequencyHz(m_hz);
			m_spring5->SetSpringFrequencyHz(m_hz);
			break;

		case 'e': // Increase natural freq.
			m_hz += 1.0f;
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			m_spring3->SetSpringFrequencyHz(m_hz);
			m_spring4->SetSpringFrequencyHz(m_hz);
			m_spring5->SetSpringFrequencyHz(m_hz);
			break;

		case 'u': //raise barrel
			if(m_turretbarrel -> GetJointAngle() < 0.1f * b2_pi)
				m_turretbarrel->SetMotorSpeed(0.1f);
			break;

		case 'j': //lower barrel
			if(m_turretbarrel->GetJointAngle() > -0.1f * b2_pi)
				m_turretbarrel->SetMotorSpeed(-0.1f);
			break;

		case 'm':
			{
				m_turretbarrel->SetMotorSpeed( 0.0f );
             
				b2CircleShape shape;
				shape.m_radius = 0.09f;

				b2FixtureDef fsd;
				fsd.shape = &shape;
				fsd.density = 20.0f;
				fsd.restitution = 0.05f;

				b2Vec2 posn = m_turretbarrel->GetAnchorB();
				float32 angle = m_turretbarrel->GetJointAngle() + m_turret->GetAngle();
				
				posn.x += 1.5f * cos(angle);
				posn.y += 1.5f * sin(angle);

				b2BodyDef bsd;
				bsd.type = b2_dynamicBody;
				bsd.bullet = true;
				bsd.position.Set(posn.x, posn.y);

				m_shell = m_world->CreateBody(&bsd);
				m_shell->CreateFixture(&fsd);
				m_shell->SetLinearVelocity(b2Vec2(400.0f*cos(angle), 400.0f*sin(angle)));
				break;
			}
		}
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;


		//settings->viewCenter.x = 335.0f;
		settings->viewCenter.x = m_car->GetPosition().x;
		settings->viewCenter.y = m_car->GetPosition().y + 2;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new T34_85;
	}

	b2Body* m_car;
	b2Body* m_turret;
	b2Body* m_barrel;
	b2Body* m_shell;
	b2Body* m_wheel1;
	b2Body* m_wheel2;
	b2Body* m_wheel3;
	b2Body* m_wheel4;
	b2Body* m_wheel5;
	b2Body* m_wheel_back;
	b2Body* m_wheel_front;
	b2Body* m_track;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;

	b2WheelJoint* m_spring1;
	b2WheelJoint* m_spring2;
	b2WheelJoint* m_spring3;
	b2WheelJoint* m_spring4;
	b2WheelJoint* m_spring5;
	b2WheelJoint* m_spring_back;
	b2WheelJoint* m_spring_front;

	b2DistanceJoint* m_car_turret;

	b2RevoluteJoint* m_turretbarrel;
	b2RevoluteJoint* m_joint;
};

#endif
