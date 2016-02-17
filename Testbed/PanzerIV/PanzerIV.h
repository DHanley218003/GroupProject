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

#ifndef PANZERIV_H
#define PANZERIV_H

// This is a fun demo that shows off the wheel joint
class Panzer : public Test
{
public:
	Panzer()
	{		
		m_hz = 15.0f;
		m_zeta = 0.7f;
		m_speed = 7.0f;

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(b2Vec2(-20.0f, 0.38f), b2Vec2(20.0f, 0.25f));
			ground->CreateFixture(&fd);

			float32 hs[10] = {0.25f, 1.0f, 4.0f, 1.0f, 0.0f, -1.0f, -2.0f, -1.0f, -0.25f, 0.25f};

			float32 x = 20.0f, y1 = 0.25f, dx = 10.0f;

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

			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.25f));
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
/*
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
*/
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

		// Tank
		{
			b2Vec2 vertices[12];

			b2PolygonShape chassis;
			vertices[0].Set(-3.0f, -2.1f);
			vertices[1].Set(-3.25f, -2.7f);
			vertices[2].Set(-3.15f, -2.85f);
			vertices[3].Set(5.0f, -2.85f);
			vertices[4].Set(5.7f,-2.2f);
			vertices[5].Set(5.88f, -2.0f);
			vertices[6].Set(5.2f, -1.5f);
			vertices[7].Set(-2.0f,-1.5f);
			chassis.Set(vertices, 8);

			b2PolygonShape chassis1;
			vertices[0].Set(-2.75f, -2.85f);
			vertices[1].Set(-2.0f, -3.15f);
			vertices[2].Set(4.4f, -3.15f);
			vertices[3].Set(5.0f, -2.85f);
			chassis1.Set(vertices, 4);

			b2PolygonShape chassis2;
			vertices[0].Set(-3.1f, -1.5f);
			vertices[1].Set(-3.0f, -2.1f);
			vertices[2].Set(-2.0f, -1.5f);
			chassis2.Set(vertices, 3);

			b2PolygonShape chassis3;
			vertices[0].Set(-3.1f, -0.6f);
			vertices[1].Set(-3.1f, -1.4f);
			vertices[2].Set(4.88f, -1.4f);
			vertices[3].Set(4.6f, -0.9f);
			vertices[4].Set(2.84f, -0.6f);
			chassis3.Set(vertices, 5);

			b2PolygonShape trackSkirt;
			trackSkirt.SetAsBox(4.32f,0.05f,b2Vec2(1.12f,-1.45f),0.0f);

			b2PolygonShape trackSkirtRear;
			trackSkirtRear.SetAsBox(0.6f, 0.05f, b2Vec2(-3.68f, -1.73f), b2_pi/6.0f);

			b2PolygonShape trackSkirtLower;
			vertices[0].Set(-3.1f, -0.6f);
			vertices[1].Set(-3.1f, -1.4f);
			vertices[2].Set(4.88f, -1.4f);
			vertices[3].Set(4.6f, -0.9f);
			trackSkirtLower.Set( vertices, 4 );

			b2PolygonShape turretRing;
			turretRing.SetAsBox(1.92f,0.05f,b2Vec2(0.92f,-0.55f),0.0f);

			b2PolygonShape turret;
			vertices[0].Set(-2.0f, -0.5f);
			vertices[1].Set(2.7f, -0.5f);
			vertices[2].Set(2.55f, 0.5f);
			vertices[3].Set(1.2f, 0.65f);
			vertices[4].Set(-0.2f, 0.65f);
			vertices[5].Set(-1.7f, 0.6f);
			vertices[6].Set(-2.0f, 0.58f);
			turret.Set(vertices, 7);

			b2PolygonShape barrel1;
			barrel1.SetAsBox(0.75f,0.2f,b2Vec2(2.75f,0.1f),0.0f);

			b2PolygonShape barrel2;
			barrel2.SetAsBox(0.4f,0.15f,b2Vec2(3.9f,0.1f),0.0f);

			b2PolygonShape barrel3;
			vertices[0].Set(4.3f, 0.0f);
			vertices[1].Set(6.5f, 0.05f);
			vertices[2].Set(6.5f, 0.15f);
			vertices[3].Set(4.3f, 0.2f);
			barrel3.Set(vertices, 4);

			b2PolygonShape barrel4;
			vertices[0].Set(6.5f, 0.05f);
			vertices[1].Set(6.6f, 0.0f);
			vertices[2].Set(6.6f, 0.2f);
			vertices[3].Set(6.5f, 0.15f);
			barrel4.Set(vertices, 4);

			b2PolygonShape barrel5;
			barrel5.SetAsBox(0.1f,0.1f,b2Vec2(6.7f,0.1f),0.0f);

			b2PolygonShape cupola[4];
			cupola[0].SetAsBox(0.6f,0.075f,b2Vec2(0.0f,0.7f),0.0f);
			cupola[1].SetAsBox(0.55f,0.075f,b2Vec2(0.0f,0.85f),0.0f);
			cupola[2].SetAsBox(0.5f,0.075f,b2Vec2(0.0f,1.0f),0.0f);
			cupola[3].SetAsBox(0.6f,0.075f,b2Vec2(0.0f,1.15f),0.0f);

			b2FixtureDef fd;
			fd.density = 2.0f;
			fd.filter.categoryBits = 0x0002;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 4.0f);

			//chassis
			m_car = m_world->CreateBody(&bd);
			fd.shape = &chassis;
			m_car->CreateFixture(&fd);
			m_car1 = m_world->CreateBody(&bd);
			fd.shape = &chassis1;
			m_car1->CreateFixture(&fd);
			m_car2 = m_world->CreateBody(&bd);
			fd.shape = &chassis2;
			m_car2->CreateFixture(&fd);
			m_car3 = m_world->CreateBody(&bd);
			fd.shape = &chassis3;
			m_car3->CreateFixture(&fd);

			//Track skirt
			m_trackSkirt = m_world->CreateBody(&bd);
			fd.shape = &trackSkirt;
			m_trackSkirt->CreateFixture(&fd);
			m_trackSkirtRear = m_world->CreateBody(&bd);
			fd.shape = &trackSkirtRear;
			m_trackSkirtRear->CreateFixture(&fd);

			//Track skirt (lower to constrin the tracks)
//			m_trackSkirtFront[0] = m_world->CreateBody(&bd);
//			m_trackSkirtFront[0]->CreateFixture(&trackSkirtFront[0], 1.0f);

			//Turret and turret ring.
			m_turretRing = m_world->CreateBody(&bd);
			fd.shape = &turretRing;
			m_turretRing->CreateFixture(&fd);
			m_turret = m_world->CreateBody(&bd);
			fd.shape = &turret;
			m_turret->CreateFixture(&fd);

			//Cupola
			for(int32 i=0; i<4; ++i)
			{
				m_cupola[i] = m_world->CreateBody(&bd);
				fd.shape = &cupola[i];
				m_cupola[i]->CreateFixture(&fd);
			}

			//Barrel parts
			m_barrel[0] = m_world->CreateBody(&bd);
			fd.shape = &barrel1;
			m_barrel[0]->CreateFixture(&fd);

			m_barrel[1] = m_world->CreateBody(&bd);
			fd.shape = &barrel2;
			m_barrel[1]->CreateFixture(&fd);

			m_barrel[2] = m_world->CreateBody(&bd);
			fd.shape = &barrel3;
			m_barrel[2]->CreateFixture(&fd);

/*			m_barrel[3] = m_world->CreateBody(&bd);
			fd.shape = &barrel4;
			m_barrel[3]->CreateFixture(&fd);

			m_barrel[4] = m_world->CreateBody(&bd);
			fd.shape = &barrel5;
			m_barrel[4]->CreateFixture(&fd);
*/
			//Weld them all together using b2WeldJoints

			b2WeldJointDef jtd;
			jtd.collideConnected = true;
			jtd.Initialize( m_car, m_car1, m_car->GetWorldCenter() );
			m_chassis = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_car, m_car2, m_car->GetWorldCenter() );
			m_chassis1 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_car, m_trackSkirt, m_car->GetWorldCenter() );
			m_chassis2 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_car3, m_trackSkirt, m_car3->GetWorldCenter() );
			m_chassis3 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_car3, m_turretRing, m_car3->GetWorldCenter() );
			m_chassis4 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_turret, m_turretRing, m_turret->GetWorldCenter() );
			m_chassis5 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_turret, m_cupola[0], m_turret->GetWorldCenter() );
			m_chassis6 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_cupola[0], m_cupola[1], m_cupola[0]->GetWorldCenter() );
			m_cupola1to2 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_cupola[1], m_cupola[2], m_cupola[1]->GetWorldCenter() );
			m_cupola2to3 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_cupola[2], m_cupola[3], m_cupola[2]->GetWorldCenter() );
			m_cupola3to4 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_barrel[0], m_barrel[1], m_barrel[0]->GetWorldCenter() );
			m_bar1to2 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_barrel[1], m_barrel[2], m_barrel[1]->GetWorldCenter() );
			m_bar2to3 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

/*			jtd.Initialize( m_barrel[2], m_barrel[3], m_barrel[2]->GetWorldCenter() );
			m_bar3to4 = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			jtd.Initialize( m_barrel[3], m_barrel[4], m_barrel[2]->GetWorldCenter() );
			m_bar4to5 = (b2WeldJoint*)m_world->CreateJoint(&jtd);
*/
			jtd.Initialize( m_trackSkirt, m_trackSkirtRear, m_trackSkirt->GetWorldCenter() );
			m_rearSkirt = (b2WeldJoint*)m_world->CreateJoint(&jtd);

			//Create hinged joint for barrel to enable elevation and depression of barrel relative to turret

			b2RevoluteJointDef jtbd;
			jtbd.lowerAngle = -0.05f * b2_pi;
			jtbd.upperAngle = 0.1f * b2_pi;
			jtbd.enableLimit = true;
			jtbd.motorSpeed = 0.0f;
			jtbd.maxMotorTorque = 100.0f;
			jtbd.enableMotor = true;
			jtbd.Initialize( m_turret, m_barrel[0], b2Vec2(2.5f, 4.1f) );
			m_turretbarrel = (b2RevoluteJoint*)m_world->CreateJoint(&jtbd);

			// Wheels
			// The 8 lower road wheels

			b2CircleShape circle;
			circle.m_radius = 0.4f;

//			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 5.0f;
			fd.friction = 0.9f;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

			for(int32 i = 0; i < 8; ++i)
			{
				bd.position.Set(-2.0f + 0.9f * i, 0.8f);
				m_wheel[i] = m_world->CreateBody(&bd);
				m_wheel[i]->CreateFixture(&fd);
			}

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			for(int32 i=0; i<8; ++i)
			{
				jd.Initialize(m_car1, m_wheel[i], m_wheel[i]->GetPosition(), axis);
				jd.motorSpeed = 0.0f;
				jd.maxMotorTorque = 50.0f;
				jd.enableMotor = true;
				jd.frequencyHz = m_hz;
				jd.dampingRatio = m_zeta;
				m_spring[i] = (b2WheelJoint*)m_world->CreateJoint(&jd);
			}

			// The four upper guide wheels

			circle.m_radius = 0.2f;		// 0.5 times the radius of the 8 road wheels
			fd.shape = &circle;

			jtbd.motorSpeed = 0.0f;
			jtbd.maxMotorTorque = 5.0f;
			jtbd.enableMotor = true;

			for(int32 i=0; i<4; ++i)
			{
				if (i==0) 
					bd.position.Set(-1.65f, 2.02f);
				else
				{
					bd.position.Set(0.25f + 1.8f * (i-1), 2.1f);
				}
				m_upperWheel[i] = m_world->CreateBody(&bd);
				m_upperWheel[i]->CreateFixture(&fd);

				jtbd.Initialize( m_car, m_upperWheel[i], m_upperWheel[i]->GetPosition() );
				m_upperSpring[i] = (b2RevoluteJoint*)m_world->CreateJoint(&jtbd);
			}

			// The front drive wheel

			circle.m_radius = 0.6f;		// 1.5 times the radius of the 8 road wheels
			fd.shape = &circle;

			bd.position.Set(5.55f, 1.7f);
			m_frontWheel = m_world->CreateBody(&bd);
			m_frontWheel->CreateFixture(&fd);

			jd.Initialize(m_car, m_frontWheel, m_frontWheel->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 50.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_frontSpring = (b2WheelJoint*)m_world->CreateJoint(&jd);

			// The rear drive wheel

			circle.m_radius = 0.7f;		// 1.75 times the radius of the 8 road wheels
			fd.shape = &circle;

			bd.position.Set(-3.15f, 1.3f);
			m_backWheel = m_world->CreateBody(&bd);
			m_backWheel->CreateFixture(&fd);

			jd.Initialize(m_car, m_backWheel, m_backWheel->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 50.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_backSpring = (b2WheelJoint*)m_world->CreateJoint(&jd);

			//Tracks
			 
			int32 N = 45;
//			int32 N = 60;
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 0.02f);

			fd.shape = &shape;
			fd.density = 0.8f;
			fd.friction = 0.5f;

			b2RevoluteJointDef jtrd;
			jtrd.motorSpeed = 0.0f;

			//Top: Front to Back
			bd.position.Set(5.48f, 2.311f);
			b2Body* prevBody = m_world->CreateBody(&bd);
			prevBody->CreateFixture(&fd);
			b2Body* firstBody = prevBody;

			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set(5.48f - 0.16f * i, 2.311f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x -=0.09f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}
			
			N = 7;
			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set(-1.7f - 0.16f * i, 2.311f - 0.034f * i);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x -= 0.09f;
				anchor.y -= 0.017f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}
			
			// Around Rear wheel: PrevBody at (-3.1,2.0464)
			N = 13;			
			for (int32 i = 0; i <= N; ++i)
			{
				float32 px = 0.05f;
				float32 py = 0.7464f;
				float32 theta = b2_pi*i/13.0f;

				shape.SetAsBox(0.1f, 0.02f, b2Vec2(0.0f,0.0f), theta);
				fd.shape = &shape;

				bd.position.Set(px*cos(theta)-py*sin(theta)-3.1f,py*cos(theta)+px*sin(theta)+1.3064f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}

			shape.SetAsBox(0.1f, 0.02f);
			fd.shape = &shape;

			//Down to ground and along to front and up to front wheel.
			N = 7;
			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set( prevBody->GetPosition().x + 0.16f, prevBody->GetPosition().y - 0.034f );
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x += 0.18f;
				anchor.y -= 0.017f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}

			N=40;
			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set(prevBody->GetPosition().x + 0.16f, prevBody->GetPosition().y);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x +=0.09f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}

			N = 7;
			shape.SetAsBox(0.1f, 0.02f, b2Vec2(0.0f,0.0f), b2_pi/6.0f);
			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set( prevBody->GetPosition().x + 0.18f*0.866f, prevBody->GetPosition().y + 0.18f*0.5f );
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x += 0.18f*0.866f;
				anchor.y += 0.18f*0.5f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}

			// Around Front Drive Wheel at (5.55, 1.7)
			N = 10;			
			for (int32 i = N; i > 0; --i)
			{
				float32 px = 0.05f;
				float32 py = 0.6464f;
				float32 theta = b2_pi*i/10.0f;

				shape.SetAsBox(0.1f, 0.02f, b2Vec2(0.0f,0.0f), -theta);
				fd.shape = &shape;

				bd.position.Set(px*cos(theta)+py*sin(theta)+5.55f,py*cos(theta)-px*sin(theta)+1.7f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}

			shape.SetAsBox(0.1f, 0.02f);
			fd.shape = &shape;

			b2Vec2 anchor(prevBody->GetPosition());
			anchor.x +=0.09f;
			jtrd.Initialize(prevBody, firstBody, anchor); 
			m_world->CreateJoint(&jtrd);
		}

		m_shell = NULL;
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			{
			for(int32 i=0; i<8; ++i)
				m_spring[i]->SetMotorSpeed(m_speed);
			m_frontSpring->SetMotorSpeed(m_speed/1.5f);
			m_backSpring->SetMotorSpeed(m_speed/1.75f);
			}
			break;

		case 's':
			{
			for(int32 i=0; i<8; ++i)
				m_spring[i]->SetMotorSpeed(0.0f);
			m_frontSpring->SetMotorSpeed(0.0f);
			m_backSpring->SetMotorSpeed(0.0f);
			}
			break;

		case 'd':
			{
			for(int32 i=0; i<8; ++i)
				m_spring[i]->SetMotorSpeed(-m_speed);
			m_frontSpring->SetMotorSpeed(-m_speed/1.5f);
			m_backSpring->SetMotorSpeed(-m_speed/1.75f);
			}
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			for(int32 i=0; i<8; ++i)
				m_spring[i]->SetSpringFrequencyHz(m_hz);
			break;

		case 'e':
			m_hz += 1.0f;
			for(int32 i=0; i<8; ++i)
				m_spring[i]->SetSpringFrequencyHz(m_hz);
			break;

		case 'u':
			if( m_turretbarrel->GetJointAngle() < (0.1f * b2_pi) )
				m_turretbarrel->SetMotorSpeed( 1.0f );
			break;

		case 'j':
			if( m_turretbarrel->GetJointAngle() > -0.1f * b2_pi )
				m_turretbarrel->SetMotorSpeed( -1.0f );
			break;

		case 'm':
			{
				m_turretbarrel->SetMotorSpeed( 0.0f );

				if (m_shell != NULL)
				{
					m_world->DestroyBody(m_shell);
					m_shell = NULL;
				}

				b2CircleShape shape;
				shape.m_radius = 0.09f;

				b2FixtureDef fsd;
				fsd.shape = &shape;
				fsd.density = 20.0f;
				fsd.restitution = 0.05f;

				b2Vec2 posn = m_turretbarrel->GetAnchorB();
				float32 angle = m_turretbarrel->GetJointAngle()+m_turret->GetAngle();

				posn.x += 4.3f * cos(angle);
				posn.y += 4.3f * sin(angle);

				b2BodyDef bsd;
				bsd.type = b2_dynamicBody;
				bsd.bullet = true;
				bsd.position.Set(posn.x, posn.y);

				m_shell = m_world->CreateBody(&bsd);
				m_shell->CreateFixture(&fsd);

				m_shell->SetLinearVelocity(b2Vec2(200.0f*cos(angle), 200.0f*sin(angle)));
				
			}
			break;
		}
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "barrel up = u, barrel down = j, barrel stop and fire = m");
		m_textLine += 15;

		settings->viewCenter.x = m_car->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Panzer;
	}

	b2Body* m_car;
	b2Body* m_car1;
	b2Body* m_car2;
	b2Body* m_car3;

	b2Body* m_trackSkirt;
	b2Body* m_trackSkirtRear;
	b2Body* m_trackSkirtFront[2];

	b2Body* m_turretRing;
	b2Body* m_turret;

	b2Body* m_barrel[5];

	b2Body* m_cupola[4];
	
	b2Body* m_shell;

	b2Body* m_wheel[8];
	b2Body* m_upperWheel[4];
	b2Body* m_frontWheel;
	b2Body* m_backWheel;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;

	b2WheelJoint* m_spring[8];
	b2WheelJoint* m_frontSpring;
	b2WheelJoint* m_backSpring;

	b2WeldJoint* m_chassis;
	b2WeldJoint* m_chassis1;
	b2WeldJoint* m_chassis2;
	b2WeldJoint* m_chassis3;
	b2WeldJoint* m_chassis4;
	b2WeldJoint* m_chassis5;
	b2WeldJoint* m_chassis6;
	b2WeldJoint* m_chassis7;
	b2WeldJoint* m_cupola1to2;
	b2WeldJoint* m_cupola2to3;
	b2WeldJoint* m_cupola3to4;
	b2WeldJoint* m_bar1to2;
	b2WeldJoint* m_bar2to3;
	b2WeldJoint* m_bar3to4;
	b2WeldJoint* m_bar4to5;
	b2WeldJoint* m_rearSkirt;

	b2RevoluteJoint* m_turretbarrel;

	b2RevoluteJoint* m_upperSpring[4];
//	b2RevoluteJoint* m_frontSpring;
//	b2RevoluteJoint* m_backSpring;
};

#endif
