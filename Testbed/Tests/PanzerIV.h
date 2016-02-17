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
		m_hz = 10.0f;
		m_zeta = 0.9f;
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

			shape.Set(b2Vec2(-40.0f,0.38f), b2Vec2(-40.0f, 20.0f));
			ground->CreateFixture(&fd);

			shape.Set(b2Vec2(-40.0f, 0.38f), b2Vec2(20.0f, 0.25f));
			ground->CreateFixture(&fd);

			float32 hx[13] = {4.0f, 4.8f, 5.3f, 7.0f, 7.5f, 9.0f, 11.0f, 12.1f, 12.5f, 13.0f, 15.0f, 15.8f, 18.0f};
			float32 hy[13] = {0.0f, 1.0f, 1.2f, 0.5f, 0.1f, 0.0f, 0.4f, 0.2f, 0.1f, 0.4f, 0.8f, 0.2f, 0.0f};

			float32 x1 = 20.0f, y1 = 0.25f;

			for (int32 i = 0; i < 13; ++i)
			{
				float32 x2 = hx[i];
				float32 y2 = hy[i];
				shape.Set(b2Vec2(x1, y1), b2Vec2(x1+x2, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x1 += x2;
			}

			for (int32 i = 0; i < 13; ++i)
			{
				float32 x2 = hx[i];
				float32 y2 = hy[i];
				shape.Set(b2Vec2(x1, y1), b2Vec2(x1+x2, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x1 += x2;
			}
			
			shape.Set(b2Vec2(x1, 0.0f), b2Vec2(x1 + 40.0f, 0.25f));
			ground->CreateFixture(&fd);

			x1 += 40.0f;
			shape.Set(b2Vec2(x1, 0.0f), b2Vec2(x1 + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x1 += 40.0f;
			shape.Set(b2Vec2(x1, 0.0f), b2Vec2(x1 + 10.0f, 5.0f));
			ground->CreateFixture(&fd);

			x1 += 10.0f;
			shape.Set(b2Vec2(x1, 5.0f), b2Vec2(x1 + 20.0f, 0.0f));
			ground->CreateFixture(&fd);

			x1 += 20.0f;
			shape.Set(b2Vec2(x1, 0.0f), b2Vec2(x1, 20.0f));
			ground->CreateFixture(&fd);
		}

		// Ruin
		{
			int32 N = 4;
			b2PolygonShape brick, halfBrick;
			brick.SetAsBox(0.5f, 0.125f);
			halfBrick.SetAsBox(0.25f, 0.125f);

			b2Body* body = NULL;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			b2Vec2 x(50.0f, 0.2f);
			b2Vec2 y;
			b2Vec2 deltaX(0.5625f, 0.3f);
			b2Vec2 deltaY(1.025f, 0.0f);

			for (int32 i = 0; i < N; ++i)
			{
				y = x;

				for (int32 j = i; j < N; ++j)
				{
					bd.position = y;
					body = m_world->CreateBody(&bd);
					body->CreateFixture(&brick, 1.0f);

					y += deltaY;
				}

				x += deltaX;
			}

			x = b2Vec2(49.75f, 0.8f);
			deltaX = b2Vec2( 0.25f, 0.3f );
			deltaY = b2Vec2( -0.25f, 0.3f );
			bd.position = x;

			for ( int32 i = 0; i < N; ++i )
			{	
				body = m_world->CreateBody(&bd);
				body->CreateFixture(&halfBrick, 1.0f);
				bd.position += deltaX;
				body = m_world->CreateBody(&bd);
				body->CreateFixture(&brick, 1.0f);
				bd.position += deltaY;
			}

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

		// Tank
		{
			b2Vec2 vertices[12];
			b2FixtureDef fd;
			fd.density = 1.0f;
			fd.filter.categoryBits = 0x0002;

			b2PolygonShape chassis[4];

			vertices[0].Set(-3.0f, -2.1f);
			vertices[1].Set(-3.25f, -2.7f);
			vertices[2].Set(-3.15f, -2.85f);
			vertices[3].Set(5.0f, -2.85f);
			vertices[4].Set(5.7f,-2.2f);
			vertices[5].Set(5.88f, -2.0f);
			vertices[6].Set(5.2f, -1.5f);
			vertices[7].Set(-2.0f,-1.5f);
			chassis[0].Set(vertices, 8);

			vertices[0].Set(-2.75f, -2.85f);
			vertices[1].Set(-2.0f, -3.15f);
			vertices[2].Set(4.4f, -3.15f);
			vertices[3].Set(5.0f, -2.85f);
			chassis[1].Set(vertices, 4);

			vertices[0].Set(-3.1f, -1.5f);
			vertices[1].Set(-3.0f, -2.1f);
			vertices[2].Set(-2.0f, -1.5f);
			chassis[2].Set(vertices, 3);

			vertices[0].Set(-3.1f, -0.6f);
			vertices[1].Set(-3.1f, -1.4f);
			vertices[2].Set(4.88f, -1.4f);
			vertices[3].Set(4.6f, -0.9f);
			vertices[4].Set(2.84f, -0.6f);
			chassis[3].Set(vertices, 5);

			b2PolygonShape trackSkirt[3];
			trackSkirt[0].SetAsBox(4.32f,0.05f,b2Vec2(1.12f,-1.45f),0.0f);
			trackSkirt[1].SetAsBox(0.6f, 0.05f, b2Vec2(-3.68f, -1.73f), b2_pi/6.0f);

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

			b2PolygonShape barrel[3];
			barrel[0].SetAsBox(0.75f,0.2f,b2Vec2(2.75f,0.1f),0.0f);
			barrel[1].SetAsBox(0.4f,0.15f,b2Vec2(3.9f,0.1f),0.0f);
			vertices[0].Set(4.3f, 0.0f);
			vertices[1].Set(6.5f, 0.05f);
			vertices[2].Set(6.5f, 0.15f);
			vertices[3].Set(4.3f, 0.2f);
			barrel[2].Set(vertices, 4);


			b2PolygonShape cupola[4];
			cupola[0].SetAsBox(0.6f,0.075f,b2Vec2(0.0f,0.7f),0.0f);
			cupola[1].SetAsBox(0.55f,0.075f,b2Vec2(0.0f,0.85f),0.0f);
			cupola[2].SetAsBox(0.5f,0.075f,b2Vec2(0.0f,1.0f),0.0f);
			cupola[3].SetAsBox(0.6f,0.075f,b2Vec2(0.0f,1.15f),0.0f);


			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 4.0f);

			//chassis
			for(int32 i=0; i<4; ++i)
			{
				m_carriage[i] = m_world->CreateBody(&bd);
				fd.shape = &chassis[i];
				m_carriage[i]->CreateFixture(&fd);
			}

			//Track skirt
			for(  int32 i=0; i<2; ++i )
			{
				m_trackSkirt[i] = m_world->CreateBody(&bd);
				fd.shape = &trackSkirt[i];
				m_trackSkirt[i]->CreateFixture(&fd);
			}

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
			for( int32 i=0; i<3; ++i)
			{
				m_barrel[i] = m_world->CreateBody(&bd);
				fd.shape = &barrel[i];
				m_barrel[i]->CreateFixture(&fd);
			}

			//Weld them all together using b2WeldJoints

			b2WeldJointDef jtd;
			jtd.collideConnected = true;

			b2Body* m_body_A = m_carriage[0];
			b2Body* m_body_B;
			
			for(int32 i=0; i<8; ++i)
			{
				switch (i)
				{
					case 0:
						m_body_B = m_carriage[1];
						break;
					case 1:
						m_body_B = m_carriage[2];
						break;
					case 2:
						m_body_B = m_trackSkirt[0];
						break;
					case 3:
						m_body_A = m_carriage[3];
						break;
					case 4:
						m_body_A = m_trackSkirt[1];
						break;
					case 5:
						m_body_A = m_carriage[3];
						m_body_B = m_turretRing;
						break;
					case 6:
						m_body_A = m_turret;
						break;
					case 7:
						m_body_B = m_cupola[0];
						break;
				}

				jtd.Initialize( m_body_A, m_body_B, m_body_A->GetWorldCenter() );
				m_chassis[i] = (b2WeldJoint*)m_world->CreateJoint(&jtd);
			}

			for(int32 i=0; i<3; ++i)
			{
				jtd.Initialize( m_cupola[i], m_cupola[i+1], m_cupola[i]->GetWorldCenter() );
				m_cup[i] = (b2WeldJoint*)m_world->CreateJoint(&jtd);
			}

			for(int32 i=0; i<2; ++i)
			{
				jtd.Initialize( m_barrel[i], m_barrel[i+1], m_barrel[i]->GetWorldCenter() );
				m_bar[i] = (b2WeldJoint*)m_world->CreateJoint(&jtd);
			}

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
			fd.density = 1.0f;
			fd.friction = 0.9f;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

			for(int32 i = 0; i < 8; ++i)
			{
				bd.position.Set(-2.0f + 0.9f * i, 0.5f);
				m_wheel[i] = m_world->CreateBody(&bd);
				m_wheel[i]->CreateFixture(&fd);
			}

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			for(int32 i=0; i<8; ++i)
			{
				jd.Initialize(m_carriage[1], m_wheel[i], m_wheel[i]->GetPosition(), axis);
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
			fd.friction = 0.8f;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;  // Set categoryBits 0x0002 to be collision category.

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

				jtbd.Initialize( m_carriage[0], m_upperWheel[i], m_upperWheel[i]->GetPosition() );
				m_upperSpring[i] = (b2RevoluteJoint*)m_world->CreateJoint(&jtbd);
			}

			// The front drive wheel

			circle.m_radius = 0.6f;		// 1.5 times the radius of the 8 road wheels
			fd.shape = &circle;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;
			fd.friction = 0.8f;

			bd.position.Set(5.55f, 1.7f);
			m_frontWheel = m_world->CreateBody(&bd);
			m_frontWheel->CreateFixture(&fd);

			//axis.x = 1.0f;
			//axis.y = 0.0f;

			jd.Initialize(m_carriage[0], m_frontWheel, m_frontWheel->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 50.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_frontSpring = (b2WheelJoint*)m_world->CreateJoint(&jd);

			// The rear drive wheel

			circle.m_radius = 0.7f;		// 1.75 times the radius of the 8 road wheels
			fd.shape = &circle;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

			bd.position.Set(-3.15f, 1.3f);
			m_backWheel = m_world->CreateBody(&bd);
			m_backWheel->CreateFixture(&fd);

			jd.Initialize(m_carriage[0], m_backWheel, m_backWheel->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 50.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_backSpring = (b2WheelJoint*)m_world->CreateJoint(&jd);

			//Tracks
			 
			int32 N = 46;
//			int32 N = 60;
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 0.02f);

			fd.shape = &shape;
			fd.density = 10.0f;
			fd.friction = 0.8f;
			fd.filter.categoryBits = 0x0001;

			b2RevoluteJointDef jtrd;
			jtrd.localAnchorA = b2Vec2(-0.09f,0.0f);
			jtrd.localAnchorA = b2Vec2(0.09f,0.0f);
			jtrd.collideConnected = false;

			//Top: Front to Back
			bd.position.Set(5.60f, 2.311f);
			b2Body* prevBody = m_world->CreateBody(&bd);
			prevBody->CreateFixture(&fd);
			b2Body* firstBody = prevBody;

			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set(5.60f - 0.16f * i, 2.311f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x -=0.09f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}

			N = 6;
			shape.SetAsBox(0.1f, 0.02f, b2Vec2(0.0f,0.0f), b2_pi*1.07f/18.0f);
			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set( prevBody->GetPosition().x - 0.18f*0.983f, prevBody->GetPosition().y - 0.18f*0.186f );
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x -= 0.18f*0.983f;
				anchor.y -= 0.18f*0.186f;
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

			
			N = 6;
			shape.SetAsBox(0.1f, 0.02f, b2Vec2(0.0f,0.0f), -b2_pi*1.20f/18.0f);
			for (int32 i = 1; i <= N; ++i)
			{
				bd.position.Set( prevBody->GetPosition().x + 0.18f*0.978f, prevBody->GetPosition().y - 0.18f*0.208f );
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(prevBody->GetPosition());
				anchor.x += 0.18f*0.978f;
				anchor.y -= 0.18f*0.208f;
				jtrd.Initialize(prevBody, body, anchor); 
				m_world->CreateJoint(&jtrd);

				prevBody = body;
			}
			
			shape.SetAsBox(0.1f, 0.02f, b2Vec2(0.0f, 0.0f), 0.0f);
			fd.shape = &shape;

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

			N = 8;
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
			N = 9;			
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
			anchor.x -=0.09f;
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

				posn.x += 4.0f * cos(angle);
				posn.y += 4.0f * sin(angle);

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

		settings->viewCenter.x = m_carriage[0]->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Panzer;
	}

	b2Body* m_carriage[4];
	b2Body* m_trackSkirt[3];
	b2Body* m_turretRing;
	b2Body* m_turret;
	b2Body* m_barrel[3];
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

	b2WeldJoint* m_chassis[8];
	b2WeldJoint* m_cup[3];
	b2WeldJoint* m_bar[2];

	b2RevoluteJoint* m_turretbarrel;
	b2RevoluteJoint* m_upperSpring[4];
};

#endif
