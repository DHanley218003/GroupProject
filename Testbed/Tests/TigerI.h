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

#ifndef TIGERI_H
#define TIGERI_H

#include<cmath>

// This is a fun demo that drives a TigerI
class TigerI : public Test
{
public:
	TigerI()
	{
		m_hz = 8.0f;
		m_zeta = 0.7f;
		//m_speed = 50.0f;
		m_speed = 0.0f;

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

			float32 hs[10] = {0.25f, 1.0f, 2.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

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
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 10.0f, 2.0f));
			ground->CreateFixture(&fd);

			x += 10.0f;
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

		// TIGER I
		{
			//  -------------------------------------------
			//                  MAIN CHASSIS
			//  -------------------------------------------
			b2PolygonShape main_chassis;
			b2Vec2 verts[8];

            verts[0].Set(2.149f, 0.0f);
            verts[1].Set(-2.757f, 0.0f);
            verts[2].Set(-2.642, -0.867f);
            verts[3].Set(-2.602, -1.167f);
            verts[4].Set(-2.484f, -1.405f);
            verts[5].Set(2.319f, -1.405f);
            verts[6].Set(2.428f, -1.366f);
			main_chassis.Set(verts, 7);
			//  -------------------------------------------
			//                   FRONT CHASSIS
			//  -------------------------------------------
			b2PolygonShape front_chassis;

            verts[0].Set(3.116f, -0.543f);
            verts[1].Set(2.224f, -0.368f);
            verts[2].Set(2.428f, -1.366f);
            verts[3].Set(2.811f, -1.229f);

			front_chassis.Set(verts, 4);


			//  -------------------------------------------
			//                 EXHAUST ARMOUR
			//  -------------------------------------------
			b2PolygonShape exhaust;

            verts[0].Set(-2.787f, 0.231f);
            verts[1].Set(-3.162f, 0.181f);
            verts[2].Set(-3.016f, -0.917f);
            verts[3].Set(-2.642f, -0.867f);

			exhaust.Set(verts, 4);

			//  -------------------------------------------
			//                  FRONT PLATE
			//  -------------------------------------------
			b2PolygonShape fPlate;

            verts[0].Set(2.221f, 0.041f);
            verts[1].Set(2.144f, 0.025f);
            verts[2].Set(2.224f, -0.368f);
            verts[3].Set(2.308f, -0.385f);

			fPlate.Set(verts, 4);

			//  -------------------------------------------
			//                  MG34 BOW
			//  -------------------------------------------
			b2PolygonShape MG34bow;

            verts[0].Set(2.376f, -0.107f);
            verts[1].Set(2.315f, -0.057f);
            verts[2].Set(2.238f, -0.039f);
            verts[3].Set(2.306f, -0.375f);
            verts[4].Set(2.370f, -0.328f);
            verts[5].Set(2.407f, -0.258f);
            verts[6].Set(2.409f, -0.179f);

			MG34bow.Set(verts, 7);
			//  -------------------------------------------
			//                  MG34
			//  -------------------------------------------
			b2PolygonShape MG34;
            /*
            verts[0].Set(2.703f, -0.155f);
            verts[1].Set(2.697f, -0.116f);
            verts[2].Set(2.373f, -0.168f);
            verts[3].Set(2.380f, -0.207f);*/
            verts[0].Set(2.704f, -0.164f);
            verts[1].Set(2.376f, -0.164f);
            verts[2].Set(2.376f, -0.204f);
            verts[3].Set(2.704f, -0.204f);

			MG34.Set(verts, 4);

            //  -------------------------------------------
			//                SIDE MUD GUARD
			//  -------------------------------------------
			b2PolygonShape mud_side;

            verts[0].Set(2.326f, -0.473f);
            verts[1].Set(2.278f, -0.237f);
            verts[2].Set(-2.694f, -0.471f);
            verts[3].Set(-2.663f, -0.708f);

			mud_side.Set(verts, 4);

			//  -------------------------------------------
			//                REAR MUD GUARD
			//  -------------------------------------------
			b2PolygonShape mud_rear;

            verts[0].Set(-2.678f, -0.597f);
            verts[1].Set(-3.018f, -0.968f);
            verts[2].Set(-2.991f, -0.993f);
            verts[3].Set(-2.671f, -0.645f);

			mud_rear.Set(verts, 4);
			//  -------------------------------------------
			//                  MG34 (counterBalance)
			//  -------------------------------------------
			b2PolygonShape MG34counterBalance;
            verts[0].Set(2.376f, -0.164f);
            verts[1].Set(2.356f, -0.164f);
            verts[2].Set(2.356f, -0.204f);
            verts[3].Set(2.376f, -0.204f);

			MG34counterBalance.Set(verts, 4);
			//  -------------------------------------------
			//                  TURRET BASE JOINT
			//  -------------------------------------------
			b2PolygonShape turretBase;
            verts[0].Set(1.149f, 0.116f);
            verts[1].Set(-1.153f, 0.116f);
            verts[2].Set(-1.153f, 0.0f);
            verts[3].Set(1.149f, 0.0f);

			turretBase.Set(verts, 4);
			//  -------------------------------------------
			//                  TURRET MAIN
			//  -------------------------------------------
			b2PolygonShape turretMain;
            verts[0].Set(1.196f, 0.688f);
            verts[1].Set(0.281f, 0.812f);
            verts[2].Set(-1.153f, 0.812f);
            verts[3].Set(-1.153f, 0.116f);
            verts[4].Set(1.247f, 0.116f);
            verts[5].Set(1.306f, 0.408f);

			turretMain.Set(verts, 6);

			b2PolygonShape turretHatch_0;
            verts[0].Set(0.021f, 0.974f);
            verts[1].Set(-0.010f, 1.005f);
            verts[2].Set(-0.832f, 1.005f);
            verts[3].Set(-0.862f, 0.974f);
            verts[4].Set(-0.862f, 0.812f);
            verts[5].Set(0.021f, 0.812f);

			turretHatch_0.Set(verts, 6);

			b2PolygonShape turretHatch_1;
            verts[0].Set(-0.010f, 1.081f);
            verts[1].Set(-0.832f, 1.081f);
            verts[2].Set(-0.832f, 1.005f);
            verts[3].Set(-0.010f, 1.005f);

			turretHatch_1.Set(verts, 4);

			b2PolygonShape turretRearBox;
            verts[0].Set(-0.990f, 0.812f);
            verts[1].Set(-1.548f, 0.783f);
            verts[2].Set(-1.548f, 0.167f);
            verts[3].Set(-0.990f, 0.167f);

			turretRearBox.Set(verts, 4);

			b2PolygonShape turretMantle;
            verts[0].Set(1.365f, 0.661f);
            verts[1].Set(1.315f, 0.661f);
            verts[2].Set(1.100f, 0.444f);
            verts[3].Set(1.081f, 0.416f);
            verts[4].Set(1.081f, 0.352f);
            verts[5].Set(1.104f, 0.320f);
            verts[6].Set(1.315f, 0.134f);
            verts[7].Set(1.365f, 0.134f);

			turretMantle.Set(verts, 8);

			b2PolygonShape canon_00;
            verts[0].Set(1.498f, 0.626f);
            verts[1].Set(1.365f, 0.661f);
            verts[2].Set(1.365f, 0.134f);
            verts[3].Set(1.498f, 0.187f);

			canon_00.Set(verts, 4);

			b2PolygonShape canon_01;
            verts[0].Set(1.572f, 0.579f);
            verts[1].Set(1.556f, 0.590f);
            verts[2].Set(1.498f, 0.590f);
            verts[3].Set(1.498f, 0.213f);
            verts[4].Set(1.556f, 0.213f);
            verts[5].Set(1.572f, 0.224f);

			canon_01.Set(verts, 6);

			b2PolygonShape canon_02;
            verts[0].Set(1.648f, 0.561f);
            verts[1].Set(1.630f, 0.579f);
            verts[2].Set(1.572f, 0.579f);
            verts[3].Set(1.572f, 0.224f);
            verts[4].Set(1.630f, 0.224f);
            verts[5].Set(1.648f, 0.242f);

			canon_02.Set(verts, 6);

			b2PolygonShape canon_03;
            verts[0].Set(2.349f, 0.528f);
            verts[1].Set(2.310f, 0.561f);
            verts[2].Set(1.648f, 0.561f);
            verts[3].Set(1.648f, 0.242f);
            verts[4].Set(2.310f, 0.242f);
            verts[5].Set(2.349f, 0.275f);

			canon_03.Set(verts, 6);

			b2PolygonShape canon_04;
            verts[0].Set(3.169f, 0.497f);
            verts[1].Set(3.149f, 0.517f);
            verts[2].Set(2.349f, 0.517f);
            verts[3].Set(2.349f, 0.286f);
            verts[4].Set(3.149f, 0.286f);
            verts[5].Set(3.169f, 0.306f);

			canon_04.Set(verts, 6);

			b2PolygonShape canon_05;
            verts[0].Set(4.770f, 0.481f);
            verts[1].Set(3.169f, 0.481f);
            verts[2].Set(3.169f, 0.322f);
            verts[3].Set(4.770f, 0.322f);

			canon_05.Set(verts, 4);

			b2PolygonShape canon_06;
            verts[0].Set(4.839f, 0.501f);
            verts[1].Set(4.770f, 0.481f);
            verts[2].Set(4.770f, 0.322f);
            verts[3].Set(4.839f, 0.301f);

			canon_06.Set(verts, 4);

			b2PolygonShape canon_07;
            verts[0].Set(4.987f, 0.522f);
            verts[1].Set(4.905f, 0.522f);
            verts[2].Set(4.839f, 0.501f);
            verts[3].Set(4.839f, 0.463f);
            verts[4].Set(4.987f, 0.463f);

			canon_07.Set(verts, 5);

			b2PolygonShape canon_08;
            verts[0].Set(4.987f, 0.340f);
            verts[1].Set(4.839f, 0.340f);
            verts[2].Set(4.839f, 0.301f);
            verts[3].Set(4.905f, 0.281f);
            verts[4].Set(4.987f, 0.281f);

			canon_08.Set(verts, 5);

			b2PolygonShape canon_09;
            verts[0].Set(5.031f, 0.522f);
            verts[1].Set(4.987f, 0.522f);
            verts[2].Set(4.987f, 0.281f);
            verts[3].Set(5.031f, 0.281f);

			canon_09.Set(verts, 4);

			b2PolygonShape canon_10;
            verts[0].Set(5.199f, 0.522f);
            verts[1].Set(5.031f, 0.522f);
            verts[2].Set(5.031f, 0.463f);
            verts[3].Set(5.199f, 0.463f);

			canon_10.Set(verts, 4);

			b2PolygonShape canon_11;
            verts[0].Set(5.199f, 0.340f);
            verts[1].Set(5.031f, 0.340f);
            verts[2].Set(5.031f, 0.281f);
            verts[3].Set(5.199f, 0.281f);

			canon_11.Set(verts, 4);

			b2PolygonShape canon_12;
            verts[0].Set(5.272f, 0.522f);
            verts[1].Set(5.199f, 0.522f);
            verts[2].Set(5.199f, 0.281f);
            verts[3].Set(5.272f, 0.281f);

			canon_12.Set(verts, 4);

			b2PolygonShape canon_13;
            verts[0].Set(5.034f, 0.539f);
            verts[1].Set(5.004f, 0.551f);
            verts[2].Set(4.966f, 0.549f);
            verts[3].Set(4.934f, 0.540f);
            verts[4].Set(4.905f, 0.522f);
            verts[5].Set(5.034f, 0.522f);

			canon_13.Set(verts, 6);

			b2PolygonShape canon_14;
            verts[0].Set(5.034f, 0.281f);
            verts[1].Set(4.905f, 0.281f);
            verts[2].Set(4.934f, 0.263f);
            verts[3].Set(4.966f, 0.254f);
            verts[4].Set(5.004f, 0.252f);
            verts[5].Set(5.034f, 0.264f);

			canon_14.Set(verts, 6);

			b2PolygonShape canon_counterBalance;
            verts[0].Set(1.081f, 0.416f);
            verts[1].Set(0.981f, 0.416f);
            verts[2].Set(0.981f, 0.352f);
            verts[3].Set(1.081f, 0.352f);

			canon_counterBalance.Set(verts, 4);



			//  -------------------------------------------
			//               BODIES from POLYs
			//  -------------------------------------------
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
            float fixX = 0.0f;//161.0f;// -3.162
			float fixY = 1.8f;
			bd.position.Set(fixX, fixY);

            b2FixtureDef fd_main;
            fd_main.filter.categoryBits = 1;
            fd_main.filter.maskBits = 65505;// ~2,4,8,16
            fd_main.density = 1.0f;
            fd_main.shape = &main_chassis;

			m_chassis = m_world->CreateBody(&bd);
			m_chassis->CreateFixture(&fd_main);

            //  ----------------- FRONT CHASSIS & WELD JOINT --------------------
			//m_fChassis = m_world->CreateBody(&bd);
			//m_fChassis->CreateFixture(&front_chassis, 1.0f);

			b2FixtureDef fd_fMain;
            fd_fMain.filter.categoryBits = 1;
            fd_fMain.filter.maskBits = 65505;// ~2,4,8,16
            fd_fMain.density = 1.0f;
            fd_fMain.shape = &front_chassis;

			m_fChassis = m_world->CreateBody(&bd);
			m_fChassis->CreateFixture(&fd_fMain);

			b2WeldJointDef jtbd1;
			jtbd1.Initialize(m_chassis, m_fChassis, b2Vec2 (2.224f + fixX, -0.368f + fixY));
			m_fChassisJoint = (b2WeldJoint*)m_world->CreateJoint(&jtbd1);


            //  ----------------- EXHAUST & WELD JOINT --------------------
            b2FixtureDef fd_exhaust;
            fd_exhaust.filter.categoryBits = 1;
            fd_exhaust.filter.maskBits = 65505;// ~2,4,8,16
            fd_exhaust.density = 0.2f;
            fd_exhaust.shape = &exhaust;

			m_exhaust = m_world->CreateBody(&bd);
			m_exhaust->CreateFixture(&fd_exhaust);

			b2WeldJointDef jtbd2;
			jtbd2.Initialize(m_chassis, m_exhaust, b2Vec2 (-2.757f + fixX, 0.0f + fixY));
			m_exhaustJoint = (b2WeldJoint*)m_world->CreateJoint(&jtbd2);

            //  ----------------- FRONT PLATE & WELD JOINT --------------------
            b2FixtureDef f_fPlate;
            f_fPlate.shape = &fPlate;
            f_fPlate.density = 1.0f;
            f_fPlate.filter.categoryBits = 1;
            f_fPlate.filter.maskBits = 65505;// ~2,4,8,16


			m_fPlate = m_world->CreateBody(&bd);
			m_fPlate->CreateFixture(&f_fPlate);

			b2WeldJointDef jtbd3;
			jtbd3.Initialize(m_chassis, m_fPlate, b2Vec2 (2.149f + fixX, 0.0f + fixY));
			m_fPlateJoint = (b2WeldJoint*)m_world->CreateJoint(&jtbd3);

            //  ----------------- MG34bow & WELD JOINT --------------------
            b2FixtureDef f_bow;
            f_bow.density = 1.0f;
            f_bow.filter.categoryBits = 1;
            f_bow.filter.maskBits = 65505;// ~2,4,8,16
            f_bow.shape = &MG34bow;

			m_bow = m_world->CreateBody(&bd);
			m_bow->CreateFixture(&f_bow);

			b2WeldJointDef jtbd4;
			jtbd4.Initialize(m_fPlate, m_bow, b2Vec2 (2.238f + fixX, -0.039f + fixY));
			m_bowJoint = (b2WeldJoint*)m_world->CreateJoint(&jtbd4);


            //  ----------------- MG34 (Counter balance?) & REVOLUTE JOINT --------------------
            b2FixtureDef f_MG34;
            f_MG34.shape = &MG34;
            f_MG34.filter.categoryBits = 1;
			f_MG34.filter.maskBits = 65535;
			f_MG34.density = 1.0f;
            f_MG34.friction = 0.25f;


			m_mg34 = m_world->CreateBody(&bd);
			m_mg34->CreateFixture(&f_MG34);

			b2FixtureDef f_MGcBal;
			f_MGcBal.filter.categoryBits = 1;
			f_MGcBal.filter.maskBits = 0;
			f_MGcBal.density = 78.720f;//80.6f;
			f_MGcBal.shape = &MG34counterBalance;

			m_mg34cBal = m_world->CreateBody(&bd);
			m_mg34cBal->CreateFixture(&f_MGcBal);

			b2WeldJointDef jtbd5;
			jtbd5.Initialize(m_mg34, m_mg34cBal, b2Vec2 (2.376f + fixX, -0.184f + fixY));
			m_mg34cBalJoint = (b2WeldJoint*)m_world->CreateJoint(&jtbd5);

			b2RevoluteJointDef rtbd1;
			rtbd1.lowerAngle = -0.436332313;// -25°
			rtbd1.upperAngle = 0.523598775;//  +30°
			rtbd1.enableLimit = true;
			rtbd1.motorSpeed = 0.0f;
			rtbd1.maxMotorTorque = 10.0f;
			rtbd1.enableMotor = false;
			rtbd1.Initialize(m_bow, m_mg34, b2Vec2(2.396 + fixX, -0.184f + fixY));

			m_mg34RevoluteJoint = (b2RevoluteJoint*)m_world->CreateJoint(&rtbd1);

            //  ----------------- SIDE MUD GUARD & WELD JOINT --------------------
            b2FixtureDef fd_mud_side;
            fd_mud_side.filter.categoryBits = 1;
            fd_mud_side.filter.maskBits = 65505;// ~2,4,8,16
            fd_mud_side.density = 1.0f;
            fd_mud_side.shape = &mud_side;

			m_mud_side = m_world->CreateBody(&bd);
			m_mud_side->CreateFixture(&fd_mud_side);

			b2WeldJointDef msbd;
			msbd.Initialize(m_chassis, m_mud_side, b2Vec2 (-2.757f + fixX, 0.0f + fixY));
			m_mud_sideJoint = (b2WeldJoint*)m_world->CreateJoint(&msbd);

            //  ----------------- REAR MUD GUARD & WELD JOINT --------------------
            b2FixtureDef fd_mud_rear;
            fd_mud_rear.filter.categoryBits = 1;
            fd_mud_rear.filter.maskBits = 65505;// ~2,4,8,16
            fd_mud_rear.density = 1.0f;
            fd_mud_rear.shape = &mud_rear;

			m_mud_rear = m_world->CreateBody(&bd);
			m_mud_rear->CreateFixture(&fd_mud_rear);

			b2WeldJointDef mrbd;
			mrbd.Initialize(m_chassis, m_mud_rear, b2Vec2 (-2.757f + fixX, 0.0f + fixY));
			m_mud_rearJoint = (b2WeldJoint*)m_world->CreateJoint(&mrbd);


            //  =====================================================================
            //                                  TURRET
            //  =====================================================================
            m_turretBase = m_world->CreateBody(&bd);
			m_turretBase->CreateFixture(&turretBase, 1.0f);

            b2DistanceJointDef jtd;
			jtd.collideConnected = true;
			jtd.length = 0.01f;
			jtd.Initialize(m_chassis, m_turretBase, b2Vec2(0.0f + fixX, -0.010f + fixY), b2Vec2(0.0 + fixX, 0.010f + fixY));
			m_turretJoint = (b2DistanceJoint*)m_world->CreateJoint(&jtd);

            b2FixtureDef fd;
            fd.density = 0.6f;

            fd.shape = &turretMain;
			m_turretBase->CreateFixture(&fd);

			fd.shape = &turretHatch_0;
			m_turretBase->CreateFixture(&fd);

			fd.shape = &turretHatch_1;
			m_turretBase->CreateFixture(&fd);

			fd.shape = &turretRearBox;
			m_turretBase->CreateFixture(&fd);



            m_88_Canon = m_world->CreateBody(&bd);
			m_88_Canon->CreateFixture(&turretMantle, 1.0f);

            b2RevoluteJointDef rj_canon88;
			rj_canon88.lowerAngle = -0.113446401;// -6.5°
			rj_canon88.upperAngle = 0.296705972;//  +17°
			rj_canon88.enableLimit = true;
			rj_canon88.motorSpeed = 0.0f;
			rj_canon88.maxMotorTorque = 20.0f;
			rj_canon88.enableMotor = true;
			rj_canon88.Initialize(m_turretBase, m_88_Canon, b2Vec2(1.160 + fixX, 0.384f + fixY));

			m_88_RevoluteJoint = (b2RevoluteJoint*)m_world->CreateJoint(&rj_canon88);

            fd.shape = &canon_00;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_01;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_02;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_03;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_04;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_05;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_06;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_07;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_08;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_09;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_10;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_11;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_12;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_13;
			m_88_Canon->CreateFixture(&fd);

			fd.shape = &canon_14;
			m_88_Canon->CreateFixture(&fd);

			fd.density = 1145.5395f;

			fd.shape = &canon_counterBalance;
			m_88_Canon->CreateFixture(&fd);



			//  ---------------------- WHEELS & SPROCKETS -------------------------
			b2CircleShape sprocket;
			//sprocket.m_radius = 0.365f;
			sprocket.m_radius = 0.3835f;

			b2FixtureDef fd_sprocket;
			fd_sprocket.shape = &sprocket;
			fd_sprocket.density = 6.0f;
			fd_sprocket.friction = 10.0f;
			fd_sprocket.filter.categoryBits = 4;
			fd_sprocket.filter.maskBits = 65511;

			fd_sprocket.restitution = 0.05f;

			bd.position.Set(-2.317f + fixX, -1.175f + fixY);
			m_rSprocket = m_world->CreateBody(&bd);
			m_rSprocket->CreateFixture(&fd_sprocket);

            bd.position.Set(2.591f + fixX, -1.037f + fixY);
			m_fSprocket = m_world->CreateBody(&bd);
			m_fSprocket->CreateFixture(&fd_sprocket);
/*
			//  ------------------------- DRIVE TEETH -----------------------------
			b2PolygonShape tooth;

            bd.position.Set(fixX, fixY);

			b2FixtureDef teeth;
			teeth.shape = &tooth;
			teeth.friction = 1.0f;
			teeth.density = 100.0f;
			teeth.filter.categoryBits = 4;
			teeth.filter.maskBits = 65511;

			teeth.restitution = 0.05f;

            b2WeldJointDef tooth_jd;

            verts[0].Set(2.5870f, -0.6530f);
            verts[1].Set(2.5762f, -0.6538f);
            verts[2].Set(2.5666f, -0.6436f);
            verts[3].Set(2.5670f, -0.6538f);
            verts[4].Set(2.5874f, -0.6530f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[0] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[0]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[0], b2Vec2 (2.5772f + fixX, -0.6534f + fixY));
			f_teeth_joint[0] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.7151f, -0.6630f);
            verts[1].Set(2.7105f, -0.6431f);
            verts[2].Set(2.6956f, -0.6570f);
            verts[3].Set(2.6926f, -0.6668f);
            verts[4].Set(2.7121f, -0.6727f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[1] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[1]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[1], b2Vec2 (2.7024f + fixX, -0.6698f + fixY));
			f_teeth_joint[1] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.8318f, -0.7033f);
            verts[1].Set(2.8131f, -0.7116f);
            verts[2].Set(2.8072f, -0.7199f);
            verts[3].Set(2.8237f, -0.7319f);
            verts[4].Set(2.8297f, -0.7236f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[2] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[2]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[2], b2Vec2 (2.8154f + fixX, -0.7259f + fixY));
			f_teeth_joint[2] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9270f, -0.7996f);
            verts[1].Set(2.9066f, -0.8015f);
            verts[2].Set(2.8983f, -0.8073f);
            verts[3].Set(2.9100f, -0.8240f);
            verts[4].Set(2.9184f, -0.8181f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[3] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[3]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[3], b2Vec2 (2.9042f + fixX, -0.8157f + fixY));
			f_teeth_joint[3] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9857f, -0.9216f);
            verts[1].Set(2.9659f, -0.9168f);
            verts[2].Set(2.9561f, -0.9196f);
            verts[3].Set(2.9618f, -0.9392f);
            verts[4].Set(2.9716f, -0.9363f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[4] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[4]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[4], b2Vec2 (2.9589f + fixX, -0.9294f + fixY));
			f_teeth_joint[4] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9845f, -1.0451f);
            verts[1].Set(2.9743f, -1.0446f);
            verts[2].Set(2.9733f, -1.0650f);
            verts[3].Set(2.9835f, -1.0654f);
            verts[4].Set(3.0017f, -1.0561f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[5] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[5]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[5], b2Vec2 (2.9738f + fixX, -1.0548f + fixY));
			f_teeth_joint[5] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9604f, -1.1724f);
            verts[1].Set(2.9509f, -1.1687f);
            verts[2].Set(2.9434f, -1.1876f);
            verts[3].Set(2.9529f, -1.1914f);
            verts[4].Set(2.9731f, -1.1885f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[6] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[6]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[6], b2Vec2 (2.9471f + fixX, -1.1782f + fixY));
			f_teeth_joint[6] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.8963f, -1.2851f);
            verts[1].Set(2.8885f, -1.2785f);
            verts[2].Set(2.8752f, -1.2940f);
            verts[3].Set(2.8830f, -1.3006f);
            verts[4].Set(2.9031f, -1.3044f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[7] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[7]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[7], b2Vec2 (2.8819f + fixX, -1.2862f + fixY));
			f_teeth_joint[7] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.7990f, -1.3708f);
            verts[1].Set(2.7939f, -1.3621f);
            verts[2].Set(2.7763f, -1.3724f);
            verts[3].Set(2.7814f, -1.3812f);
            verts[4].Set(2.7992f, -1.3913f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[8] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[8]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[8], b2Vec2 (2.7851f + fixX, -1.3672f + fixY));
			f_teeth_joint[8] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.6772f, -1.4104f);
            verts[1].Set(2.6572f, -1.4144f);
            verts[2].Set(2.6592f, -1.4224f);
            verts[3].Set(2.6728f, -1.4398f);
            verts[4].Set(2.6792f, -1.4204f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[9] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[9]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[9], b2Vec2 (2.6672f + fixX, -1.4124f + fixY));
			f_teeth_joint[9] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.5512f, -1.4182f);
            verts[1].Set(2.5309f, -1.4156f);
            verts[2].Set(2.5296f, -1.4257f);
            verts[3].Set(2.5374f, -1.4445f);
            verts[4].Set(2.5498f, -1.4283f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[10] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[10]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[10], b2Vec2 (2.5410f + fixX, -1.4169f + fixY));
			f_teeth_joint[10] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.4294f, -1.3847f);
            verts[1].Set(2.4111f, -1.3756f);
            verts[2].Set(2.4066f, -1.3847f);
            verts[3].Set(2.4079f, -1.4051f);
            verts[4].Set(2.4249f, -1.3938f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[11] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[11]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[11], b2Vec2 (2.4203f + fixX, -1.3801f + fixY));
			f_teeth_joint[11] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.3251f, -1.3134f);
            verts[1].Set(2.3108f, -1.2989f);
            verts[2].Set(2.3035f, -1.3061f);
            verts[3].Set(2.2981f, -1.3258f);
            verts[4].Set(2.3179f, -1.3206f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[12] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[12]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[12], b2Vec2 (2.3180f + fixX, -1.3062f + fixY));
			f_teeth_joint[12] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.2408f, -1.1938f);
            verts[1].Set(2.2316f, -1.1982f);
            verts[2].Set(2.2201f, -1.2151f);
            verts[3].Set(2.2405f, -1.2166f);
            verts[4].Set(2.2497f, -1.2122f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[13] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[13]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[13], b2Vec2 (2.2452f + fixX, -1.2030f + fixY));
			f_teeth_joint[13] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.2087f, -1.0717f);
            verts[1].Set(2.1986f, -1.0729f);
            verts[2].Set(2.1822f, -1.0851f);
            verts[3].Set(2.2010f, -1.0931f);
            verts[4].Set(2.2111f, -1.0919f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[14] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[14]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[14], b2Vec2 (2.2099f + fixX, -1.0818f + fixY));
			f_teeth_joint[14] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.2181f, -0.9457f);
            verts[1].Set(2.2081f, -0.9436f);
            verts[2].Set(2.1886f, -0.9498f);
            verts[3].Set(2.2038f, -0.9635f);
            verts[4].Set(2.2137f, -0.9657f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[15] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[15]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[15], b2Vec2 (2.2159f + fixX, -0.9557f + fixY));
			f_teeth_joint[15] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);


			verts[0].Set(2.2678f, -0.8296f);
            verts[1].Set(2.2590f, -0.8244f);
            verts[2].Set(2.2386f, -0.8240f);
            verts[3].Set(2.2485f, -0.8418f);
            verts[4].Set(2.2572f, -0.8471f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[16] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[16]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[16], b2Vec2 (2.2625f + fixX, -0.8384f + fixY));
			f_teeth_joint[16] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.3525f, -0.7360f);
            verts[1].Set(2.3459f, -0.7282f);
            verts[2].Set(2.3267f, -0.7211f);
            verts[3].Set(2.3303f, -0.7413f);
            verts[4].Set(2.3368f, -0.7491f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[17] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[17]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[17], b2Vec2 (2.3446f + fixX, -0.7425f + fixY));
			f_teeth_joint[17] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.4593f, -0.6654f);
            verts[1].Set(2.4435f, -0.6525f);
            verts[2].Set(2.4403f, -0.6727f);
            verts[3].Set(2.4440f, -0.6822f);
            verts[4].Set(2.4630f, -0.6749f);

            tooth.Set(verts, 5);

            m_driveSprocketTeeth[18] = m_world->CreateBody(&bd);
            m_driveSprocketTeeth[18]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_fSprocket, m_driveSprocketTeeth[18], b2Vec2 (2.4535f + fixX, -0.6786f + fixY));
			f_teeth_joint[18] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //                                      REAR SPROCKET TEETH
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            float teeth_dx = -4.9080f;
            float teeth_dy = -0.1389f;

            verts[0].Set(2.5877f + teeth_dx, -0.6618f + teeth_dy);
            verts[1].Set(2.5769f + teeth_dx, -0.6445f + teeth_dy);
            verts[2].Set(2.5673f + teeth_dx, -0.6625f + teeth_dy);
            verts[3].Set(2.5677f + teeth_dx, -0.6727f + teeth_dy);
            verts[4].Set(2.5880f + teeth_dx, -0.6720f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[0] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[0]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[0], b2Vec2 (2.5778f + fixX + teeth_dx, -0.06723f + fixY + teeth_dy));
			r_teeth_joint[0] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.7096f + teeth_dx, -0.6811f + teeth_dy);
            verts[1].Set(2.7050f + teeth_dx, -0.6612f + teeth_dy);
            verts[2].Set(2.6900f + teeth_dx, -0.6752f + teeth_dy);
            verts[3].Set(2.6871f + teeth_dx, -0.6849f + teeth_dy);
            verts[4].Set(2.7066f + teeth_dx, -0.6909f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[1] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[1]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[1], b2Vec2 (2.6968f + fixX + teeth_dx, -0.06879f + fixY + teeth_dy));
			r_teeth_joint[1] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.8186f + teeth_dx, -0.7389f + teeth_dy);
            verts[1].Set(2.8207f + teeth_dx, -0.7186f + teeth_dy);
            verts[2].Set(2.8020f + teeth_dx, -0.7270f + teeth_dy);
            verts[3].Set(2.7961f + teeth_dx, -0.7353f + teeth_dy);
            verts[4].Set(2.8126f + teeth_dx, -0.7472f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[2] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[2]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[2], b2Vec2 (2.8043f + fixX + teeth_dx, -0.7412f + fixY + teeth_dy));
			r_teeth_joint[2] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9028f + teeth_dx, -0.8290f + teeth_dy);
            verts[1].Set(2.9115f + teeth_dx, -0.8105f + teeth_dy);
            verts[2].Set(2.8911f + teeth_dx, -0.8124f + teeth_dy);
            verts[3].Set(2.8828f + teeth_dx, -0.8183f + teeth_dy);
            verts[4].Set(2.8945f + teeth_dx, -0.8349f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[3] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[3]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[3], b2Vec2 (2.8887f + fixX + teeth_dx, -0.8266f + fixY + teeth_dy));
			r_teeth_joint[3] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9534f + teeth_dx, -0.9416f + teeth_dy);
            verts[1].Set(2.9675f + teeth_dx, -0.9269f + teeth_dy);
            verts[2].Set(2.9477f + teeth_dx, -0.9221f + teeth_dy);
            verts[3].Set(2.9379f + teeth_dx, -0.9249f + teeth_dy);
            verts[4].Set(2.9436f + teeth_dx, -0.9445f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[4] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[4]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[4], b2Vec2 (2.9379f + fixX + teeth_dx, -0.9347f + fixY + teeth_dy));
			r_teeth_joint[4] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9646f + teeth_dx, -1.0645f + teeth_dy);
            verts[1].Set(2.9827f + teeth_dx, -1.0552f + teeth_dy);
            verts[2].Set(2.9655f + teeth_dx, -1.0442f + teeth_dy);
            verts[3].Set(2.9553f + teeth_dx, -1.0437f + teeth_dy);
            verts[4].Set(2.9544f + teeth_dx, -1.0641f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[5] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[5]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[5], b2Vec2 (2.9549f + fixX + teeth_dx, -1.0539f + fixY + teeth_dy));
			r_teeth_joint[5] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.9353f + teeth_dx, -1.1844f + teeth_dy);
            verts[1].Set(2.9444f + teeth_dx, -1.1815f + teeth_dy);
            verts[2].Set(2.9428f + teeth_dx, -1.1654f + teeth_dy);
            verts[3].Set(2.9333f + teeth_dx, -1.1617f + teeth_dy);
            verts[4].Set(2.9258f + teeth_dx, -1.1806f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[6] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[6]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[6], b2Vec2 (2.9295f + fixX + teeth_dx, -1.1712f + fixY + teeth_dy));
			r_teeth_joint[6] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.8686f + teeth_dx, -1.2883f + teeth_dy);
            verts[1].Set(2.8887f + teeth_dx, -1.2920f + teeth_dy);
            verts[2].Set(2.8819f + teeth_dx, -1.2728f + teeth_dy);
            verts[3].Set(2.8741f + teeth_dx, -1.2661f + teeth_dy);
            verts[4].Set(2.8609f + teeth_dx, -1.2816f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[7] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[7]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[7], b2Vec2 (2.9295f + fixX + teeth_dx, -1.1712f + fixY + teeth_dy));
			r_teeth_joint[7] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.7718f + teeth_dx, -1.3649f + teeth_dy);
            verts[1].Set(2.7896f + teeth_dx, -1.3749f + teeth_dy);
            verts[2].Set(2.7894f + teeth_dx, -1.3535f + teeth_dy);
            verts[3].Set(2.7843f + teeth_dx, -1.3457f + teeth_dy);
            verts[4].Set(2.7667f + teeth_dx, -1.3561f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[8] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[8]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[8], b2Vec2 (2.7755f + fixX + teeth_dx, -1.3509f + fixY + teeth_dy));
			r_teeth_joint[8] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.6555f + teeth_dx, -1.4059f + teeth_dy);
            verts[1].Set(2.6690f + teeth_dx, -1.4212f + teeth_dy);
            verts[2].Set(2.6754f + teeth_dx, -1.4018f + teeth_dy);
            verts[3].Set(2.6734f + teeth_dx, -1.3918f + teeth_dy);
            verts[4].Set(2.6534f + teeth_dx, -1.3959f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[9] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[9]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[9], b2Vec2 (2.6634f + fixX + teeth_dx, -1.3938f + fixY + teeth_dy));
			r_teeth_joint[9] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.5321f + teeth_dx, -1.4069f + teeth_dy);
            verts[1].Set(2.5399f + teeth_dx, -1.4258f + teeth_dy);
            verts[2].Set(2.5523f + teeth_dx, -1.4095f + teeth_dy);
            verts[3].Set(2.5536f + teeth_dx, -1.3994f + teeth_dy);
            verts[4].Set(2.5334f + teeth_dx, -1.3968f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[10] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[10]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[10], b2Vec2 (2.5435f + fixX + teeth_dx, -1.3981f + fixY + teeth_dy));
			r_teeth_joint[10] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.4150f + teeth_dx, -1.3678f + teeth_dy);
            verts[1].Set(2.4163f + teeth_dx, -1.3882f + teeth_dy);
            verts[2].Set(2.4333f + teeth_dx, -1.3768f + teeth_dy);
            verts[3].Set(2.4378f + teeth_dx, -1.3677f + teeth_dy);
            verts[4].Set(2.4196f + teeth_dx, -1.3586f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[11] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[11]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[11], b2Vec2 (2.4287f + fixX + teeth_dx, -1.3632f + fixY + teeth_dy));
			r_teeth_joint[11] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.3170f + teeth_dx, -1.2928f + teeth_dy);
            verts[1].Set(2.3116f + teeth_dx, -1.3125f + teeth_dy);
            verts[2].Set(2.3314f + teeth_dx, -1.3073f + teeth_dy);
            verts[3].Set(2.3386f + teeth_dx, -1.3001f + teeth_dy);
            verts[4].Set(2.3243f + teeth_dx, -1.2856f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[12] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[12]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[12], b2Vec2 (2.3315f + fixX + teeth_dx, -1.2929f + fixY + teeth_dy));
			r_teeth_joint[12] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.2487f + teeth_dx, -1.1900f + teeth_dy);
            verts[1].Set(2.2372f + teeth_dx, -1.2069f + teeth_dy);
            verts[2].Set(2.2575f + teeth_dx, -1.2084f + teeth_dy);
            verts[3].Set(2.2667f + teeth_dx, -1.2040f + teeth_dy);
            verts[4].Set(2.2579f + teeth_dx, -1.1856f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[13] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[13]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[13], b2Vec2 (2.2623f + fixX + teeth_dx, -1.1948f + fixY + teeth_dy));
			r_teeth_joint[13] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.2174f + teeth_dx, -1.0706f + teeth_dy);
            verts[1].Set(2.2010f + teeth_dx, -1.0829f + teeth_dy);
            verts[2].Set(2.2198f + teeth_dx, -1.0909f + teeth_dy);
            verts[3].Set(2.2300f + teeth_dx, -1.0897f + teeth_dy);
            verts[4].Set(2.2275f + teeth_dx, -1.0694f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[14] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[14]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[14], b2Vec2 (2.2288f + fixX + teeth_dx, -1.0796f + fixY + teeth_dy));
			r_teeth_joint[14] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.2266f + teeth_dx, -0.9476f + teeth_dy);
            verts[1].Set(2.2072f + teeth_dx, -0.9538f + teeth_dy);
            verts[2].Set(2.2223f + teeth_dx, -0.9675f + teeth_dy);
            verts[3].Set(2.2323f + teeth_dx, -0.9697f + teeth_dy);
            verts[4].Set(2.2366f + teeth_dx, -0.9497f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[15] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[15]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[15], b2Vec2 (2.2344f + fixX + teeth_dx, -0.9597f + fixY + teeth_dy));
			r_teeth_joint[15] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);


			verts[0].Set(2.2753f + teeth_dx, -0.8342f + teeth_dy);
            verts[1].Set(2.2548f + teeth_dx, -0.8337f + teeth_dy);
            verts[2].Set(2.2647f + teeth_dx, -0.8516f + teeth_dy);
            verts[3].Set(2.2734f + teeth_dx, -0.8569f + teeth_dy);
            verts[4].Set(2.2840f + teeth_dx, -0.8394f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[16] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[16]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[16], b2Vec2 (2.2787f + fixX + teeth_dx, -0.8482f + fixY + teeth_dy));
			r_teeth_joint[16] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.3581f + teeth_dx, -0.7427f + teeth_dy);
            verts[1].Set(2.3389f + teeth_dx, -0.7357f + teeth_dy);
            verts[2].Set(2.3424f + teeth_dx, -0.7558f + teeth_dy);
            verts[3].Set(2.3490f + teeth_dx, -0.7636f + teeth_dy);
            verts[4].Set(2.3646f + teeth_dx, -0.7505f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[17] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[17]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[17], b2Vec2 (2.3568f + fixX + teeth_dx, -0.7571f + fixY + teeth_dy));
			r_teeth_joint[17] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);

			verts[0].Set(2.4661f + teeth_dx, -0.6831f + teeth_dy);
            verts[1].Set(2.4503f + teeth_dx, -0.6702f + teeth_dy);
            verts[2].Set(2.4471f + teeth_dx, -0.6904f + teeth_dy);
            verts[3].Set(2.4507f + teeth_dx, -0.6999f + teeth_dy);
            verts[4].Set(2.4698f + teeth_dx, -0.6926f + teeth_dy);

            tooth.Set(verts, 5);

            m_rearSprocketTeeth[18] = m_world->CreateBody(&bd);
            m_rearSprocketTeeth[18]->CreateFixture(&teeth);

			tooth_jd.Initialize(m_rSprocket, m_rearSprocketTeeth[18], b2Vec2 (2.4603f + fixX + teeth_dx, -0.6963f + fixY + teeth_dy));
			r_teeth_joint[18] = (b2WeldJoint*)m_world->CreateJoint(&tooth_jd);
*/


            //  -------------------------------------------------------
            //                      OUTER WHEELS
            //  -------------------------------------------------------
			b2CircleShape suspensionWheel;
			suspensionWheel.m_radius = 0.399;

			b2FixtureDef f_sWheel1;
			f_sWheel1.shape = &suspensionWheel;
			f_sWheel1.density = 1.0f;
			f_sWheel1.friction = 0.75f;
			f_sWheel1.filter.categoryBits = 8;
			f_sWheel1.filter.maskBits = 65515;


			f_sWheel1.restitution = 0.05f;


			bd.position.Set(1.929f + fixX, -1.269f + fixY);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&f_sWheel1);

			bd.position.Set(0.901f + fixX, -1.269f + fixY);
			m_wheel3 = m_world->CreateBody(&bd);
			m_wheel3->CreateFixture(&f_sWheel1);

			bd.position.Set(-0.126f + fixX, -1.269f + fixY);
			m_wheel5 = m_world->CreateBody(&bd);
			m_wheel5->CreateFixture(&f_sWheel1);

			bd.position.Set(-1.154f + fixX, -1.269f + fixY);
			m_wheel7 = m_world->CreateBody(&bd);
			m_wheel7->CreateFixture(&f_sWheel1);

            //  -------------------------------------------------------
            //                      INNER WHEELS
            //  -------------------------------------------------------
			b2FixtureDef f_sWheel2;
			f_sWheel2.shape = &suspensionWheel;
			f_sWheel2.density = 1.0f;
			f_sWheel2.friction = 0.75f;
			f_sWheel2.filter.categoryBits = 16;
			f_sWheel2.filter.maskBits = 65523;

			f_sWheel2.restitution = 0.05f;

			bd.position.Set(1.415f + fixX, -1.269f + fixY);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&f_sWheel2);

			bd.position.Set(0.388f + fixX, -1.269f + fixY);
			m_wheel4 = m_world->CreateBody(&bd);
			m_wheel4->CreateFixture(&f_sWheel2);

			bd.position.Set(-0.640f + fixX, -1.269f + fixY);
			m_wheel6 = m_world->CreateBody(&bd);
			m_wheel6->CreateFixture(&f_sWheel2);

			bd.position.Set(-1.667f + fixX, -1.269f + fixY);
			m_wheel8 = m_world->CreateBody(&bd);
			m_wheel8->CreateFixture(&f_sWheel2);

			//  ----------------------------- WHEEL JOINTS -----------------------------
			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 200.0f;
			jd.enableMotor = true;
			jd.frequencyHz = 100.0f;//m_hz;
			jd.dampingRatio = 0.0f;//m_zeta;
			jd.Initialize(m_fChassis, m_fSprocket, m_fSprocket->GetPosition(), axis);
			m_springFront = (b2WheelJoint*)m_world->CreateJoint(&jd);


			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 200.0f;
			jd.enableMotor = true;
			jd.frequencyHz = 100.0f;//m_hz;
			jd.dampingRatio = 0.0f;//m_zeta;
			jd.Initialize(m_chassis, m_rSprocket, m_rSprocket->GetPosition(), axis);
			m_springRear = (b2WheelJoint*)m_world->CreateJoint(&jd);



			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;

			jd.Initialize(m_chassis, m_wheel1, m_wheel1->GetPosition(), axis);
			m_springWheels[0] = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_chassis, m_wheel2, m_wheel2->GetPosition(), axis);
			m_springWheels[1] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            jd.Initialize(m_chassis, m_wheel3, m_wheel3->GetPosition(), axis);
			m_springWheels[2] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            jd.Initialize(m_chassis, m_wheel4, m_wheel4->GetPosition(), axis);
			m_springWheels[3] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            jd.Initialize(m_chassis, m_wheel5, m_wheel5->GetPosition(), axis);
			m_springWheels[4] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            jd.Initialize(m_chassis, m_wheel6, m_wheel6->GetPosition(), axis);
			m_springWheels[5] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            jd.Initialize(m_chassis, m_wheel7, m_wheel7->GetPosition(), axis);
			m_springWheels[6] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            jd.Initialize(m_chassis, m_wheel8, m_wheel8->GetPosition(), axis);
			m_springWheels[7] = (b2WheelJoint*)m_world->CreateJoint(&jd);

            //  =================================================================================
            //                                      TRACK!
            //  =================================================================================
			b2PolygonShape tLink;

            bd.position.Set(fixX, fixY);

			b2FixtureDef track;
			track.shape = &tLink;
			track.friction = 10.0f;
			track.density = 48.0f;
			track.filter.categoryBits = 2;
			track.filter.maskBits = 65535;

			track.restitution = 0.05f;

            float add_x = 0.12624f;

			for (int f = 0; f < 31; f++)
            {
                verts[0].Set(-1.8248f + add_x * f, -0.8200f);
                verts[1].Set(-1.9102f + add_x * f, -0.8200f);
                verts[2].Set(-1.9302f + add_x * f, -0.8450f);
                verts[3].Set(-1.9102f + add_x * f, -0.8700f);
                verts[4].Set(-1.8248f + add_x * f, -0.8700f);
                verts[5].Set(-1.8048f + add_x * f, -0.8450f);

                tLink.Set(verts, 6);

                m_link[f] = m_world->CreateBody(&bd);
                m_link[f]->CreateFixture(&track);
            }


            verts[0].Set(2.1027f, -0.8153f);
            verts[1].Set(2.0773f, -0.7958f);
            verts[2].Set(1.9944f, -0.8163f);
            verts[3].Set(1.9809f, -0.8453f);
            verts[4].Set(2.0063f, -0.8648f);
            verts[5].Set(2.0893f, -0.8444f);

            tLink.Set(verts, 6);

            m_link[31] = m_world->CreateBody(&bd);
            m_link[31]->CreateFixture(&track);

            for (int f = 0; f < 3; f++)
            {
                float add_x_ = 0.11685f;
                float add_y_ = 0.04777f;

                verts[0].Set(2.2196f + f * add_x_, -0.7675f + f * add_y_);
                verts[1].Set(2.1916f + f * add_x_, -0.7520f + f * add_y_);
                verts[2].Set(2.1125f + f * add_x_, -0.7843f + f * add_y_);
                verts[3].Set(2.1035f + f * add_x_, -0.8150f + f * add_y_);
                verts[4].Set(2.1315f + f * add_x_, -0.8306f + f * add_y_);
                verts[5].Set(2.2105f + f * add_x_, -0.7983f + f * add_y_);

                tLink.Set(verts, 6);

                m_link[32 + f] = m_world->CreateBody(&bd);
                m_link[32 + f]->CreateFixture(&track);
            }
            verts[0].Set(2.5758f, -0.6424f);
            verts[1].Set(2.5513f, -0.6218f);
            verts[2].Set(2.4676f, -0.6385f);
            verts[3].Set(2.4528f, -0.6670f);
            verts[4].Set(2.4774f, -0.6876f);
            verts[5].Set(2.5611f, -0.6708f);

            tLink.Set(verts, 6);

            m_link[35] = m_world->CreateBody(&bd);
            m_link[35]->CreateFixture(&track);

            verts[0].Set(2.6855f, -0.6308f);
            verts[1].Set(2.6007f, -0.6200f);
            verts[2].Set(2.5777f, -0.6423f);
            verts[3].Set(2.5944f, -0.6696f);
            verts[4].Set(2.6792f, -0.6804f);
            verts[5].Set(2.7022f, -0.6581f);

            tLink.Set(verts, 6);

            m_link[36] = m_world->CreateBody(&bd);
            m_link[36]->CreateFixture(&track);

            verts[0].Set(2.7329f, -0.6449f);
            verts[1].Set(2.7040f, -0.6542f);
            verts[2].Set(2.7111f, -0.6899f);
            verts[3].Set(2.7880f, -0.7271f);
            verts[4].Set(2.8169f, -0.7133f);
            verts[5].Set(2.8098f, -0.6821f);

            tLink.Set(verts, 6);

            m_link[37] = m_world->CreateBody(&bd);
            m_link[37]->CreateFixture(&track);

            verts[0].Set(2.8502f, -0.7105f);
            verts[1].Set(2.8184f, -0.7144f);
            verts[2].Set(2.8152f, -0.7463f);
            verts[3].Set(2.8763f, -0.8060f);
            verts[4].Set(2.9081f, -0.8022f);
            verts[5].Set(2.9113f, -0.7703f);

            tLink.Set(verts, 6);

            m_link[38] = m_world->CreateBody(&bd);
            m_link[38]->CreateFixture(&track);

            verts[0].Set(2.9405f, -0.8101f);
            verts[1].Set(2.9092f, -0.8037f);
            verts[2].Set(2.8960f, -0.8329f);
            verts[3].Set(2.9348f, -0.9090f);
            verts[4].Set(2.9662f, -0.9154f);
            verts[5].Set(2.9794f, -0.8862f);

            tLink.Set(verts, 6);

            m_link[39] = m_world->CreateBody(&bd);
            m_link[39]->CreateFixture(&track);

            verts[0].Set(2.9945f, -0.9333f);
            verts[1].Set(2.9668f, -0.9172f);
            verts[2].Set(2.9450f, -0.9407f);
            verts[3].Set(2.9576f, -1.0252f);
            verts[4].Set(2.9853f, -1.0413f);
            verts[5].Set(3.0070f, -1.0178f);

            tLink.Set(verts, 6);

            m_link[40] = m_world->CreateBody(&bd);
            m_link[40]->CreateFixture(&track);

            verts[0].Set(3.0063f, -1.0673f);
            verts[1].Set(2.9852f, -1.0432f);
            verts[2].Set(2.9571f, -1.0585f);
            verts[3].Set(2.9421f, -1.1423f);
            verts[4].Set(2.9632f, -1.1667f);
            verts[5].Set(2.9914f, -1.1514f);

            tLink.Set(verts, 6);

            m_link[41] = m_world->CreateBody(&bd);
            m_link[41]->CreateFixture(&track);

            verts[0].Set(2.9750f, -1.1980f);
            verts[1].Set(2.9626f, -1.1685f);
            verts[2].Set(2.9311f, -1.1740f);
            verts[3].Set(2.8901f, -1.2490f);
            verts[4].Set(2.9024f, -1.2785f);
            verts[5].Set(2.9340f, -1.2730f);

            tLink.Set(verts, 6);

            m_link[42] = m_world->CreateBody(&bd);
            m_link[42]->CreateFixture(&track);

            verts[0].Set(2.9063f, -1.3119f);
            verts[1].Set(2.9013f, -1.2800f);
            verts[2].Set(2.8696f, -1.2752f);
            verts[3].Set(2.8069f, -1.3333f);
            verts[4].Set(2.8092f, -1.3652f);
            verts[5].Set(2.8408f, -1.3700f);

            tLink.Set(verts, 6);

            m_link[43] = m_world->CreateBody(&bd);
            m_link[43]->CreateFixture(&track);

            verts[0].Set(2.7996f, -1.3972f);
            verts[1].Set(2.8076f, -1.3662f);
            verts[2].Set(2.7791f, -1.3516f);
            verts[3].Set(2.7012f, -1.3866f);
            verts[4].Set(2.6932f, -1.4176f);
            verts[5].Set(2.7217f, -1.4323f);

            tLink.Set(verts, 6);

            m_link[44] = m_world->CreateBody(&bd);
            m_link[44]->CreateFixture(&track);

            for (int f = 0; f < 5; f++)
            {
                float add_x_ = -0.11685f;
                float add_y_ = -0.04777f;

                verts[0].Set(2.6826f + f * add_x_, -1.4469f + f * add_y_);
                verts[1].Set(2.6917f + f * add_x_, -1.4162f + f * add_y_);
                verts[2].Set(2.6637f + f * add_x_, -1.4006f + f * add_y_);
                verts[3].Set(2.5846f + f * add_x_, -1.4329f + f * add_y_);
                verts[4].Set(2.5756f + f * add_x_, -1.4637f + f * add_y_);
                verts[5].Set(2.6036f + f * add_x_, -1.4792f + f * add_y_);

                tLink.Set(verts, 6);

                m_link[45 + f] = m_world->CreateBody(&bd);
                m_link[45 + f]->CreateFixture(&track);
            }

            verts[0].Set(2.0967f, -1.6886f);
            verts[1].Set(2.1088f, -1.6590f);
            verts[2].Set(2.0826f, -1.6407f);
            verts[3].Set(2.0006f, -1.6648f);
            verts[4].Set(1.9885f, -1.6944f);
            verts[5].Set(2.0147f, -1.7127f);

            tLink.Set(verts, 6);

            m_link[50] = m_world->CreateBody(&bd);
            m_link[50]->CreateFixture(&track);

            verts[0].Set(1.9868f, -1.6939f);
            verts[1].Set(1.9668f, -1.6689f);
            verts[2].Set(1.8813f, -1.6689f);
            verts[3].Set(1.8613f, -1.6939f);
            verts[4].Set(1.8813f, -1.7189f);
            verts[5].Set(1.9668f, -1.7189f);

            tLink.Set(verts, 6);

            m_link[51] = m_world->CreateBody(&bd);
            m_link[51]->CreateFixture(&track);

            for (int f = 0; f < 28; f++)
            {
                verts[0].Set(1.8405f + f * -add_x, -1.6689f);
                verts[1].Set(1.7551f + f * -add_x, -1.6689f);
                verts[2].Set(1.7351f + f * -add_x, -1.6939f);
                verts[3].Set(1.7551f + f * -add_x, -1.7189f);
                verts[4].Set(1.8405f + f * -add_x, -1.7189f);
                verts[5].Set(1.8605f + f * -add_x, -1.6939f);

                tLink.Set(verts, 6);

                m_link[52 + f] = m_world->CreateBody(&bd);
                m_link[52 + f]->CreateFixture(&track);
            }

            verts[0].Set(-1.7746f, -1.6566f);
            verts[1].Set(-1.7984f, -1.6780f);
            verts[2].Set(-1.7827f, -1.7059f);
            verts[3].Set(-1.6984f, -1.7199f);
            verts[4].Set(-1.6746f, -1.6985f);
            verts[5].Set(-1.6903f, -1.6705f);

            tLink.Set(verts, 6);

            m_link[80] = m_world->CreateBody(&bd);
            m_link[80]->CreateFixture(&track);

            for (int f = 0; f < 5; f++)
            {
                float add_x_ = -0.12380f;
                float add_y_ = 0.02468f;

                verts[0].Set(-1.8136f + f * add_x_, -1.6476f + f * add_y_);
                verts[1].Set(-1.8974f + f * add_x_, -1.6309f + f * add_y_);
                verts[2].Set(-1.9219f + f * add_x_, -1.6515f + f * add_y_);
                verts[3].Set(-1.9071f + f * add_x_, -1.6799f + f * add_y_);
                verts[4].Set(-1.8234f + f * add_x_, -1.6967f + f * add_y_);
                verts[5].Set(-1.7989f + f * add_x_, -1.6760f + f * add_y_);

                tLink.Set(verts, 6);

                m_link[81 + f] = m_world->CreateBody(&bd);
                m_link[81 + f]->CreateFixture(&track);
            }

            verts[0].Set(-2.5055f, -1.4906f);
            verts[1].Set(-2.5339f, -1.5052f);
            verts[2].Set(-2.5259f, -1.5362f);
            verts[3].Set(-2.4480f, -1.5712f);
            verts[4].Set(-2.4195f, -1.5566f);
            verts[5].Set(-2.4275f, -1.5256f);

            tLink.Set(verts, 6);

            m_link[86] = m_world->CreateBody(&bd);
            m_link[86]->CreateFixture(&track);

            verts[0].Set(-2.5960f, -1.4143f);
            verts[1].Set(-2.6277f, -1.4191f);
            verts[2].Set(-2.6299f, -1.4510f);
            verts[3].Set(-2.5672f, -1.5090f);
            verts[4].Set(-2.5355f, -1.5042f);
            verts[5].Set(-2.5332f, -1.4722f);

            tLink.Set(verts, 6);

            m_link[87] = m_world->CreateBody(&bd);
            m_link[87]->CreateFixture(&track);

            verts[0].Set(-2.6575f, -1.3131f);
            verts[1].Set(-2.6891f, -1.3075f);
            verts[2].Set(-2.7014f, -1.3371f);
            verts[3].Set(-2.6604f, -1.4120f);
            verts[4].Set(-2.6288f, -1.4176f);
            verts[5].Set(-2.6165f, -1.3880f);

            tLink.Set(verts, 6);

            m_link[88] = m_world->CreateBody(&bd);
            m_link[88]->CreateFixture(&track);

            verts[0].Set(-2.6836f, -1.1976f);
            verts[1].Set(-2.7118f, -1.1823f);
            verts[2].Set(-2.7329f, -1.2064f);
            verts[3].Set(-2.7178f, -1.2905f);
            verts[4].Set(-2.6897f, -1.3058f);
            verts[5].Set(-2.6686f, -1.2817f);

            tLink.Set(verts, 6);

            m_link[89] = m_world->CreateBody(&bd);
            m_link[89]->CreateFixture(&track);

            verts[0].Set(-2.6934f, -1.0563f);
            verts[1].Set(-2.7211f, -1.0724f);
            verts[2].Set(-2.7336f, -1.1569f);
            verts[3].Set(-2.7118f, -1.1804f);
            verts[4].Set(-2.6841f, -1.1643f);
            verts[5].Set(-2.6716f, -1.0798f);

            tLink.Set(verts, 6);

            m_link[90] = m_world->CreateBody(&bd);
            m_link[90]->CreateFixture(&track);

            verts[0].Set(-2.6359f, -0.9427f);
            verts[1].Set(-2.6672f, -0.9492f);
            verts[2].Set(-2.7060f, -1.0253f);
            verts[3].Set(-2.6928f, -1.0545f);
            verts[4].Set(-2.6615f, -1.0480f);
            verts[5].Set(-2.6227f, -0.9719f);

            tLink.Set(verts, 6);

            m_link[91] = m_world->CreateBody(&bd);
            m_link[91]->CreateFixture(&track);

            verts[0].Set(-2.5452f, -0.8534f);
            verts[1].Set(-2.5769f, -0.8496f);
            verts[2].Set(-2.6380f, -0.9094f);
            verts[3].Set(-2.6347f, -0.9412f);
            verts[4].Set(-2.6030f, -0.9451f);
            verts[5].Set(-2.5419f, -0.8853f);

            tLink.Set(verts, 6);

            m_link[92] = m_world->CreateBody(&bd);
            m_link[92]->CreateFixture(&track);

            verts[0].Set(-2.4307f, -0.7976f);
            verts[1].Set(-2.4596f, -0.7838f);
            verts[2].Set(-2.5365f, -0.8211f);
            verts[3].Set(-2.5436f, -0.8523f);
            verts[4].Set(-2.5147f, -0.8661f);
            verts[5].Set(-2.4378f, -0.8288f);

            tLink.Set(verts, 6);

            m_link[93] = m_world->CreateBody(&bd);
            m_link[93]->CreateFixture(&track);

            verts[0].Set(-2.3045f, -0.7811f);
            verts[1].Set(-2.3275f, -0.7589f);
            verts[2].Set(-2.4123f, -0.7697f);
            verts[3].Set(-2.4289f, -0.7971f);
            verts[4].Set(-2.4059f, -0.8193f);
            verts[5].Set(-2.3212f, -0.8085f);

            tLink.Set(verts, 6);

            m_link[94] = m_world->CreateBody(&bd);
            m_link[94]->CreateFixture(&track);

            verts[0].Set(-2.1943f, -0.7773f);
            verts[1].Set(-2.2781f, -0.7606f);
            verts[2].Set(-2.3026f, -0.7812f);
            verts[3].Set(-2.2879f, -0.8096f);
            verts[4].Set(-2.2041f, -0.8263f);
            verts[5].Set(-2.1796f, -0.8057f);

            tLink.Set(verts, 6);

            m_link[95] = m_world->CreateBody(&bd);
            m_link[95]->CreateFixture(&track);

            verts[0].Set(-2.1543f, -0.7853f);
            verts[1].Set(-2.1788f, -0.8059f);
            verts[2].Set(-2.1641f, -0.8343f);
            verts[3].Set(-2.0803f, -0.8510f);
            verts[4].Set(-2.0558f, -0.8304f);
            verts[5].Set(-2.0705f, -0.8020f);

            tLink.Set(verts, 6);

            m_link[96] = m_world->CreateBody(&bd);
            m_link[96]->CreateFixture(&track);

            verts[0].Set(-1.9474f, -0.8178f);
            verts[1].Set(-2.0323f, -0.8080f);
            verts[2].Set(-2.0550f, -0.8306f);
            verts[3].Set(-2.0380f, -0.8577f);
            verts[4].Set(-1.9531f, -0.8675f);
            verts[5].Set(-1.9304f, -0.8450f);

            tLink.Set(verts, 6);

            m_link[97] = m_world->CreateBody(&bd);
            m_link[97]->CreateFixture(&track);

            //  ====================================================================================
            //                                  TRACK  JOINTS
            //  ====================================================================================
            b2RevoluteJointDef link_jd;

            //link_jd.collideConnected = true;
            //link_jd.length = 0.010f;
            //link_jd.dampingRatio = 1000000.0f;
            //link_jd.frequencyHz = 10.0f;

            float diff = 0.003f;
            for (int f = 0; f < 31; f++)
            {
                b2Vec2 anchor1(-1.8044f + 0.12624f * f, -0.845f + fixY);
                b2Vec2 anchor2(-1.8044f + diff + 0.12624f * f, -0.845f + fixY);

                int nextIndex = f + 1;

                //b2RevoluteJointDef rtbd1;
                //rtbd1.Initialize(m_bow, m_mg34, b2Vec2(2.396 + fixX, -0.184f + fixY));

                //m_mg34RevoluteJoint = (b2RevoluteJoint*)m_world->CreateJoint(&rtbd1);

				link_jd.Initialize(m_link[f], m_link[nextIndex], anchor1);
				m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
//*
            {
            b2Vec2 anchor1(2.1008f, -0.8158f + fixY);
            b2Vec2 anchor2(2.1053f, -0.8143f + fixY);

            link_jd.Initialize(m_link[31], m_link[32], b2Vec2(2.1031f + fixX, -0.8152f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.2177f, -0.7683f + fixY);
            b2Vec2 anchor2(2.2222f, -0.7665f + fixY);

            link_jd.Initialize(m_link[32], m_link[33], b2Vec2(2.2200f + fixX, -0.7674f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(2.3346f, -0.7205f + fixY);
            b2Vec2 anchor2(2.3390f, -0.7187f + fixY);

            link_jd.Initialize(m_link[33], m_link[34], b2Vec2(2.3368f + fixX, -0.7196f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(2.4514f, -0.6728f + fixY);
            b2Vec2 anchor2(2.4548f, -0.6666f + fixY);

            link_jd.Initialize(m_link[34], m_link[35], b2Vec2(2.4531f + fixX, -0.6695f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.5739f, -0.6428f + fixY);
            b2Vec2 anchor2(2.5797f, -0.6426f + fixY);

            link_jd.Initialize(m_link[35], m_link[36], b2Vec2(2.5768f + fixX, -0.6423f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.7002f, -0.6579f + fixY);
            b2Vec2 anchor2(2.7058f, -0.6596f + fixY);

            link_jd.Initialize(m_link[36], m_link[37], b2Vec2(2.7031f + fixX, -0.6584f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.8151f, -0.7125f + fixY);
            b2Vec2 anchor2(2.8199f, -0.7158f + fixY);

            link_jd.Initialize(m_link[37], m_link[38], b2Vec2(2.8177f + fixX, -0.7139f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(2.9066f, -0.8008f + fixY);
            b2Vec2 anchor2(2.9101f, -0.8055f + fixY);

            link_jd.Initialize(m_link[38], m_link[39], b2Vec2(2.9086f + fixX, -0.8029f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.9653f, -0.9136f + fixY);
            b2Vec2 anchor2(2.9671f, -0.9192f + fixY);

            link_jd.Initialize(m_link[39], m_link[40], b2Vec2(2.9665f + fixX, -0.9163f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.9850f, -1.0393f + fixY);
            b2Vec2 anchor2(2.9849f, -1.0452f + fixY);

            link_jd.Initialize(m_link[40], m_link[41], b2Vec2(2.9852f + fixX, -1.0422f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.9636f, -1.1647f + fixY);
            b2Vec2 anchor2(2.9617f, -1.1702f + fixY);

            link_jd.Initialize(m_link[41], m_link[42], b2Vec2(2.9629f + fixX, -1.1676f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.9034f, -1.2768f + fixY);
            b2Vec2 anchor2(2.8998f, -1.2814f + fixY);

            link_jd.Initialize(m_link[42], m_link[43], b2Vec2(2.9019f + fixX, -1.2793f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.8107f, -1.3638f + fixY);
            b2Vec2 anchor2(2.8058f, -1.3671f + fixY);

            link_jd.Initialize(m_link[43], m_link[44], b2Vec2(2.8084f + fixX, -1.3657f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.6950f, -1.4168f + fixY);
            b2Vec2 anchor2(2.6898f, -1.4169f + fixY);

            link_jd.Initialize(m_link[44], m_link[45], b2Vec2(2.6924f + fixX, -1.4169f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            for (int f = 0; f < 5; f++)
            {
                b2Vec2 anchor1(2.5774f + f * -0.1161f, -1.4629f + fixY + f * -0.0475f);
                b2Vec2 anchor2(2.5730f + f * -0.1161f, -1.4647f + fixY + f * -0.0475f);

                int nextIndex = f + 1;

				link_jd.Initialize(m_link[f + 45], m_link[nextIndex + 45], b2Vec2(2.5752f + fixX + f * -0.1161f,
                                                                      -1.4638f + fixY + f * -0.0475f));
				m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(2.1100f, -1.6540f + fixY);
            b2Vec2 anchor2(2.1069f, -1.6596f + fixY);

            link_jd.Initialize(m_link[49], m_link[50], b2Vec2(2.1085f + fixX, -1.6569f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(1.9904f, -1.6938f + fixY);
            b2Vec2 anchor2(1.9848f, -1.6939f + fixY);

            link_jd.Initialize(m_link[50], m_link[51], b2Vec2(1.9876f + fixX, -1.6941f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(1.8633f, -1.6939f + fixY);
            b2Vec2 anchor2(1.8585f, -1.6939f + fixY);

            link_jd.Initialize(m_link[51], m_link[52], b2Vec2(1.8609f + fixX, -1.6939f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            for (int f = 0; f < 27; f++)
            {
                b2Vec2 anchor1(1.7371f + f * -0.12624f, -1.6939f + fixY);
                b2Vec2 anchor2(1.7323f + f * -0.12624f, -1.6939f + fixY);

                int nextIndex = f + 1;

				link_jd.Initialize(m_link[f + 52], m_link[nextIndex + 52], b2Vec2(1.7347f + fixX - f * 0.12624f, -1.6939f + fixY));
				m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(-1.6713f, -1.6939f + fixY);
            b2Vec2 anchor2(-1.6766f, -1.6981f + fixY);

            link_jd.Initialize(m_link[79], m_link[80], b2Vec2(-1.6740f + fixX, -1.6962f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            {
            b2Vec2 anchor1(-1.7964f, -1.6783f + fixY);
            b2Vec2 anchor2(-1.8008f, -1.6757f + fixY);

            link_jd.Initialize(m_link[80], m_link[81], b2Vec2(-1.7986f + fixX, -1.6770f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

            for (int f = 0; f < 5; f++)
            {
                b2Vec2 anchor1(-1.9199f + f * -0.1230f, -1.6519f + fixY + f * 0.0245f);
                b2Vec2 anchor2(-1.9246f + f * -0.1230f, -1.6510f + fixY + f * 0.0245f);

                int nextIndex = f + 1;

				link_jd.Initialize(m_link[f + 81], m_link[nextIndex + 81], b2Vec2(-1.9223f + fixX - f * 0.1230f,
                                                                      -1.6514f + fixY + f * 0.0245f));
				m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.4151f, -1.5532f + fixY);
            b2Vec2 anchor2(-2.4213f, -1.5557f + fixY);

            link_jd.Initialize(m_link[85], m_link[86], b2Vec2(-2.4183f + fixX, -1.5547f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.5321f, -1.5060f + fixY);
            b2Vec2 anchor2(-2.5370f, -1.5028f + fixY);

            link_jd.Initialize(m_link[86], m_link[87], b2Vec2(-2.5347f + fixX, -1.5047f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.6262f, -1.4204f + fixY);
            b2Vec2 anchor2(-2.6298f, -1.4158f + fixY);

            link_jd.Initialize(m_link[87], m_link[88], b2Vec2(-2.6282f + fixX, -1.4183f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.6881f, -1.3093f + fixY);
            b2Vec2 anchor2(-2.6900f, -1.3038f + fixY);

            link_jd.Initialize(m_link[88], m_link[89], b2Vec2(-2.6894f + fixX, -1.3067f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.7114f, -1.1842f + fixY);
            b2Vec2 anchor2(-2.7115f, -1.1784f + fixY);

            link_jd.Initialize(m_link[89], m_link[90], b2Vec2(-2.7118f + fixX, -1.1813f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.6937f, -1.0583f + fixY);
            b2Vec2 anchor2(-2.6919f, -1.0527f + fixY);

            link_jd.Initialize(m_link[90], m_link[91], b2Vec2(-2.6931f + fixX, -1.0554f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.6368f, -0.9445f + fixY);
            b2Vec2 anchor2(-2.6333f, -0.9398f + fixY);

            link_jd.Initialize(m_link[91], m_link[92], b2Vec2(-2.6353f + fixX, -0.9420f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.5466f, -0.8548f + fixY);
            b2Vec2 anchor2(-2.5418f, -0.8515f + fixY);

            link_jd.Initialize(m_link[92], m_link[93], b2Vec2(-2.5444f + fixX, -0.8529f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.4325f, -0.7985f + fixY);
            b2Vec2 anchor2(-2.4269f, -0.7968f + fixY);

            link_jd.Initialize(m_link[93], m_link[94], b2Vec2(-2.4298f + fixX, -0.7973f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.3065f, -0.7814f + fixY);
            b2Vec2 anchor2(-2.3007f, -0.7816f + fixY);

            link_jd.Initialize(m_link[94], m_link[95], b2Vec2(-2.3036f + fixX, -0.7812f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.1816f, -0.8054f + fixY);
            b2Vec2 anchor2(-2.1769f, -0.8063f + fixY);

            link_jd.Initialize(m_link[95], m_link[96], b2Vec2(-2.1792f + fixX, -0.8058f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-2.0578f, -0.8300f + fixY);
            b2Vec2 anchor2(-2.0530f, -0.8308f + fixY);

            link_jd.Initialize(m_link[96], m_link[97], b2Vec2(-2.0554f + fixX, -0.8305f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }
            {
            b2Vec2 anchor1(-1.9324f, -0.8447f + fixY);
            b2Vec2 anchor2(-1.9282f, -0.8450f + fixY);

            link_jd.Initialize(m_link[97], m_link[0], b2Vec2(-1.9303f + fixX, -0.8450f + fixY));
            m_linkJoint = (b2RevoluteJoint*)m_world->CreateJoint(&link_jd);
            }

//*/

		}
	}

	void Keyboard(unsigned char key)
	{

		switch (key)
		{
		case 'a':
			m_springFront->SetMotorSpeed((m_speed += 1.0f));
			m_springRear->EnableMotor(false);
			for (int f = 0; f < 8; f++)
            {
                m_springWheels[f]->EnableMotor(false);
            }
			break;

		case 's':
			m_springFront->SetMotorSpeed((m_speed = 0.0f));

			m_springRear->EnableMotor(true);
			m_springRear->SetMotorSpeed((m_speed));
            for (int f = 0; f < 8; f++)
            {
                m_springWheels[f]->EnableMotor(true);
                m_springWheels[f]->SetMotorSpeed((m_speed));
            }
			break;

		case 'd':
			m_springFront->SetMotorSpeed((m_speed -= 1.0f));
			m_springRear->EnableMotor(false);
            for (int f = 0; f < 8; f++)
            {
                m_springWheels[f]->EnableMotor(false);
            }
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
            for (int f = 0; f < 8; f++)
            {
                m_springWheels[f]->SetSpringFrequencyHz(m_hz);
            }
			break;

		case 'e':
			m_hz += 1.0f;
            for (int f = 0; f < 8; f++)
            {
                m_springWheels[f]->SetSpringFrequencyHz(m_hz);
            }
			break;


        case 'j':
            if(m_88_RevoluteJoint->GetJointAngle() < 0.296705672)
            {
                m_88_RevoluteJoint->SetMotorSpeed(0.15f);
            }
            break;

        case 'i':
            if(m_88_RevoluteJoint->GetJointAngle() > -0.113446401)
            {
                m_88_RevoluteJoint->SetMotorSpeed(-0.15f);
            }
            break;

        case 'm':
            m_88_RevoluteJoint->SetMotorSpeed(0.0f);
            break;


        case 'f':
            m_88_RevoluteJoint->SetMotorSpeed(0.0f);

            if (m_shell != NULL)
            {
                m_world->DestroyBody(m_shell);
                m_shell = NULL;
            }

            b2CircleShape round20mm;
            round20mm.m_radius = 0.044f;

            b2FixtureDef fsd;
            fsd.shape = &round20mm;
            fsd.density = 20.0f;
            fsd.restitution = 0.05f;

            b2Vec2 posn = m_88_RevoluteJoint->GetAnchorB();
            float32 angle = m_88_RevoluteJoint->GetJointAngle() + m_turretBase->GetAngle();

            //posn.x += 4.212f * cos(angle);
            posn.x += 4.182f * cos(angle);
            //posn.y += 4.212f * sin(angle);
            posn.y += 4.182f * sin(angle);

            b2BodyDef bsd;
            bsd.type = b2_dynamicBody;
            bsd.bullet = true;
            bsd.position.Set(posn.x, posn.y);

            m_shell = m_world->CreateBody(&bsd);
            m_shell->CreateFixture(&fsd);
            m_shell->SetLinearVelocity(b2Vec2(200.0f * cos(angle), 200.0f * sin(angle)));

            break;
		}
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: reverse = a, lock wheels = s, forward = d, hz down = q, hz up = e");
		m_textLine += 15;
		if (m_speed < 0)
        {
            m_debugDraw.DrawString(5, m_textLine, "FORWARD: %g", abs(m_speed));
            m_textLine += 15;
        }
        else if (m_speed > 0)
        {
            m_debugDraw.DrawString(5, m_textLine, "REVERSE: %g", m_speed);
            m_textLine += 15;
        }
        else
        {
            m_debugDraw.DrawString(5, m_textLine, "STOPPED: %g", abs(m_speed));
            m_textLine += 15;
        }


		if (m_springRear->IsMotorEnabled())
        {
            m_debugDraw.DrawString(5, m_textLine, "BRAKES: ON");
        }
        else
        {
             m_debugDraw.DrawString(5, m_textLine, "BRAKES: OFF");
        }
		m_textLine += 15;

		m_debugDraw.DrawString(5, m_textLine, "Raise Canon = j, Lower Canon = i, Stop Canon = m, Fire 88 = f");
		m_textLine += 15;

		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;

		settings->viewCenter.x = m_chassis->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new TigerI;
	}

	b2Body *m_chassis;
	b2Body *m_fChassis;
	b2Body *m_exhaust;
	b2Body *m_fPlate;
	b2Body *m_bow;
	b2Body *m_mg34;
	b2Body *m_mg34cBal;

    b2Body *m_mud_side;
	b2Body *m_mud_rear;

    b2Body* m_turretBase;
    b2Body* m_turretMain;
    b2Body* m_88_Canon;

	b2Body * m_shell;

    b2Body *m_fSprocket;
    b2Body *m_rSprocket;

	b2Body* m_wheel1;
	b2Body* m_wheel2;
	b2Body* m_wheel3;
	b2Body* m_wheel4;
	b2Body* m_wheel5;
	b2Body* m_wheel6;
	b2Body* m_wheel7;
	b2Body* m_wheel8;

	b2Body* m_link[98];
	b2Body* m_driveSprocketTeeth[19];
	b2Body* m_rearSprocketTeeth[19];

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
	b2WheelJoint* m_springFront;
	b2WheelJoint* m_springRear;

	b2WheelJoint* m_springWheels[8];
	/*
	b2WheelJoint* m_springWheel1;
	b2WheelJoint* m_springWheel2;
	b2WheelJoint* m_springWheel3;
	b2WheelJoint* m_springWheel4;
	b2WheelJoint* m_springWheel5;
	b2WheelJoint* m_springWheel6;
	b2WheelJoint* m_springWheel7;
	b2WheelJoint* m_springWheel8;
	*/

    b2WeldJoint* m_fChassisJoint;
    b2WeldJoint* m_exhaustJoint;
    b2WeldJoint* m_fPlateJoint;
    b2WeldJoint* m_bowJoint;
    b2RevoluteJoint* m_mg34RevoluteJoint;
    b2WeldJoint* m_mg34cBalJoint;

    b2WeldJoint* m_mud_sideJoint;
    b2WeldJoint* m_mud_rearJoint;

    b2RevoluteJoint* m_linkJoint;

    b2DistanceJoint* m_turretJoint;
    b2RevoluteJoint* m_88_RevoluteJoint;

    b2WeldJoint* f_teeth_joint[19];
    b2WeldJoint* r_teeth_joint[19];

	//b2DistanceJoint* m_tBaseJoint;
	//b2DistanceJoint* m_turretJoint;
	//b2RevoluteJoint* m_cannonJoint;
};

#endif
